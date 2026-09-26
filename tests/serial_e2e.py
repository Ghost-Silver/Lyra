#!/usr/bin/env python3
"""tests/serial_e2e.py — E 批端到端：WS 指令 → arm_sim →（PTY 真机路径）→ 从机 → STATE_REP

验证「主线接线」而不只是单元：
  1. 用 posix_openpt 建 PTY；把 **slave 路径**交给 arm_sim（--serial <pts> --serial-role master）
  2. 本脚本扮演下位机（持 master 端）：解析 arm_sim 发来的 CMD_*，回复 STATE_REP
  3. 通过 WS 下发 joint_target → 断言：从机收到对应 CMD_POS 帧、数值正确
  4. 从机回复 STATE_REP → 断言：/api/health.serial_state_frames 递增（上行链路可用）
  5. 断言 /api/health 的 serial_* 指标（队列/写入字节/角色）非零或符合预期

用法： python3 tests/serial_e2e.py --spawn ./build-learn/arm_sim --port 8310
"""
import argparse
import json
import os
import pty
import select
import struct
import subprocess
import sys
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ws_probe import ws_handshake, ws_send_text  # noqa: E402

HOST = "127.0.0.1"
SOF = b"\xAA\x55"
fails = 0


def check(cond, tag, extra=""):
    global fails
    print(("PASS " if cond else "FAIL ") + tag + ((" | " + extra) if extra else ""))
    if not cond:
        fails += 1


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return SOF + body + bytes([crc & 0xFF, crc >> 8])


def decode(buf: bytearray):
    """从缓冲里取出一个完整帧，返回 (type, payload, badCrc) 或 None。"""
    while True:
        i = buf.find(SOF)
        if i < 0:
            if len(buf) > 1:
                del buf[:max(0, len(buf) - 1)]
            return None
        if i > 0:
            del buf[:i]
        if len(buf) < 4:
            return None
        ftype, ln = buf[2], buf[3]
        if len(buf) < 6 + ln:
            return None
        crc = buf[4 + ln] | (buf[5 + ln] << 8)
        body = bytes(buf[2:4 + ln])
        frame = bytes(buf[:6 + ln])
        del buf[:6 + ln]
        return (ftype, bytes(buf[0:0]) or frame[4:4 + ln], crc == crc16_ccitt(body))


def health(port):
    with urllib.request.urlopen(f"http://{HOST}:{port}/api/health", timeout=3) as r:
        return json.loads(r.read().decode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn", required=True)
    ap.add_argument("--port", type=int, default=8310)
    ap.add_argument("--web", default="web")
    args = ap.parse_args()

    master, slave = pty.openpty()
    sname = os.ttyname(slave)
    # 从机端 raw（行规程会缓冲二进制帧）
    import termios
    attrs = termios.tcgetattr(slave)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(slave, termios.TCSANOW, attrs)

    # stdbuf -o0：管道下 stdout 默认块缓冲，启动日志要即时可见（也便于人工观察）
    proc = subprocess.Popen(["stdbuf", "-o0", args.spawn, "--port", str(args.port), "--web", args.web,
                             "--serial", sname, "--serial-role", "master"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    # 后台读取启动日志（管道读必须持续消费，否则子进程会被写阻塞）
    startup_lines: list[str] = []

    def _drain():
        try:
            for line in proc.stdout:      # type: ignore[union-attr]
                startup_lines.append(line.rstrip())
        except Exception:
            pass

    threading.Thread(target=_drain, daemon=True).start()
    try:
        for _ in range(100):
            try:
                health(args.port)
                break
            except Exception:
                time.sleep(0.1)
        else:
            print("FAIL: arm_sim 未就绪")
            return 1

        # 启动行里应能看到串口已连接与角色
        startup = ""
        for _ in range(20):
            startup = "\n".join(startup_lines)
            if "[serial]" in startup:
                break
            time.sleep(0.1)
        check("[serial]" in startup and sname in startup,
              "启动日志确认串口已连接并标明角色",
              next((l for l in startup_lines if "[serial]" in l), "(无输出)"))

        buf = bytearray()
        got_pos = None

        # ---- WS 下发 joint_target ----
        ws, _ = ws_handshake(args.port)
        q_cmd = [0.11, -0.22, 0.33, -0.44, 0.55, -0.66]
        ws_send_text(ws, json.dumps({"type": "joint_target", "q": q_cmd, "speed": 0.5}))

        deadline = time.time() + 3
        while time.time() < deadline and got_pos is None:
            r, _, _ = select.select([master], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(master, 4096)
            except OSError:
                break
            buf.extend(data)
            while True:
                fr = decode(buf)
                if fr is None:
                    break
                ftype, payload, crc_ok = fr
                if ftype == 0x01:      # CMD_POS
                    vals = struct.unpack("<6h", payload[:12]) if len(payload) >= 12 else ()
                    got_pos = ([v / 1000.0 for v in vals], crc_ok)

        check(got_pos is not None, "WS joint_target 被转发为串口 CMD_POS 帧")
        if got_pos:
            vals, crc_ok = got_pos
            check(crc_ok, "CMD_POS 帧 CRC16-CCITT 校验通过")
            dev = max(abs(a - b) for a, b in zip(vals, q_cmd))
            check(dev < 2e-3, "CMD_POS 数值与 WS 指令一致（最大偏差 %.4f rad，量化 0.001）" % dev)

        # ---- 从机回复 STATE_REP ----
        h0 = health(args.port)
        payload = b"".join(struct.pack("<h", int(v * 1000)) for v in q_cmd)
        payload += b"".join(struct.pack("<h", 0) for _ in range(6))
        payload += bytes([128])                      # grip 0.5
        os.write(master, encode(0x10, payload))
        time.sleep(0.5)
        h1 = health(args.port)
        check(h1.get("serial_state_frames", 0) > h0.get("serial_state_frames", 0),
              "STATE_REP 被 arm_sim 解析并计数（/api/health.serial_state_frames）",
              f"{h0.get('serial_state_frames', 0)} → {h1.get('serial_state_frames', 0)}")
        check(h1.get("serial_role") == "master" and h1.get("serial_online") is True,
              "health 反映角色与在线状态",
              f"role={h1.get('serial_role')} online={h1.get('serial_online')}")
        check(h1.get("serial_tx_written", 0) > 0, "下行写入字节数被统计",
              f"written={h1.get('serial_tx_written')}")

        # ---- 急停 → CMD_ESTOP 帧 ----
        ws_send_text(ws, json.dumps({"type": "estop", "on": True}))
        got_estop = False
        deadline = time.time() + 2
        while time.time() < deadline and not got_estop:
            r, _, _ = select.select([master], [], [], 0.2)
            if not r:
                continue
            buf.extend(os.read(master, 4096))
            while True:
                fr = decode(buf)
                if fr is None:
                    break
                if fr[0] == 0x04 and fr[1] == b"\x01":
                    got_estop = True
        check(got_estop, "WS estop 被转发为 CMD_ESTOP(1) 帧")
        ws.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        os.close(master)
        os.close(slave)

    print(f"\nSERIAL E2E {'ALL PASS' if fails == 0 else f'FAIL({fails})'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
