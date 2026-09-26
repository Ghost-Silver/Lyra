#!/usr/bin/env python3
"""tests/ws_probe.py — C2 资源上限残留检查（三场景实测时延 + 资源占用）

场景：
  1) 连接数上限：第 17 个连接应收到 503（而非静默断开），并计入 /api/health.rejected_clients
  2) 半开连接回收：客户端 TCP 断开（RST）后，服务端回收时延 + ws_clients 计数回落时间
  3) 超长请求头：单行 32 KiB 请求头 → 服务端丢弃时延（kMaxHttpReq=16 KiB 上限）

另含：静态文件上限 413（>192 KiB）、静态缓存命中、symlink 逃逸回归。

用法（可自行拉起服务）：
  python3 tests/ws_probe.py --spawn ./build-learn/arm_sim --port 8123
  python3 tests/ws_probe.py --port 8080            # 或连到已运行的实例
"""
import argparse
import base64
import json
import os
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request

HOST = "127.0.0.1"
fails = 0


def check(cond, tag, extra=""):
    global fails
    print(("PASS " if cond else "FAIL ") + tag + ((" | " + extra) if extra else ""))
    if not cond:
        fails += 1


def health(port):
    with urllib.request.urlopen(f"http://{HOST}:{port}/api/health", timeout=3) as r:
        return json.loads(r.read().decode())


def ws_handshake(port, timeout=3):
    s = socket.create_connection((HOST, port), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    req = (f"GET /ws HTTP/1.1\r\nHost: {HOST}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
           f"Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {key}\r\n\r\n")
    s.sendall(req.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        c = s.recv(4096)
        if not c:
            break
        buf += c
    return s, buf.split(b"\r\n")[0].decode(errors="replace")


def ws_send_text(sock, text):
    """客户端 → 服务端必须掩码（RFC6455）。"""
    data = text.encode()
    hdr = bytearray([0x81])                      # FIN + text
    n = len(data)
    if n < 126:
        hdr.append(0x80 | n)
    elif n < 65536:
        hdr.append(0x80 | 126)
        hdr += struct.pack(">H", n)
    else:
        hdr.append(0x80 | 127)
        hdr += struct.pack(">Q", n)
    mask = os.urandom(4)
    hdr += mask
    sock.sendall(bytes(hdr) + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))


def ws_recv_frame(sock, timeout=3.0):
    """服务端 → 客户端无掩码。返回 (opcode, payload) 或 None。"""
    sock.settimeout(timeout)
    h = sock.recv(2)
    if len(h) < 2:
        return None
    opcode, ln = h[0] & 0x0F, h[1] & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", sock.recv(2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", sock.recv(8))[0]
    data = b""
    while len(data) < ln:
        c = sock.recv(ln - len(data))
        if not c:
            break
        data += c
    return opcode, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8123)
    ap.add_argument("--spawn", default=None, help="可选：自行拉起 arm_sim 的路径")
    ap.add_argument("--web", default="web")
    args = ap.parse_args()
    port = args.port

    proc = None
    if args.spawn:
        proc = subprocess.Popen([args.spawn, "--port", str(port), "--web", args.web],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(100):
            try:
                health(port)
                break
            except Exception:
                time.sleep(0.05)
        else:
            print("FAIL 服务未能启动")
            return 1

    try:
        # ---------- 场景 4：端到端注入越限 joint_target（B1 安全层拦截）----------
        s4, _st4 = ws_handshake(port)
        hpre = health(port)
        ws_send_text(s4, json.dumps({"type": "joint_target", "q": [5.0] * 6}))
        maxq = 0.0
        states = 0
        deadline = time.time() + 3
        while time.time() < deadline and states < 3:
            fr = ws_recv_frame(s4, 1.0)
            if not fr or fr[0] != 1:
                continue
            try:
                msg = json.loads(fr[1].decode())
            except Exception:
                continue
            if msg.get("type") == "state":
                states += 1
                maxq = max([maxq] + [abs(x) for x in msg.get("q", [])])
        hpost = health(port)
        # 说明（如实）：服务端**所有 WS 指令入口**都有 gate_.clampQ/clampV 前置夹回
        # （纵深防御第一层），越限指令在到达安全层之前已被夹回 —— 因此 safety_pos
        # 不递增是**预期行为**，不是缺陷。安全层自身的拦截验收（B1）由
        # tests/test_safety.cpp 直接注入 setJointPositionTarget（等价于绕过上层的路径）
        # 验证：posClamps 递增、旧规划器越限轨迹 400 拍零泄漏。
        check(states >= 1 and maxq <= 3.0,
              "场景4 越限 joint_target 经服务端后状态不越限（第一层 gate 夹回；安全层为纵深兜底）",
              f"收到 {states} 帧状态, max|q|={maxq:.4f} ≤ 2.967；safety_pos {hpre.get('safety_pos', 0)}"
              f" → {hpost.get('safety_pos', 0)}（上层已夹回，故不增）")
        check(hpost.get("safety_enabled") is True,
              "场景4 /api/health 暴露安全层状态（safety_enabled=true，healthExtra 注入生效）",
              f"safety_enabled={hpost.get('safety_enabled')}")
        s4.close()

        # ---------- 场景 3：超长请求头丢弃（占用 slot 前做）----------
        t0 = time.perf_counter()
        s3 = socket.create_connection((HOST, port), timeout=5)
        try:
            s3.sendall(b"GET / HTTP/1.1\r\nHost: x\r\nX-Big: " + b"A" * 32768 + b"\r\n\r\n")
            closed = False
            s3.settimeout(2.0)
            try:
                closed = (s3.recv(1024) == b"")   # FIN
            except ConnectionResetError:
                closed = True                     # RST 也算被丢弃（未读入站数据所致）
            except socket.timeout:
                closed = False
        finally:
            drop_ms = (time.perf_counter() - t0) * 1000
            s3.close()
        check(closed, "场景3 超长请求头（32 KiB > 16 KiB 上限）连接被丢弃",
              f"丢弃时延 {drop_ms:.1f} ms")

        # ---------- 附加：静态文件上限 413 / 缓存 / symlink 逃逸 ----------
        big = os.path.join(args.web, "_probe_big.bin")
        with open(big, "wb") as f:
            f.write(b"x" * (300 * 1024))   # 300 KiB > 192 KiB 上限
        try:
            code = 200
            try:
                with urllib.request.urlopen(f"http://{HOST}:{port}/_probe_big.bin", timeout=5) as r:
                    code = r.status
            except urllib.error.HTTPError as e:
                code = e.code
            check(code == 413, "附加 超限静态文件 → 413（保护控制回路）", f"code={code}")
        finally:
            os.unlink(big)

        hits0 = health(port).get("static_cache_hits", 0)
        misses0 = health(port).get("static_cache_misses", 0)
        for _ in range(3):
            with urllib.request.urlopen(f"http://{HOST}:{port}/index.html", timeout=5) as r:
                r.read()
        h = health(port)
        check(h.get("static_cache_hits", 0) > hits0, "附加 静态缓存命中递增（零磁盘 IO 路径生效）",
              f"hits {hits0} → {h.get('static_cache_hits')}, misses {misses0} → {h.get('static_cache_misses')}")

        link = os.path.join(args.web, "_probe_link.html")
        if not os.path.islink(link):
            try:
                os.symlink("/etc/passwd", link)
            except FileExistsError:
                pass
        try:
            code = 200
            try:
                with urllib.request.urlopen(f"http://{HOST}:{port}/_probe_link.html", timeout=5) as r:
                    code = r.status
            except urllib.error.HTTPError as e:
                code = e.code
            check(code == 403, "附加 symlink 逃逸仍为 403（无回归）", f"code={code}")
        finally:
            if os.path.islink(link):
                os.unlink(link)

        # ---------- 场景 2：半开连接回收（单连接，RST 断开）----------
        s1, _st = ws_handshake(port)
        time.sleep(0.2)
        before = health(port).get("ws_clients", 0)
        t0 = time.perf_counter()
        s1.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
        s1.close()   # close 即发 RST（模拟掉线/半开，不发 WS close 帧）
        reclaimed_ms = None
        for _ in range(600):   # 最多 6 s
            time.sleep(0.01)
            if health(port).get("ws_clients", 0) < before:
                reclaimed_ms = (time.perf_counter() - t0) * 1000
                break
        check(reclaimed_ms is not None, "场景2 半开连接被回收（ws_clients 计数回落）",
              (f"{before} → {health(port).get('ws_clients')}，RST→回收 {reclaimed_ms:.1f} ms"
               if reclaimed_ms is not None else "超时未回收"))

        # ---------- 场景 1：连接数上限（占满 16 后第 17 个）----------
        held = []
        for _ in range(16):
            sk, _st2 = ws_handshake(port)
            held.append(sk)
        time.sleep(0.3)
        t0 = time.perf_counter()
        s17 = socket.create_connection((HOST, port), timeout=3)
        s17.sendall(b"GET /api/health HTTP/1.1\r\nHost: x\r\n\r\n")
        resp = b""
        try:
            while True:
                c = s17.recv(4096)
                if not c:
                    break
                resp += c
        except (socket.timeout, ConnectionResetError):
            pass   # RST 兜底：503 可能已到达（下面按内容判定）
        dt_ms = (time.perf_counter() - t0) * 1000
        s17.close()
        first_line = resp.split(b"\r\n")[0].decode(errors="replace") if resp else "(无响应)"
        check("503" in first_line, "场景1 第 17 连接收到 503（前端可感知，非静默断开）",
              f"{first_line} | 时延 {dt_ms:.1f} ms")
        body_txt = resp.split(b"\r\n\r\n")[-1][:20].decode(errors="replace")
        check(b"busy" in resp, "场景1 503 带响应体 busy", "body=" + body_txt)

        for sk in held:
            try:
                sk.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
                sk.close()
            except Exception:
                pass
        time.sleep(0.5)
        h2 = health(port)
        check(h2.get("rejected_clients", 0) >= 1, "场景1 rejected_clients 计数递增",
              f"rejected_clients={h2.get('rejected_clients')}")
        check(h2.get("http_clients", 0) <= 16, "场景1 释放后活跃连接回落",
              f"http_clients={h2.get('http_clients')}, ws_clients={h2.get('ws_clients')}")
    finally:
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    print(f"\nC2 PROBE {'ALL PASS' if fails == 0 else f'FAIL({fails})'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
