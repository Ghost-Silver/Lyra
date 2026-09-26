#!/usr/bin/env python3
"""tests/soak_longrun.py — G1：长时运行的内存/资源曲线

分段（可调）：
  A 无客户端 / B 1 客户端 / C 4 客户端（并持续压测：静态文件拉取 + teach_add 洪水 + 指令）
每 `--sample` 秒采样一次 RSS（/proc/<pid>/status VmRSS）与 /api/health 计数，输出 CSV。

用法：
  python3 tests/soak_longrun.py --spawn ./build-learn/arm_sim --port 8210 --minutes 12
（--minutes 为**每段**分钟数；三段时间合计 = 3×minutes）
"""
import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ws_probe import ws_handshake, ws_send_text  # noqa: E402

HOST = "127.0.0.1"


def rss_kb(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except OSError:
        return -1
    return -1


def health(port):
    try:
        with urllib.request.urlopen(f"http://{HOST}:{port}/api/health", timeout=3) as r:
            return json.loads(r.read().decode())
    except Exception:
        return {}


def drain_sockets(socks):
    """排空已连接 WS 客户端的入站数据。

    设计使然：**不读取**的客户端会因发送队列（256 KiB）溢出被服务端丢弃——
    真实浏览器持续读取 50 Hz 状态帧，故压测客户端也必须读取，否则测到的是「连接 churn」
    而不是稳态负载（churn 本身也有意义：见 --churn 说明）。
    """
    for sk in list(socks):
        try:
            sk.setblocking(False)
            for _ in range(8):
                if not sk.recv(65536):
                    break
        except Exception:
            pass
        finally:
            try:
                sk.setblocking(False)   # 之后按非阻塞使用
            except Exception:
                pass


class Hammer(threading.Thread):
    """持续压测线程：拉静态文件 / teach_add 洪水 / 偶发指令。"""

    def __init__(self, port, kind):
        super().__init__(daemon=True)
        self.port, self.kind = port, kind
        self.stop = False
        self.ops = 0
        self.errors = 0

    def run(self):
        if self.kind == "http":
            while not self.stop:
                try:
                    with urllib.request.urlopen(f"http://{HOST}:{self.port}/index.html", timeout=5) as r:
                        r.read()
                    self.ops += 1
                except Exception:
                    self.errors += 1
                time.sleep(0.02)
        else:   # ws：teach_add 洪水 + 周期性 joint_target / ee_drag
            s, _ = ws_handshake(self.port)
            s.setblocking(False)
            i = 0
            while not self.stop:
                try:
                    try:                       # 读取入站（保持「健康客户端」语义）
                        while s.recv(65536):
                            pass
                    except Exception:
                        pass
                    if i % 4 == 3:
                        ws_send_text(s, json.dumps({"type": "joint_target",
                                                    "q": [0.2, -0.4, 0.3, 0.1, 0.2, 0.0], "speed": 0.5}))
                    else:
                        ws_send_text(s, json.dumps({"type": "teach_add"}))
                    self.ops += 1
                except Exception:
                    self.errors += 1
                i += 1
                time.sleep(0.05)
            try:
                s.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn", required=True, help="arm_sim 路径")
    ap.add_argument("--port", type=int, default=8210)
    ap.add_argument("--minutes", type=float, default=12.0, help="每段分钟数（共 3 段）")
    ap.add_argument("--sample", type=float, default=15.0, help="采样间隔（秒）")
    ap.add_argument("--csv", default="/tmp/lyra_soak.csv")
    args = ap.parse_args()
    port = args.port

    proc = subprocess.Popen([args.spawn, "--port", str(port), "--web", "web"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(200):
        if health(port):
            break
        time.sleep(0.1)
    else:
        print("FAIL: arm_sim 未就绪")
        proc.kill()
        return 1

    phases = [("A 无客户端", 0), ("B 1 客户端", 1), ("C 4 客户端", 4)]
    rows = []
    t_start = time.time()
    print(f"# pigz=off pid={proc.pid} port={port} 每段 {args.minutes} min，采样 {args.sample} s")
    print("phase,elapsed_s,rss_kb,ws_clients,http_clients,teach_points,safety_total,"
          "static_cache_bytes,tx_dropped,header_rejects,conn_rejects,hammer_ops")
    phase_idx = 0
    hammer = []
    held_ws = []
    seg_end = time.time() + args.minutes * 60
    last_sample = 0.0
    while phase_idx < len(phases):
        name, want_ws = phases[phase_idx]
        now = time.time()
        if now >= seg_end:
            # 阶段切换：清理上一阶段的客户端
            for h in hammer:
                h.stop = True
            for s in held_ws:
                try:
                    s.close()
                except Exception:
                    pass
            hammer, held_ws = [], []
            phase_idx += 1
            if phase_idx >= len(phases):
                break
            seg_end = time.time() + args.minutes * 60
            last_sample = 0.0
            continue

        # 维持本阶段所需的 WS 客户端
        while len(held_ws) < want_ws:
            try:
                s, _ = ws_handshake(port)
                held_ws.append(s)
            except Exception:
                time.sleep(0.5)
        drain_sockets(held_ws)

        if now - last_sample >= args.sample:
            last_sample = now
            # 观测 churn：持有的客户端若被服务端丢弃（不读取/超时），sample 会显示 ws_clients 下降

            h = health(port)
            rows.append((name, round(now - t_start, 1), rss_kb(proc.pid),
                         h.get("ws_clients", 0), h.get("http_clients", 0), h.get("teach_points", 0),
                         h.get("safety_total", 0), h.get("static_cache_bytes", 0),
                         h.get("serial_tx_dropped_full", 0), h.get("header_rejects", 0),
                         h.get("rejected_clients", 0),
                         sum(x.ops for x in hammer) if hammer else 0))
            r = rows[-1]
            print(f"{r[0]},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]},{r[7]},{r[8]},{r[9]},{r[10]},{r[11]}",
                  flush=True)

        # C 阶段压测：1 个 HTTP 线程 + 3 个 WS 洪水线程（与 4 个客户端并存）
        if phase_idx == 2 and not hammer:
            hammer = [Hammer(port, "http")] + [Hammer(port, "ws") for _ in range(3)]
            for h in hammer:
                h.start()
        time.sleep(0.5)

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()

    with open(args.csv, "w") as f:
        f.write("phase,elapsed_s,rss_kb,ws_clients,http_clients,teach_points,safety_total,"
                "static_cache_bytes,tx_dropped,header_rejects,conn_rejects,hammer_ops\n")
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")

    # ---- 汇总：按阶段给出 RSS 首末/斜率 ----
    print("\n===== G1 汇总 =====")
    ok = True
    for name, _ in phases:
        seg = [r for r in rows if r[0] == name]
        if len(seg) < 2:
            print(f"  {name}: 样本不足（{len(seg)}）")
            continue
        first, last = seg[0], seg[-1]
        dt_min = (last[1] - first[1]) / 60.0
        grow_kb = last[2] - first[2]
        slope = grow_kb / dt_min if dt_min > 0 else 0
        print(f"  {name}: 样本 {len(seg)}，时长 {dt_min:.1f} min，RSS {first[2]} → {last[2]} KB "
              f"(Δ{grow_kb:+d} KB, {slope:+.1f} KB/min)，teach={first[5]}→{last[5]}，"
              f"hammer_ops={last[11]}")
        # 阈值：任何阶段 RSS 斜率 > 1 MB/min 视为异常增长（30 min 会涨 30 MB）
        if slope > 1024:
            print(f"    ⚠️ 增长过快：{slope:.0f} KB/min")
            ok = False
    print(f"CSV: {args.csv}")
    print("SOAK " + ("OK（无异常增长）" if ok else "WARN（存在异常增长，见上）"))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
