// tests/serial_loopback.cpp — H：PTY 软件从机模拟器（真机协议闭环，无硬件）
//
// 结构：driver 侧 = SerialDriver(FdByteIo(pty master))，role=Master（真机语义：发 CMD_* / 收 STATE_REP）
//       从机侧 = 本文件内的 SlaveSim，跑在 pty slave 上，解析 CMD_* 并回 STATE_REP
//
// 验证：
//   ① 完整往返：CMD_POS → 从机执行 → STATE_REP → remoteState() 数值吻合（附字节级抓包）
//   ② 连续 N 帧无丢失/乱序：frames 计数与末值正确
//   ③ CRC 错误帧被拒（不污染 remoteState）
//   ④ 从机侧半截帧（分片到达）也能被正确重组
#include "arm/control/serial_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <thread>
#include <vector>

#include <chrono>
#include <termios.h>
#include <unistd.h>

using namespace arm;

static int g_fail = 0;
#define CHECK(cond, ...)                                                     \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("FAIL %s:%d  ", __FILE__, __LINE__);                       \
      std::printf(__VA_ARGS__);                                              \
      std::printf("\n");                                                     \
      g_fail++;                                                              \
    }                                                                        \
  } while (0)

static std::string hex(const std::vector<uint8_t>& b, size_t maxBytes = 64) {
  std::string s;
  char buf[8];
  for (size_t i = 0; i < b.size() && i < maxBytes; i++) {
    std::snprintf(buf, sizeof buf, "%02X ", b[i]);
    s += buf;
  }
  if (b.size() > maxBytes) s += "…";
  return s;
}

// ---------- 软件从机：解析 CMD_*，回 STATE_REP ----------
struct SlaveSim {
  int fd = -1;
  std::array<double, 6> q{}, qd{};
  double grip = 0.0;
  bool estop = false;
  std::vector<uint8_t> rxLog;       // 收到的原始字节（抓包）
  int framesIn = 0, framesOut = 0;
  bool corruptNextReply = false;    // 注入：下一帧 STATE_REP 破坏 CRC
  int fragmentReplyInto = 1;        // 注入：把回复切成 N 片发送（测试半截帧重组）
  bool stop = false;

  explicit SlaveSim(int slaveFd) : fd(slaveFd) {}

  void runOnce() {
    uint8_t tmp[256];
    ssize_t n = ::read(fd, tmp, sizeof tmp);
    if (n <= 0) return;
    rxLog.insert(rxLog.end(), tmp, tmp + n);
    rxBuf_.insert(rxBuf_.end(), tmp, tmp + n);   // 送入解码缓冲（rxLog 仅抓包用）
    serialproto::FrameOut fr;
    bool bad = false;
    while (serialproto::tryDecode(rxBuf_, fr, bad) > 0) {
      framesIn++;
      using namespace serialproto;
      switch (fr.type) {
        case kCmdPos:
          for (size_t i = 0; i + 1 < fr.payload.size() && i / 2 < 6; i += 2)
            q[i / 2] = double(getI16(fr.payload.data() + i)) / 1000.0;
          qd.fill(0.0);
          reply();
          break;
        case kCmdVel:
          for (size_t i = 0; i + 1 < fr.payload.size() && i / 2 < 6; i += 2)
            qd[i / 2] = double(getI16(fr.payload.data() + i)) / 1000.0;
          reply();
          break;
        case kCmdGrip:
          grip = fr.payload.empty() ? 0.0 : fr.payload[0] / 255.0;
          reply();
          break;
        case kCmdEstop:
          estop = !fr.payload.empty() && fr.payload[0] != 0;
          if (estop) qd.fill(0.0);
          reply();
          break;
        case kStateReq:
          reply();
          break;
        default:
          break;   // 未知类型：忽略（协议前向兼容）
      }
      fr = FrameOut{};
    }
  }

  void reply() {
    using namespace serialproto;
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, int16_t(q[i] * 1000.0));
    for (int i = 0; i < 6; i++) putI16(p, int16_t(qd[i] * 1000.0));
    p.push_back(uint8_t(std::min(1.0, std::max(0.0, grip)) * 255.0));
    auto f = encode(kStateRep, p);
    if (corruptNextReply) {           // 破坏 CRC（末两字节）
      f[f.size() - 1] ^= 0xFF;
      corruptNextReply = false;
    }
    framesOut++;
    int parts = std::max(1, fragmentReplyInto);
    size_t per = std::max<size_t>(1, f.size() / size_t(parts));
    for (size_t off = 0; off < f.size(); off += per) {
      size_t len = std::min(per, f.size() - off);
      ::write(fd, f.data() + off, len);
      if (parts > 1) std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 制造分片
    }
  }

  std::vector<uint8_t> rxBuf_;
};

int main() {
  using namespace serialproto;

  // ---------- 建 PTY ----------
  int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  CHECK(master >= 0, "posix_openpt 失败");
  if (master < 0) return 1;
  CHECK(::grantpt(master) == 0 && ::unlockpt(master) == 0, "grantpt/unlockpt 失败");
  const char* sname = ::ptsname(master);
  CHECK(sname != nullptr, "ptsname 失败");
  if (!sname) return 1;
  int slave = ::open(sname, O_RDWR | O_NOCTTY | O_NONBLOCK);
  CHECK(slave >= 0, "打开从机端 %s 失败", sname);
  if (slave < 0) return 1;

  // 从机端必须 raw（否则行规程按 \n 缓冲二进制帧、并回显）
  {
    termios tty{};
    tcgetattr(slave, &tty);
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(slave, TCSANOW, &tty);
  }

  SlaveSim sim(slave);
  std::thread simThread([&] {
    while (!sim.stop) {
      sim.runOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  // ---------- driver 侧 ----------
  SerialDriver drv(sname, 115200, SerialDriver::Role::Master);
  drv.setByteIo(std::unique_ptr<ByteIo>(new FdByteIo(master)));
  CHECK(drv.online(), "driver 上线失败");

  auto waitFor = [&](int wantFrames, int timeoutMs) {
    for (int i = 0; i < timeoutMs; i++) {
      drv.pollCommands();                       // 顺带读入
      if (int(drv.remoteState().frames) >= wantFrames) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  };

  // ================= ① 完整往返（CMD_POS → STATE_REP）=================
  {
    std::array<double, 6> q{0.10, -0.20, 0.30, -0.40, 0.50, -0.60};
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, int16_t(q[size_t(i)] * 1000.0));
    auto expectFrame = encode(kCmdPos, p);

    drv.sendCmdPos(q);
    drv.pumpTx();
    bool ok = waitFor(1, 500);
    CHECK(ok && drv.remoteState().valid, "未收到从机 STATE_REP");
    bool match = true;
    for (int i = 0; i < 6; i++)
      if (std::abs(drv.remoteState().q[size_t(i)] - q[size_t(i)]) > 1e-3) match = false;
    CHECK(match, "回读关节角与指令不一致");
    CHECK(sim.framesIn >= 1 && sim.framesOut >= 1, "从机未解析/未回复");
    std::printf("[H-①] 往返：driver 发出 %zu B | 从机收到并解析 %d 帧 | 回发 %d 帧 | driver 回读 q=[",
                expectFrame.size(), sim.framesIn, sim.framesOut);
    for (int i = 0; i < 6; i++) std::printf("%.2f%s", drv.remoteState().q[size_t(i)], i < 5 ? " " : "");
    std::printf("]\n");
    std::printf("        抓包（driver→从机）: %s\n", hex(sim.rxLog).c_str());
  }

  // ================= ② 连续 20 帧无丢失/乱序 =================
  {
    const int N = 20;
    for (int k = 0; k < N; k++) {
      std::array<double, 6> q{};
      for (int i = 0; i < 6; i++) q[size_t(i)] = 0.001 * double(k * 6 + i);
      drv.sendCmdPos(q);
      drv.pumpTx();
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    bool ok = waitFor(int(drv.remoteState().frames) + N, 1000);
    CHECK(ok, "连续帧回读不足（frames=%llu）", (unsigned long long)drv.remoteState().frames);
    CHECK(sim.framesIn == sim.framesOut, "从机收发不等（in=%d out=%d）", sim.framesIn, sim.framesOut);
    double want = 0.001 * double((N - 1) * 6 + 5);
    CHECK(std::abs(drv.remoteState().q[5] - want) < 1e-3, "末帧数值错乱：q5=%.4f 应 %.4f",
          drv.remoteState().q[5], want);
    std::printf("[H-②] 连发 %d 帧：从机 in=%d out=%d，driver frames=%llu，末值 q5=%.3f（应 %.3f）"
                " —— 无丢失/乱序\n",
                N, sim.framesIn, sim.framesOut, (unsigned long long)drv.remoteState().frames,
                drv.remoteState().q[5], want);
  }

  // ================= ③ CRC 错误帧被拒 =================
  {
    auto before = drv.remoteState();
    sim.corruptNextReply = true;
    drv.sendCmdPos({0.7, 0.7, 0.7, 0.7, 0.7, 0.7});
    drv.pumpTx();
    for (int i = 0; i < 200; i++) { drv.pollCommands(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    // 坏帧被跳过后，driver 会继续等待：remoteState 不更新（数值仍是上一帧）
    CHECK(drv.remoteState().frames == before.frames,
          "坏 CRC 帧不应更新状态（frames %llu → %llu）", (unsigned long long)before.frames,
          (unsigned long long)drv.remoteState().frames);
    CHECK(std::abs(drv.remoteState().q[0] - before.q[0]) < 1e-9, "坏帧污染了数值");
    std::printf("[H-③] 注入坏 CRC 的 STATE_REP：frames 保持 %llu（旧值 q0=%.3f 未被覆盖）\n",
                (unsigned long long)drv.remoteState().frames, drv.remoteState().q[0]);

    // 随后的好帧必须仍然可用（坏帧不能卡死链路）
    sim.fragmentReplyInto = 1;
    drv.sendCmdPos({0.9, 0, 0, 0, 0, 0});
    drv.pumpTx();
    bool ok = waitFor(int(before.frames) + 1, 500);
    CHECK(ok && std::abs(drv.remoteState().q[0] - 0.9) < 1e-3, "坏帧后链路未能恢复");
    std::printf("[H-③补] 坏帧后链路自愈：新帧 q0=%.3f（frames=%llu）\n", drv.remoteState().q[0],
                (unsigned long long)drv.remoteState().frames);
  }

  // ================= ④ 分片到达（半截帧重组）=================
  {
    sim.fragmentReplyInto = 4;               // 每帧切成 4 片发
    auto f0 = drv.remoteState().frames;
    drv.sendCmdPos({0.123, -0.456, 0.789, 0, 0, 0});
    drv.pumpTx();
    bool ok = waitFor(int(f0) + 1, 800);
    CHECK(ok, "分片回复未被重组（frames=%llu）", (unsigned long long)drv.remoteState().frames);
    CHECK(std::abs(drv.remoteState().q[0] - 0.123) < 1e-3 && std::abs(drv.remoteState().q[2] - 0.789) < 1e-3,
          "分片重组数值错误：q0=%.3f q2=%.3f", drv.remoteState().q[0], drv.remoteState().q[2]);
    std::printf("[H-④] 回复切成 4 片（每片间隔 2 ms）：跨拍重组成功 q0=%.3f q2=%.3f\n",
                drv.remoteState().q[0], drv.remoteState().q[2]);
  }

  // ================= ⑤ 急停通道 =================
  {
    drv.emergencyStop();
    for (int i = 0; i < 300 && !sim.estop; i++) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    CHECK(sim.estop, "从机未收到 ESTOP");
    CHECK(drv.txStats().queuedFrames == 0 || drv.txStats().queuedBytes < SerialDriver::kDefaultTxLimit,
          "急停后队列未清空");
    std::printf("[H-⑤] ESTOP：从机置位 estop=%d，driver 队列剩余 %zu 帧 / %zu B\n", int(sim.estop),
                drv.txStats().queuedFrames, drv.txStats().queuedBytes);
  }

  sim.stop = true;
  simThread.join();
  ::close(slave);

  auto st = drv.txStats();
  std::printf("[H-summary] 累计写 %llu B，重试 %llu，硬错误 %llu，丢弃(满/离线) %llu/%llu，"
              "rxOverflow %llu\n",
              (unsigned long long)st.written, (unsigned long long)st.retries,
              (unsigned long long)st.ioErrors, (unsigned long long)st.droppedFull,
              (unsigned long long)st.droppedOffline, (unsigned long long)st.rxOverflow);

  if (g_fail == 0) std::printf("test_serial_loopback PASS\n");
  else std::printf("test_serial_loopback FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
