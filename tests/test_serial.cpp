// tests/test_serial.cpp — 串口驱动测试（协议 + 真机 IO 错误路径）
//
// 覆盖三层：
//   1) 协议编解码 / CRC / 坏帧重同步（原有）
//   2) 角色语义：Pendant（推状态/收指令）与 Master（发指令/收状态）—— 消除语义倒置
//   3) **真机写路径硬化（E1/E2/E3）**：部分写续传、EAGAIN 下拍重试、硬错误离线、
//      队列上限与丢弃计数、ESTOP 抢占、读缓冲上限、E2 无限增长回归
#include "arm/control/serial_driver.hpp"

#include <chrono>
#include <cstdio>
#include <vector>

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

// ---------- 可编程假串口：模拟部分写 / EAGAIN / 硬错误 ----------
class FakeIo : public ByteIo {
 public:
  enum class Mode { Ok, Partial, AlwaysEagain, HardError };
  Mode mode = Mode::Ok;
  size_t partialChunk = 0;            // Partial：每次最多接受的字节数（0 = 一半）
  std::vector<uint8_t> written;       // 实际写出（EAGAIN/硬错误不计）
  int writeCalls = 0;                 // writeSome 调用次数（部分写场景 >1）
  bool closed = false;

  ssize_t writeSome(const uint8_t* d, size_t n) override {
    writeCalls++;
    switch (mode) {
      case Mode::AlwaysEagain: errno = EAGAIN; return -1;
      case Mode::HardError:    errno = EIO;    return -1;
      case Mode::Partial: {
        size_t take = partialChunk ? std::min(partialChunk, n) : std::max<size_t>(1, n / 2);
        written.insert(written.end(), d, d + take);
        return ssize_t(take);
      }
      default:
        written.insert(written.end(), d, d + n);
        return ssize_t(n);
    }
  }
  ssize_t readSome(uint8_t*, size_t) override { errno = EAGAIN; return -1; }
  bool valid() const override { return !closed; }
  void close() override { closed = true; }
};

// 便捷：造一个已连接假串口的驱动，返回其假 IO 裸指针
static std::unique_ptr<SerialDriver> makeDriver(FakeIo** ioOut,
                                                SerialDriver::Role role = SerialDriver::Role::Master) {
  auto* io = new FakeIo();
  auto d = std::make_unique<SerialDriver>("/dev/fake", 115200, role);
  d->setByteIo(std::unique_ptr<ByteIo>(io));
  *ioOut = io;
  return d;
}

static std::array<double, 6> qOf(double v) {
  std::array<double, 6> q{};
  q.fill(v);
  return q;
}

int main() {
  using namespace serialproto;

  // ================= 1. 协议层 =================
  {
    const char* s = "123456789";
    CHECK(crc16(reinterpret_cast<const uint8_t*>(s), 9) == 0x29B1, "crc16 已知向量");
  }
  {
    std::vector<uint8_t> payload;
    putI16(payload, 1234);
    putI16(payload, -5678);
    auto f = encode(kCmdPos, payload);
    CHECK(f.size() == 2 + 2 + 4 + 2, "帧长 %zu", f.size());
    std::vector<uint8_t> buf = f;
    FrameOut out;
    bool bad = false;
    CHECK(tryDecode(buf, out, bad) == f.size() && !bad, "解码失败");
    CHECK(out.type == kCmdPos && out.payload.size() == 4, "帧内容");
    CHECK(getI16(out.payload.data()) == 1234 && getI16(out.payload.data() + 2) == -5678, "payload");
    CHECK(buf.empty(), "缓冲未清空");
  }
  {
    auto good = encode(kCmdGrip, {200});
    auto badFrame = good;
    badFrame[5] ^= 0xFF;   // 破坏 CRC
    std::vector<uint8_t> buf = badFrame;
    buf.insert(buf.end(), good.begin(), good.end());
    FrameOut out;
    bool badcrc = false;
    int frames = 0;
    bool sawBad = false;
    while (tryDecode(buf, out, badcrc) > 0) { frames++; if (badcrc) sawBad = true; }
    CHECK(frames == 1 && sawBad, "坏 CRC 跳过后应重新同步（frames=%d bad=%d）", frames, int(sawBad));
  }

  // ================= 2. Pendant 角色（兼容旧行为）=================
  {
    SerialDriver drv("/dev/null-does-not-exist", 115200, SerialDriver::Role::Pendant);
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, int16_t((0.5 + 0.1 * i) * 1000.0));
    drv.injectRx(encode(kCmdPos, p));
    drv.injectRx(encode(kCmdEstop, {1}));
    auto cmds = drv.pollCommands();
    CHECK(cmds.size() == 2, "指令数 %zu", cmds.size());
    if (cmds.size() == 2) {
      CHECK(cmds[0].get("type").asString() == "joint_target", "type1");
      CHECK(std::abs(cmds[0].get("q").numAt(0) - 0.5) < 1e-6, "q[0]");
      CHECK(cmds[1].get("type").asString() == "estop" && cmds[1].get("on").asBool(), "estop");
    }
    std::printf("[role] Pendant：injectRx(CMD_POS/CMD_ESTOP) → pollCommands 映射 %zu 条指令\n",
                cmds.size());
  }

  // ================= 3. Master 角色：下行帧与上行解析 =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io, SerialDriver::Role::Master);
    drv->sendCmdPos(qOf(0.25));
    size_t n = drv->pumpTx();
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, 250);
    auto expect = encode(kCmdPos, p);
    CHECK(n == expect.size() && io->written == expect, "Master 下行 CMD_POS 字节级正确（%zu B）",
          io->written.size());

    // 上行：STATE_REP → remoteState()
    std::vector<uint8_t> sp;
    for (int i = 0; i < 6; i++) putI16(sp, int16_t((0.1 * (i + 1)) * 1000.0));
    for (int i = 0; i < 6; i++) putI16(sp, int16_t((-0.05 * (i + 1)) * 1000.0));
    sp.push_back(uint8_t(0.5 * 255));
    drv->injectRx(encode(kStateRep, sp));
    auto cmds = drv->pollCommands();
    const auto& rs = drv->remoteState();
    CHECK(cmds.empty(), "Master 角色不把 STATE_REP 当指令（%zu）", cmds.size());
    CHECK(rs.valid && rs.frames == 1, "STATE_REP 被解析（frames=%llu）",
          (unsigned long long)rs.frames);
    CHECK(std::abs(rs.q[5] - 0.6) < 1e-9 && std::abs(rs.qd[0] + 0.05) < 1e-9 &&
              std::abs(rs.grip - 0.5) < 0.01,
          "STATE_REP 数值 q5=%.3f qd0=%.3f grip=%.3f", rs.q[5], rs.qd[0], rs.grip);
    std::printf("[role] Master：CMD_POS 下行 %zu B 字节级吻合；STATE_REP 上行 q5=%.3f qd0=%.3f grip=%.2f\n",
                io->written.size(), rs.q[5], rs.qd[0], rs.grip);

    // 坏 CRC 的 STATE_REP 必须被拒（remoteState 不更新）
    auto badFrame = encode(kStateRep, sp);
    badFrame[badFrame.size() - 1] ^= 0xFF;
    drv->injectRx(badFrame);
    (void)drv->pollCommands();
    CHECK(drv->remoteState().frames == 1, "坏 CRC 状态帧被拒（frames 仍为 %llu）",
          (unsigned long long)drv->remoteState().frames);
  }

  // ================= 4. E3-1 部分写续传 =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io);
    io->mode = FakeIo::Mode::Partial;
    io->partialChunk = 3;                      // 每次只接受 3 字节（帧 17 B）
    drv->sendCmdPos(qOf(-0.125));
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, -125);
    auto expect = encode(kCmdPos, p);
    int pumps = 0;
    while (drv->txStats().queuedFrames > 0 && pumps < 100) { drv->pumpTx(); pumps++; }
    CHECK(io->written == expect, "部分写续传：最终字节流完整（%zu/%zu B）", io->written.size(),
          expect.size());
    CHECK(io->writeCalls >= 6, "单拍内经历 %d 次部分写续传（每帧 18 B / 每次接受 3 B）",
          io->writeCalls);
    std::printf("[E3-1] 部分写：每次仅接受 3 B → 单拍内 %d 次 writeSome 续传，%zu B 完整送达"
                "（%d 次 pumpTx），无丢帧无重复\n",
                io->writeCalls, io->written.size(), pumps);
  }

  // ================= 5. E3-2 EAGAIN 下拍重试 =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io);
    io->mode = FakeIo::Mode::AlwaysEagain;
    drv->sendCmdPos(qOf(0.5));
    drv->pumpTx();
    auto st1 = drv->txStats();
    CHECK(st1.retries == 1 && st1.queuedFrames == 1 && io->written.empty(),
          "EAGAIN：本轮 0 写入、帧保留、retries=%llu", (unsigned long long)st1.retries);
    io->mode = FakeIo::Mode::Ok;               // 下一拍端口可写
    size_t n = drv->pumpTx();
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, 500);
    CHECK(n > 0 && io->written == encode(kCmdPos, p) && drv->txStats().queuedFrames == 0,
          "EAGAIN 后下拍补齐：%zu B，队列清空", io->written.size());
    std::printf("[E3-2] EAGAIN：首拍 0 B（retries=1，帧未丢）→ 次拍补齐 %zu B\n", io->written.size());
  }

  // ================= 6. E3-3 硬错误 → 离线可观察 =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io);
    io->mode = FakeIo::Mode::HardError;
    drv->sendCmdPos(qOf(0.1));
    drv->pumpTx();
    CHECK(!drv->online(), "硬错误后 online()=false（可被上层观察）");
    CHECK(drv->txStats().ioErrors == 1, "ioErrors=%llu", (unsigned long long)drv->txStats().ioErrors);
    drv->sendCmdPos(qOf(0.2));                // 离线后继续发起
    auto st3 = drv->txStats();
    CHECK(st3.droppedOffline == 2 && st3.queuedFrames == 0,
          "离线后：在途帧被清空+新帧不再入队（droppedOffline=%llu, queued=%zu）",
          (unsigned long long)st3.droppedOffline, st3.queuedFrames);
    std::printf("[E3-3] 硬错误(EIO)：offline=true、ioErrors=1；在途+后续共 %llu 帧计入"
                " droppedOffline（队列清空，内存不再增长）\n",
                (unsigned long long)st3.droppedOffline);
  }

  // ================= 7. E3-4 队列上限：丢弃计数 + 不阻塞调用方 =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io);
    io->mode = FakeIo::Mode::AlwaysEagain;     // 端口一直不可写 → 队列必然填满
    drv->setTxQueueLimit(200);
    const int N = 20000;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < N; i++) drv->sendCmdPos(qOf(0.001 * i));
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    auto st = drv->txStats();
    CHECK(st.queuedBytes <= 200, "队列严格受限：%zu B ≤ 200 B", st.queuedBytes);
    CHECK(st.droppedFull > 0, "丢弃被计数：droppedFull=%llu", (unsigned long long)st.droppedFull);
    // 不变式：每次尝试都成功入队（enqueued == N）；被挤出队列的（droppedFull）+ 仍在队列里的
    // 恰好等于入队总数（没有任何帧被静默吞掉）
    CHECK(st.enqueued == uint64_t(N) && st.droppedFull + st.queuedFrames == st.enqueued,
          "队列守恒：enqueued=%llu = droppedFull(%llu) + queuedFrames(%zu)",
          (unsigned long long)st.enqueued, (unsigned long long)st.droppedFull, st.queuedFrames);
    CHECK(ms < 200.0, "调用方不被阻塞：%d 次入队耗时 %.1f ms", N, ms);
    std::printf("[E3-4] 队列上限 200 B：%d 次高速发送耗时 %.1f ms（不阻塞），"
                "积压 %zu B/%zu 帧，droppedFull=%llu\n",
                N, ms, st.queuedBytes, st.queuedFrames, (unsigned long long)st.droppedFull);
  }

  // ================= 8. E2 回归：50 Hz 广播 1 小时等效，内存不增长 =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io, SerialDriver::Role::Pendant);
    io->mode = FakeIo::Mode::AlwaysEagain;     // 最坏情况：对端一直不收
    StateSnapshot st;
    st.q = qOf(0.3);
    st.qd = qOf(0.01);
    st.grip = 0.5;
    const int hourFrames = 50 * 3600;          // 1 小时 @50 Hz
    size_t maxQueued = 0;
    for (int i = 0; i < hourFrames; i++) {
      drv->broadcastState(st);
      maxQueued = std::max(maxQueued, drv->txStats().queuedBytes);
      if ((i % 1000) == 0) drv->pumpTx();      // 周期性尝试（仍 EAGAIN）
    }
    auto t2 = drv->txStats();
    CHECK(maxQueued <= SerialDriver::kDefaultTxLimit,
          "E2 回归：1 小时 50 Hz 广播后队列峰值 %zu B ≤ %zu B（旧实现 +5.6 MB/h 无上限）",
          maxQueued, SerialDriver::kDefaultTxLimit);
    CHECK(t2.droppedFull + t2.coalesced > 0, "丢弃/合并被计数：droppedFull=%llu coalesced=%llu",
          (unsigned long long)t2.droppedFull, (unsigned long long)t2.coalesced);
    std::printf("[E2] 50 Hz × 1 h（%d 帧，对端始终不收）：队列峰值 %zu B（上限 %zu B），"
                "coalesced=%llu droppedFull=%llu —— 内存有界\n",
                hourFrames, maxQueued, SerialDriver::kDefaultTxLimit,
                (unsigned long long)t2.coalesced, (unsigned long long)t2.droppedFull);
  }

  // ================= 9. ESTOP 抢占（队列满也不丢） =================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io);
    io->mode = FakeIo::Mode::AlwaysEagain;
    for (int i = 0; i < 500; i++) drv->sendCmdPos(qOf(0.01 * i));   // 塞满
    io->mode = FakeIo::Mode::Ok;
    drv->emergencyStop();                      // 清空 + 置顶 ESTOP + 立即尝试发出
    auto st = drv->txStats();
    CHECK(st.superseded > 0, "在途数据被急停取代（superseded=%llu）", (unsigned long long)st.superseded);
    CHECK(io->written == encode(kCmdEstop, {1}), "写出的第一帧就是 ESTOP（%zu B）",
          io->written.size());
    std::printf("[ESTOP] 队列 %llu 帧在途 → 急停清空并立即发出 ESTOP 帧（%zu B），"
                "superseded=%llu\n",
                (unsigned long long)st.superseded, io->written.size(),
                (unsigned long long)st.superseded);
  }

  // ================= 10a. 半发送帧保护：不被合并覆盖、不被丢弃（防线路半截帧）=================
  {
    FakeIo* io = nullptr;
    auto drv = makeDriver(&io, SerialDriver::Role::Pendant);
    io->mode = FakeIo::Mode::Partial;
    io->partialChunk = 5;                      // 每次只写 5 B
    StateSnapshot st1;
    st1.q = qOf(0.2);                          // 第一帧内容 q=0.2
    st1.grip = 0.3;
    drv->setTxQueueLimit(400);
    drv->broadcastState(st1);
    drv->pumpTx(5);   // 只允许写 5 B → 队首帧停在「半发送」（5/31 B）
    auto a = drv->txStats();
    CHECK(a.queuedFrames == 1, "队首应处于半发送状态（frames=%zu）", a.queuedFrames);

    StateSnapshot st2;                         // 后续广播使用**不同**内容（q=0.9）
    st2.q = qOf(0.9);
    st2.grip = 0.8;
    for (int i = 0; i < 40; i++) drv->broadcastState(st2);

    size_t guard = 0;
    while (drv->txStats().queuedFrames > 0 && guard++ < 10000) drv->pumpTx();

    // 期望：第一帧仍是 q=0.2 的内容（半发送帧未被新内容覆盖）；且整串可解码
    std::vector<uint8_t> p0;
    for (int i = 0; i < 6; i++) putI16(p0, int16_t(0.2 * 1000.0));
    for (int i = 0; i < 6; i++) putI16(p0, 0);
    p0.push_back(uint8_t(0.3 * 255.0));
    auto expectFirst = encode(kStateRep, p0);

    std::vector<uint8_t> stream = io->written;
    FrameOut fr;
    bool bad = false;
    size_t n1 = tryDecode(stream, fr, bad);
    CHECK(n1 == expectFirst.size() && !bad && fr.type == kStateRep,
          "半发送帧续传后仍为合法帧（解码 %zu B / 期望 %zu B, bad=%d）", n1, expectFirst.size(),
          int(bad));
    bool samePayload = (fr.payload.size() == p0.size()) &&
                       std::equal(fr.payload.begin(), fr.payload.end(), p0.begin());
    CHECK(samePayload, "半发送帧内容不得被新状态覆盖（首帧 q 应为 0.2 而非 0.9）");
    std::printf("[半发送保护] 队首 5/31 B 时追加 40 帧（内容 q=0.9）：coalesced=%llu，"
                "排空后首帧仍为 q=0.2 的合法帧（%zu B，payload 逐字节相同）——半帧不被覆盖/丢弃\n",
                (unsigned long long)drv->txStats().coalesced, n1);
  }

  // ================= 10. 读缓冲上限 =================
  {
    SerialDriver drv("/dev/none", 115200, SerialDriver::Role::Pendant);
    drv.setRxBufferLimit(64);
    std::vector<uint8_t> junk(200, 0x5A);
    drv.injectRx(junk);
    CHECK(drv.txStats().rxOverflow == 1, "读缓冲超限被计数（rxOverflow=%llu）",
          (unsigned long long)drv.txStats().rxOverflow);
    std::printf("[RX] 垃圾流 200 B > 上限 64 B → 清空并计数 rxOverflow=1（不留半截帧）\n");
  }

  if (g_fail == 0) std::printf("test_serial PASS\n");
  else std::printf("test_serial FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
