// tests/test_serial.cpp — 串口协议帧编解码单元测试（无硬件）
#include "arm/control/serial_driver.hpp"
#include <cstdio>
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

int main() {
  using namespace serialproto;

  // ---- 1. CRC16-CCITT 已知向量 ----
  {
    const char* s = "123456789";
    uint16_t crc = crc16(reinterpret_cast<const uint8_t*>(s), 9);
    CHECK(crc == 0x29B1, "crc16 = 0x%04X 应为 0x29B1", crc);
  }

  // ---- 2. 编解码往返 ----
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
    CHECK(getI16(out.payload.data()) == 1234 && getI16(out.payload.data() + 2) == -5678, "payload 数值");
    CHECK(buf.empty(), "缓冲未清空");
  }

  // ---- 3. 坏 CRC 跳过 + 噪声后重新同步 ----
  {
    auto good = encode(kCmdGrip, {200});
    auto bad = good;
    bad[5] ^= 0xFF;   // 破坏 CRC
    std::vector<uint8_t> buf = bad;
    buf.insert(buf.end(), good.begin(), good.end());
    FrameOut out;
    bool badcrc = false;
    int frames = 0;
    bool sawBad = false;
    while (tryDecode(buf, out, badcrc) > 0) { frames++; if (badcrc) sawBad = true; }
    CHECK(frames == 1, "应只收到 1 个完好帧，实际 %d", frames);
    CHECK(sawBad, "应报告坏 CRC");
  }

  // ---- 4. SerialDriver JSON 映射（注入字节 → pollCommands）----
  {
    SerialDriver drv("/dev/null-does-not-exist");
    // 不 begin() 也可测协议路径（纯缓冲）
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
  }

  if (g_fail == 0) std::printf("test_serial PASS\n");
  else std::printf("test_serial FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
