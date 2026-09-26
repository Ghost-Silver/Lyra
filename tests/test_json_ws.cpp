// tests/test_json_ws.cpp — JSON / SHA1 / Base64 / WS 帧编解码单元测试
#include "arm/control/json.hpp"
#include "arm/control/ws_server.hpp"
#include <cstdio>
using namespace arm;
using namespace arm::json;

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
  // ---- 1. JSON 解析 / 序列化 ----
  {
    Value v;
    CHECK(Value::parse(R"({"a":1.5,"b":[1,2,3],"c":{"d":true,"e":null},"s":"x\ny\u4e2d"})", v),
          "解析失败");
    CHECK(v.get("a").asNumber() == 1.5, "num");
    CHECK(v.get("b").size() == 3 && v.get("b").numAt(2) == 3, "array");
    CHECK(v.get("c").get("d").asBool(), "bool");
    CHECK(v.get("c").get("e").isNull(), "null");
    CHECK(v.get("s").asString() == "x\ny\xe4\xb8\xad", "escape: %s", v.get("s").asString().c_str());
    // dump → reparse 往返
    std::string s = v.dump();
    Value v2;
    CHECK(Value::parse(s, v2), "reparse 失败: %s", s.c_str());
    CHECK(v2.get("a").asNumber() == 1.5 && v2.get("b").size() == 3, "roundtrip");
    // 数字格式
    Value n;
    CHECK(Value::parse("[-2.5e3, 1e-3, 42]", n) && n.numAt(0) == -2500 && n.numAt(2) == 42, "nums");
    Value bad;
    CHECK(!Value::parse("{oops}", bad), "应拒绝坏 JSON");
    // 严格数字语法（P2 回归）
    CHECK(!Value::parse("1..2....3", bad), "应拒绝 1..2....3");
    CHECK(!Value::parse("--5", bad), "应拒绝 --5");
    CHECK(!Value::parse("1e5e5", bad), "应拒绝 1e5e5");
    CHECK(!Value::parse("+5", bad), "应拒绝 +5");
    CHECK(!Value::parse("01", bad), "应拒绝 01");
    CHECK(Value::parse("1e5", bad) && bad.asNumber() == 1e5, "1e5");
    CHECK(Value::parse("-0.5", bad) && bad.asNumber() == -0.5, "-0.5");
    CHECK(Value::parse("0", bad) && bad.asNumber() == 0, "0");
    // 深度限制（P0-3 回归：修复前 20 万层递归 SIGSEGV）
    std::string deep(201, '[');
    deep += "1";
    deep.append(201, ']');
    CHECK(!Value::parse(deep, bad), "201 层嵌套应拒绝");
    std::string deep2(100, '[');
    deep2 += "1";
    deep2.append(100, ']');
    CHECK(Value::parse(deep2, bad), "100 层嵌套应通过");
    // 容器安全 API（P2 回归）
    Value arr;
    CHECK(Value::parse("[7]", arr) && arr[0].asNumber() == 7, "arr[0]");
    CHECK(arr[5].isNull(), "operator[] 越界应 Null");
    CHECK(arr.get("zz").isNull(), "get() 缺键应 Null");
  }

  // ---- 2. SHA1 / Base64 已知向量 ----
  {
    uint8_t d[20];
    std::string abc = "abc";
    wsutil::sha1(reinterpret_cast<const uint8_t*>(abc.data()), abc.size(), d);
    char hex[41];
    for (int i = 0; i < 20; i++) std::sprintf(hex + 2 * i, "%02x", d[i]);
    CHECK(std::string(hex) == "a9993e364706816aba3e25717850c26c9cd0d89d", "sha1(abc) = %s", hex);
    CHECK(wsutil::base64(reinterpret_cast<const uint8_t*>("hello"), 5) == "aGVsbG8=", "b64");
    // RFC6455 §1.3 握手示例
    CHECK(wsutil::acceptKey("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=",
          "acceptKey 错: %s", wsutil::acceptKey("dGhlIHNhbXBsZSBub25jZQ==").c_str());
  }

  // ---- 3. WS 帧编解码 ----
  {
    std::string f = wsutil::encodeFrame(wsutil::kText, "hello ws");
    std::vector<uint8_t> buf(f.begin(), f.end());
    wsutil::Frame fr;
    size_t used = 0;
    CHECK(wsutil::tryDecodeFrame(buf, fr, used), "decode 失败");
    CHECK(fr.opcode == wsutil::kText && fr.payload == "hello ws" && used == f.size(), "帧内容错");
    // 掐断：数据不足应等待
    std::vector<uint8_t> part(f.begin(), f.begin() + 5);
    CHECK(!wsutil::tryDecodeFrame(part, fr, used), "半帧应等待");

    // 客户端掩码帧（手工构造，模拟浏览器）
    std::string payload = R"({"type":"grip","g":0.5})";
    std::string cf;
    cf += char(0x81);
    cf += char(0x80 | uint8_t(payload.size()));
    uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
    for (int i = 0; i < 4; i++) cf += char(mask[i]);
    for (size_t i = 0; i < payload.size(); i++) cf += char(payload[i] ^ mask[i % 4]);
    std::vector<uint8_t> cbuf(cf.begin(), cf.end());
    CHECK(wsutil::tryDecodeFrame(cbuf, fr, used), "掩码帧解码失败");
    CHECK(fr.payload == payload, "掩码解包错: %s", fr.payload.c_str());

    // 长帧（len=126 路径）
    std::string longp(200, 'x');
    std::string lf = wsutil::encodeFrame(wsutil::kBinary, longp);
    std::vector<uint8_t> lbuf(lf.begin(), lf.end());
    CHECK(wsutil::tryDecodeFrame(lbuf, fr, used) && fr.payload.size() == 200, "长帧");
    CHECK(!fr.masked, "服务器帧应无掩码");

    // P0-2 回归：64 位长度回绕 / 超上限帧 → 协议错误（err），且非「数据不够」
    std::string ov;
    ov += char(0x82);
    ov += char(0x7F);
    ov += char(0x80);                       // 最长度最高位 1 → 协议错误
    for (int i = 0; i < 7; i++) ov += char(0);
    std::vector<uint8_t> obuf(ov.begin(), ov.end());
    bool err = false;
    CHECK(!wsutil::tryDecodeFrame(obuf, fr, used, &err) && err, "64 位长度最高位应协议错误");
    std::string ov2;
    ov2 += char(0x82);
    ov2 += char(0x7F);
    ov2 += char(0x7F);                      // len = 2^63−1（MSB 0，通过最高位检查）
    for (int i = 0; i < 7; i++) ov2 += char(0xFF);
    std::vector<uint8_t> obuf2(ov2.begin(), ov2.end());
    err = false;
    CHECK(!wsutil::tryDecodeFrame(obuf2, fr, used, &err) && err, "超上限长帧应协议错误（resize OOM/回绕防护）");
    // 数据不足 ≠ 协议错误
    std::string pv;
    pv += char(0x82);
    pv += char(0x7E);
    pv += char(0x01);                       // 声明 382 字节，只给 2 字节头
    std::vector<uint8_t> pbuf(pv.begin(), pv.end());
    err = false;
    CHECK(!wsutil::tryDecodeFrame(pbuf, fr, used, &err) && !err, "半帧应等待且非协议错误");
  }

  if (g_fail == 0) std::printf("test_json_ws PASS\n");
  else std::printf("test_json_ws FAIL (%d)\n", g_fail);
  return g_fail ? 1 : 0;
}
