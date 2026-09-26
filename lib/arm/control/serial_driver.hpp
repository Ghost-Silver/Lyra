// lib/arm/control/serial_driver.hpp — 串口驱动骨架（真机预留）
// 帧协议（小端）：
//   [0xAA][0x55][type:u8][len:u8][payload...][crc16:u16]
//   type:
//     0x01 CMD_POS   payload = 6×int16 目标关节角 (0.001 rad)
//     0x02 CMD_VEL   payload = 6×int16 关节速度 (0.001 rad/s)
//     0x03 CMD_GRIP  payload = 1×uint8 开合 (0..255)
//     0x04 CMD_ESTOP payload = 1×uint8 (0/1)
//     0x10 STATE_REP payload = 6×int16 实际关节角 + 6×int16 速度 + 1×uint8 夹爪
//     0x11 STATE_REQ payload = 空
// CRC16-CCITT (poly 0x1021, init 0xFFFF) 覆盖 type..payload。
// 本骨架完整实现帧编解码 + CRC（可单测）；设备 I/O 为 POSIX termios 薄封装，
// 无硬件时 send 直接返回 false、poll 返回空（纯仿真阶段仅保留接口）。
#pragma once
#include "arm/control/control_interface.hpp"
#include "arm/control/json.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace arm {

namespace serialproto {
enum : uint8_t {
  kCmdPos = 0x01, kCmdVel = 0x02, kCmdGrip = 0x03, kCmdEstop = 0x04,
  kStateRep = 0x10, kStateReq = 0x11,
};

inline uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= uint16_t(d[i]) << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

inline std::vector<uint8_t> encode(uint8_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> f;
  f.push_back(0xAA);
  f.push_back(0x55);
  f.push_back(type);
  f.push_back(uint8_t(payload.size()));
  f.insert(f.end(), payload.begin(), payload.end());
  uint16_t crc = crc16(f.data() + 2, f.size() - 2);
  f.push_back(uint8_t(crc & 0xFF));       // 小端
  f.push_back(uint8_t(crc >> 8));
  return f;
}

inline void putI16(std::vector<uint8_t>& p, int16_t v) {
  p.push_back(uint8_t(v & 0xFF));
  p.push_back(uint8_t((v >> 8) & 0xFF));
}
inline int16_t getI16(const uint8_t* p) { return int16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8)); }

// 从字节流里抠出完整帧；成功返回帧长（含头尾），坏 CRC 跳过该帧返回已消费字节并置 badCrc
struct FrameOut {
  uint8_t type = 0;
  std::vector<uint8_t> payload;
};
inline size_t tryDecode(std::vector<uint8_t>& buf, FrameOut& out, bool& badCrc) {
  badCrc = false;
  size_t consumed = 0;
  while (buf.size() - consumed >= 6) {
    // 找头
    size_t i = consumed;
    while (i + 1 < buf.size() && !(buf[i] == 0xAA && buf[i + 1] == 0x55)) i++;
    if (i + 5 >= buf.size()) { consumed = i; break; }
    uint8_t type = buf[i + 2];
    uint8_t len = buf[i + 3];
    if (buf.size() - i < size_t(6 + len)) { consumed = i; break; }
    uint16_t crc = uint16_t(buf[i + 4 + len]) | (uint16_t(buf[i + 5 + len]) << 8);
    uint16_t calc = crc16(buf.data() + i + 2, size_t(2 + len));
    if (crc != calc) {
      badCrc = true;
      consumed = i + 2;   // 跳过坏头
      continue;
    }
    out.type = type;
    out.payload.assign(buf.begin() + long(i + 4), buf.begin() + long(i + 4 + len));
    consumed = i + size_t(6 + len);
    break;
  }
  if (consumed > 0) buf.erase(buf.begin(), buf.begin() + long(consumed));
  return consumed;
}
}  // namespace serialproto

class SerialDriver : public ControlInterface {
 public:
  explicit SerialDriver(std::string device, int baud = 115200)
      : device_(std::move(device)), baud_(baud) {}
  ~SerialDriver() override { closePort(); }

  bool begin() override {
#ifndef _WIN32
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { closePort(); return false; }
    cfmakeraw(&tty);
    speed_t sp = B115200;
    if (baud_ == 9600) sp = B9600;
    else if (baud_ == 57600) sp = B57600;
    else if (baud_ == 230400) sp = B230400;
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { closePort(); return false; }
    online_ = true;
    return true;
#else
    return false;
#endif
  }

  // 状态广播 → 下行 STATE 帧（真机为请求/应答式，这里简化为定周期推送）
  void broadcastState(const StateSnapshot& st) override {
    using namespace serialproto;
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, int16_t(st.q[i] * 1000.0));
    for (int i = 0; i < 6; i++) putI16(p, int16_t(st.qd[i] * 1000.0));
    p.push_back(uint8_t(st.grip * 255.0));
    lastStateFrame_ = encode(kStateRep, p);
    sendBytes(lastStateFrame_);
  }

  // JSON 指令 → 串口帧；同构映射以便主循环统一派发
  std::vector<json::Value> pollCommands() override {
    using namespace serialproto;
    std::vector<json::Value> out;
#ifndef _WIN32
    if (fd_ >= 0) {
      uint8_t tmp[512];
      while (true) {
        ssize_t n = ::read(fd_, tmp, sizeof tmp);
        if (n > 0) rxBuf_.insert(rxBuf_.end(), tmp, tmp + n);
        else break;
      }
    }
#endif
    FrameOut fr;
    bool bad = false;
    while (tryDecode(rxBuf_, fr, bad) > 0) {
      json::Value v = json::Value::object();
      switch (fr.type) {
        case kCmdPos: {
          v.set("type", json::Value("joint_target"));
          json::Value q = json::Value::array();
          for (size_t i = 0; i + 1 < fr.payload.size() && i / 2 < 6; i += 2)
            q.pushBack(json::Value(double(getI16(fr.payload.data() + i)) / 1000.0));
          v.set("q", q);
          out.push_back(std::move(v));
          break;
        }
        case kCmdVel: {
          v.set("type", json::Value("joint_vel"));
          json::Value q = json::Value::array();
          for (size_t i = 0; i + 1 < fr.payload.size() && i / 2 < 6; i += 2)
            q.pushBack(json::Value(double(getI16(fr.payload.data() + i)) / 1000.0));
          v.set("qd", q);
          out.push_back(std::move(v));
          break;
        }
        case kCmdGrip:
          v.set("type", json::Value("grip"));
          v.set("g", json::Value(fr.payload.empty() ? 0.0 : fr.payload[0] / 255.0));
          out.push_back(std::move(v));
          break;
        case kCmdEstop:
          v.set("type", json::Value("estop"));
          v.set("on", json::Value(!fr.payload.empty() && fr.payload[0] != 0));
          out.push_back(std::move(v));
          break;
        case kStateReq: {
          if (!lastStateFrame_.empty()) sendBytes(lastStateFrame_);
          break;
        }
        default:
          break;
      }
      fr = FrameOut{};
    }
    return out;
  }

  void emergencyStop() override {
    using namespace serialproto;
    sendBytes(encode(kCmdEstop, {1}));
    rxBuf_.clear();
  }

  bool online() const { return online_; }
  // 测试钩子：注入接收字节 / 取出发送字节
  void injectRx(const std::vector<uint8_t>& bytes) {
    rxBuf_.insert(rxBuf_.end(), bytes.begin(), bytes.end());
  }
  std::vector<uint8_t> takeTx() {
    std::vector<uint8_t> t;
    t.swap(txBuf_);
    return t;
  }

 private:
  void closePort() {
#ifndef _WIN32
    if (fd_ >= 0) ::close(fd_);
#endif
    fd_ = -1;
    online_ = false;
  }
  void sendBytes(const std::vector<uint8_t>& b) {
    txBuf_.insert(txBuf_.end(), b.begin(), b.end());
#ifndef _WIN32
    if (fd_ >= 0) ::write(fd_, b.data(), b.size());
#endif
  }

  std::string device_;
  int baud_;
  int fd_ = -1;
  bool online_ = false;
  std::vector<uint8_t> rxBuf_, txBuf_, lastStateFrame_;
};

}  // namespace arm
