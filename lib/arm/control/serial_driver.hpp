// lib/arm/control/serial_driver.hpp — 串口驱动（真机路径硬化版）
//
// 帧协议（小端）：
//   [0xAA][0x55][type:u8][len:u8][payload...][crc16:u16]
//   type:
//     0x01 CMD_POS    payload = 6×int16 目标关节角 (0.001 rad)
//     0x02 CMD_VEL    payload = 6×int16 关节速度 (0.001 rad/s)
//     0x03 CMD_GRIP   payload = 1×uint8 开合 (0..255)
//     0x04 CMD_ESTOP  payload = 1×uint8 (0/1)
//     0x10 STATE_REP  payload = 6×int16 实际关节角 + 6×int16 速度 + 1×uint8 夹爪
//     0x11 STATE_REQ  payload = 空
// CRC16-CCITT (poly 0x1021, init 0xFFFF) 覆盖 type..payload。
//
// ── 角色（Role）：显式区分两种用法，消除「语义倒置」 ──────────────────────────
//   Master（默认，真机语义）：本端 = 控制上位机
//     · 下行：sendCmdPos/sendCmdVel/sendCmdGrip/sendCmdEstop（CMD_*）
//     · 上行：STATE_REP → hasRemoteState()/remoteState()
//     · broadcastState() 只缓存最近状态帧（供 STATE_REQ 应答），不产生周期性下行流量
//   Pendant（示教器语义，兼容旧行为）：对端 = 指令源
//     · 下行：broadcastState() 定周期推 STATE_REP
//     · 上行：CMD_* → pollCommands() 映射为 JSON 指令
//
// ── 写路径硬化（E1/E2） ────────────────────────────────────────────────────
//   · 有界待发队列（默认 16 KiB）：满时丢**最旧**帧并计数。理由：CMD_POS/CMD_VEL 是
//     置位式指令，最新意图比陈旧积压更重要；丢弃绝不静默（txStats().droppedFull）。
//   · 部分写续传（按帧记 offset）；EAGAIN/EWOULDBLOCK/EINTR → 下拍继续，不丢帧；
//     真错误（EIO/ENXIO/EBADF…）→ 标记离线 + 计数（online()/txStats().ioErrors 可观察）。
//   · ESTOP 帧插队：入队时清空在途数据（急停取代一切），因此**永不因队列满被丢弃**。
//   · 离线（无设备 / 已断开）时不再入队，直接计入 droppedOffline —— 杜绝
//     「无人取走的队列无限增长」（原实现 txBuf_ 在 50 Hz 广播下每小时 +5.6 MB）。
//   · 读缓冲同样设上限（默认 4 KiB）：半截帧/垃圾流不再无限滞留，超限清空并计数。
//
// ── 可测试性 ──────────────────────────────────────────────────────────────
//   · ByteIo 抽象（真机 = FdByteIo 包 termios fd；测试 = 注入假串口 / PTY 从机）
//   · pumpTx(maxBytes) 可显式驱动，便于确定性验证部分写/EAGAIN/错误路径
#pragma once
#include "arm/control/control_interface.hpp"
#include "arm/control/json.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
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

// ---------- 底层字节 IO 抽象（真机 = fd；测试 = 假串口 / PTY） ----------
class ByteIo {
 public:
  virtual ~ByteIo() = default;
  // 返回 >0 写入字节数；0 表示无法推进；<0 表示错误（必须设置 errno）
  virtual ssize_t writeSome(const uint8_t* d, size_t n) = 0;
  // 返回 >0 读出字节数；0 表示暂无数据；<0 表示错误（必须设置 errno）
  virtual ssize_t readSome(uint8_t* d, size_t n) = 0;
  virtual bool valid() const = 0;
  virtual void close() {}
};

#ifndef _WIN32
class FdByteIo : public ByteIo {
 public:
  explicit FdByteIo(int fd) : fd_(fd) {}
  ~FdByteIo() override { close(); }
  ssize_t writeSome(const uint8_t* d, size_t n) override {
    if (fd_ < 0) { errno = EBADF; return -1; }
    return ::write(fd_, d, n);
  }
  ssize_t readSome(uint8_t* d, size_t n) override {
    if (fd_ < 0) { errno = EBADF; return -1; }
    return ::read(fd_, d, n);
  }
  bool valid() const override { return fd_ >= 0; }
  void close() override {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  int fd() const { return fd_; }

 private:
  int fd_ = -1;
};
#endif

class SerialDriver : public ControlInterface {
 public:
  // 角色：Master = 真机语义（发指令/收状态）；Pendant = 示教器语义（推状态/收指令）
  enum class Role { Master, Pendant };

  // 待发队列与发送统计（丢弃**绝不静默**）：全部可查询
  struct TxStats {
    size_t queuedBytes = 0;       // 当前积压字节
    size_t queuedFrames = 0;      // 当前积压帧数
    uint64_t enqueued = 0;        // 累计入队帧数
    uint64_t written = 0;         // 累计写入设备字节
    uint64_t droppedFull = 0;     // 因队列满丢弃（丢最旧）
    uint64_t droppedOffline = 0;  // 离线（无设备/已断开）时直接丢弃
    uint64_t droppedOversize = 0; // 单帧超过队列上限（协议 len ≤ 255，正常不会发生）
    uint64_t superseded = 0;      // 被 ESTOP 清空的在途帧
    uint64_t coalesced = 0;       // 状态帧合并（尾部状态帧被更新值替换）
    uint64_t retries = 0;         // 本轮 write 返回 EAGAIN/EINTR 的次数（下拍续传）
    uint64_t ioErrors = 0;        // 硬错误次数（触发离线）
    uint64_t rxOverflow = 0;      // 读缓冲超限清空次数
  };

  // 收到的下位机状态（Master 角色解析 STATE_REP）
  struct RemoteState {
    bool valid = false;
    std::array<double, 6> q{}, qd{};
    double grip = 0.0;
    uint64_t frames = 0;   // 累计收到的 STATE_REP 帧数
  };

  explicit SerialDriver(std::string device, int baud = 115200, Role role = Role::Master)
      : device_(std::move(device)), baud_(baud), role_(role) {}
  ~SerialDriver() override { closePort(); }

  // 协议上限：len 为 u8 ⇒ 单帧 ≤ 261 B
  static constexpr size_t kMaxFrameBytes = 261;
  static constexpr size_t kDefaultTxLimit = 16 << 10;   // 16 KiB（≈50 Hz 状态帧 8 s 积压）
  static constexpr size_t kDefaultRxLimit = 4 << 10;    // 4 KiB 读缓冲上限

  bool begin() override {
#ifndef _WIN32
    int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }
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
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }
    io_ = std::make_unique<FdByteIo>(fd);
    online_ = true;
    return true;
#else
    return false;
#endif
  }

  // 注入式底层 IO（测试用：假串口 / PTY 从机）。传入后由本类接管生命周期。
  void setByteIo(std::unique_ptr<ByteIo> io) {
    io_ = std::move(io);
    online_ = io_ && io_->valid();
  }
  bool online() const { return online_; }
  Role role() const { return role_; }
  void setRole(Role r) { role_ = r; }

  // 队列上限（测试可调小以复现「队列满」路径）
  void setTxQueueLimit(size_t bytes) { txLimit_ = bytes; }
  void setRxBufferLimit(size_t bytes) { rxLimit_ = bytes; }
  size_t txQueueLimit() const { return txLimit_; }
  // 返回**即时**快照：queuedBytes/queuedFrames 由当前队列计算（出队后也准确）
  TxStats txStats() const {
    TxStats s = stats_;
    s.queuedBytes = queuedBytes_;
    s.queuedFrames = txq_.size();
    return s;
  }
  void resetTxStats() { stats_ = TxStats{}; }

  // ---------- 下行（Master 角色：真机指令） ----------
  void sendCmdPos(const std::array<double, 6>& q) {
    using namespace serialproto;
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, clampI16(q[i] * 1000.0));
    enqueue(encode(kCmdPos, p), Kind::kCommand);
  }
  void sendCmdVel(const std::array<double, 6>& qd) {
    using namespace serialproto;
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, clampI16(qd[i] * 1000.0));
    enqueue(encode(kCmdVel, p), Kind::kCommand);
  }
  void sendCmdGrip(double g) {
    using namespace serialproto;
    enqueue(encode(kCmdGrip, {uint8_t(std::min(1.0, std::max(0.0, g)) * 255.0)}), Kind::kCommand);
  }
  void sendCmdEstop(bool on) {
    using namespace serialproto;
    enqueue(encode(kCmdEstop, {uint8_t(on ? 1 : 0)}), Kind::kEstop);
  }

  // ---------- 状态广播 ----------
  // Pendant → 入队下行；Master → 仅缓存（真机语义下本端不周期推状态）
  void broadcastState(const StateSnapshot& st) override {
    using namespace serialproto;
    std::vector<uint8_t> p;
    for (int i = 0; i < 6; i++) putI16(p, clampI16(st.q[i] * 1000.0));
    for (int i = 0; i < 6; i++) putI16(p, clampI16(st.qd[i] * 1000.0));
    p.push_back(uint8_t(std::min(1.0, std::max(0.0, st.grip)) * 255.0));
    lastStateFrame_ = encode(kStateRep, p);
    if (role_ == Role::Pendant) enqueue(lastStateFrame_, Kind::kState);
  }

  // JSON 指令 → 串口帧；同构映射以便主循环统一派发
  std::vector<json::Value> pollCommands() override {
    using namespace serialproto;
    readInto();
    std::vector<json::Value> out;
    FrameOut fr;
    bool bad = false;
    while (tryDecode(rxBuf_, fr, bad) > 0) {
      if (role_ == Role::Master) {
        // 真机语义：对端回的是 STATE_REP
        if (fr.type == kStateRep) {
          RemoteState rs;
          rs.valid = true;
          rs.frames = remote_.frames + 1;
          for (size_t i = 0; i + 1 < fr.payload.size() && i / 2 < 6; i += 2)
            rs.q[i / 2] = double(getI16(fr.payload.data() + i)) / 1000.0;
          size_t off = 12;
          for (size_t i = 0; i + 1 < fr.payload.size() && off + i + 1 < fr.payload.size() && i / 2 < 6;
               i += 2)
            rs.qd[i / 2] = double(getI16(fr.payload.data() + off + i)) / 1000.0;
          rs.grip = (fr.payload.size() > off + 12) ? fr.payload[off + 12] / 255.0 : 0.0;
          remote_ = rs;
        } else if (fr.type == kStateReq) {
          if (!lastStateFrame_.empty()) enqueue(lastStateFrame_, Kind::kState);
        }
        fr = FrameOut{};
        continue;
      }
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
          if (!lastStateFrame_.empty()) enqueue(lastStateFrame_, Kind::kState);
          break;
        }
        default:
          break;
      }
      fr = FrameOut{};
    }
    return out;
  }

  // 急停：清空在途数据并**置顶**入队 ESTOP（队列满也不会丢），立即尝试发出
  void emergencyStop() override {
    using namespace serialproto;
    std::vector<uint8_t> f = encode(kCmdEstop, {1});
    stats_.superseded += txq_.size();
    txq_.clear();
    queuedBytes_ = 0;
    rxBuf_.clear();
    enqueue(std::move(f), Kind::kEstop);
    pumpTx(kPumpBytesPerCall);
  }

  // 每拍服务：重试上次 EAGAIN/部分写的剩余字节（下拍续传）
  void service(int timeoutMs) override {
    (void)timeoutMs;
    pumpTx(kPumpBytesPerCall);
  }

  const RemoteState& remoteState() const { return remote_; }
  bool hasRemoteState() const { return remote_.valid; }

  // 测试钩子：注入接收字节 / 取出发送队列（未发出部分，诊断与测试用）
  void injectRx(const std::vector<uint8_t>& bytes) {
    rxBuf_.insert(rxBuf_.end(), bytes.begin(), bytes.end());
    if (rxBuf_.size() > rxLimit_) { rxBuf_.clear(); stats_.rxOverflow++; }
  }
  std::vector<uint8_t> takeTx() {
    std::vector<uint8_t> t;
    for (auto& p : txq_)
      t.insert(t.end(), p.bytes.begin() + long(p.sent), p.bytes.end());
    txq_.clear();
    queuedBytes_ = 0;
    return t;
  }

  // 排空待发队列到设备（非阻塞）。返回本轮写入字节数。
  // 部分写 → 保留下次偏移；EAGAIN/EWOULDBLOCK/EINTR → 本轮停止，下拍续传；
  // 真错误 → 计数并离线（后续 enqueue 记 droppedOffline）。
  size_t pumpTx(size_t maxBytes = 4096) {
    if (txq_.empty()) return 0;
    if (!io_ || !io_->valid()) { online_ = false; return 0; }
    size_t written = 0;
    while (!txq_.empty() && written < maxBytes) {
      Pending& p = txq_.front();
      size_t remain = p.bytes.size() - p.sent;
      size_t want = std::min(remain, maxBytes - written);
      ssize_t n = io_->writeSome(p.bytes.data() + p.sent, want);
      if (n > 0) {
        p.sent += size_t(n);
        written += size_t(n);
        stats_.written += size_t(n);
        if (p.sent == p.bytes.size()) {
          queuedBytes_ -= p.bytes.size();
          txq_.pop_front();
        }
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        stats_.retries++;      // 写不进 → 下拍续传（不丢帧）
        break;
      } else {
        stats_.ioErrors++;     // 真错误 → 离线（上层经 online() 可观察）
        markOffline();
        break;
      }
    }
    return written;
  }

 private:
  struct Pending {
    std::vector<uint8_t> bytes;
    size_t sent = 0;
  };
  enum class Kind { kState, kCommand, kEstop };

  static constexpr size_t kPumpBytesPerCall = 4096;   // 单次 service 的写入上限（防饿死控制回路）

  static int16_t clampI16(double v) {
    if (!(v > -32768.0)) return int16_t(-32768);       // 含 NaN → 下限
    if (v > 32767.0) return int16_t(32767);
    return int16_t(v);
  }

  void readInto() {
    if (!io_ || !io_->valid()) return;
    uint8_t tmp[512];
    for (int k = 0; k < 8; k++) {                      // 单拍最多 4 KiB，避免饿死控制回路
      ssize_t n = io_->readSome(tmp, sizeof tmp);
      if (n > 0) {
        rxBuf_.insert(rxBuf_.end(), tmp, tmp + n);
        if (rxBuf_.size() > rxLimit_) {                 // 半截帧/垃圾流不再无限滞留
          rxBuf_.clear();
          stats_.rxOverflow++;
        }
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        break;
      } else if (n == 0) {
        break;
      } else {
        stats_.ioErrors++;
        markOffline();
        break;
      }
    }
  }

  // 入队策略（明确，且全部计数）：
  //   ESTOP  → 清空在途 + 置顶（急停优先，永不因满而丢）
  //   状态帧 → 尾部若已是状态帧则直接替换（合并），否则按普通帧处理
  //   队列满 → 丢**最旧**帧（置位式指令保留最新意图），droppedFull 递增
  //   离线    → 不入队，droppedOffline 递增（避免无人取走导致增长）
  void enqueue(std::vector<uint8_t> f, Kind k) {
    if (f.size() > kMaxFrameBytes || f.size() > txLimit_) { stats_.droppedOversize++; return; }
    if (k == Kind::kEstop) {
      stats_.superseded += txq_.size();
      txq_.clear();
      queuedBytes_ = 0;
    }
    if (!online_ || !io_ || !io_->valid()) { stats_.droppedOffline++; return; }
    if (k == Kind::kState && !txq_.empty() && stateFrameAtTail_ && txq_.back().sent == 0) {
      // 仅合并「尚未开始发送」的帧：已部分写入的帧不能改写（否则线上的前半段与新的后半段拼接成坏帧）
      queuedBytes_ -= txq_.back().bytes.size();
      txq_.back().bytes = std::move(f);
      queuedBytes_ += txq_.back().bytes.size();
      stats_.coalesced++;
      stats_.queuedBytes = queuedBytes_;
      stats_.queuedFrames = txq_.size();
      return;
    }
    while (!txq_.empty() && queuedBytes_ + f.size() > txLimit_) {
      // 只丢最旧**未开始发送**的帧；已部分写入的帧留在队列里（丢弃会让线路上出现半截帧，
      // 接收端要到下一次 0xAA55 才能重新同步）。pumpTx 一次只推进队首帧，故至多一个半发送帧。
      size_t victim = txq_.size();
      for (size_t i = 0; i < txq_.size(); i++)
        if (txq_[i].sent == 0) { victim = i; break; }
      if (victim == txq_.size()) { stats_.droppedOversize++; return; }  // 无可用牺牲者：拒新帧
      queuedBytes_ -= (txq_[victim].bytes.size() - txq_[victim].sent);
      txq_.erase(txq_.begin() + long(victim));
      stats_.droppedFull++;
    }
    stateFrameAtTail_ = (k == Kind::kState);
    queuedBytes_ += f.size();
    txq_.push_back(Pending{std::move(f), 0});
    stats_.enqueued++;
    stats_.queuedBytes = queuedBytes_;
    stats_.queuedFrames = txq_.size();
  }

  void markOffline() {
    if (io_) io_->close();
    online_ = false;
    // 在途帧已不可能发出：清空并计入 droppedOffline（内存立即回落，且计数可见）
    if (!txq_.empty()) {
      stats_.droppedOffline += txq_.size();
      txq_.clear();
      queuedBytes_ = 0;
      stateFrameAtTail_ = false;
    }
  }

  void closePort() {
    markOffline();
  }

  std::string device_;
  int baud_;
  Role role_ = Role::Master;
  std::unique_ptr<ByteIo> io_;
  bool online_ = false;
  bool stateFrameAtTail_ = false;
  size_t txLimit_ = kDefaultTxLimit;
  size_t rxLimit_ = kDefaultRxLimit;
  size_t queuedBytes_ = 0;
  std::deque<Pending> txq_;
  TxStats stats_;
  RemoteState remote_;
  std::vector<uint8_t> rxBuf_, lastStateFrame_;
};

}  // namespace arm
