// lib/arm/control/ws_server.hpp — WebSocket 服务实装 (RFC6455) + 静态 HTTP
// 自包含 socket / SHA-1 / Base64 / JSON：
//   - HTTP GET 静态文件（web/ 目录，realpath 前缀校验 + O_NOFOLLOW：路径穿越/symlink 逃逸均拒绝）
//   - GET /ws 升级为 WebSocket（握手校验 Connection/Version/Key、帧编解码、ping/pong、分片、close；
//     客户端帧强制掩码、控制帧 ≤125 禁分片、孤立 Cont 拒收、帧/消息/缓冲三重上限）
//   - GET /api/health 健康检查
//   - 广播机器人状态 JSON / 接收指令 JSON；Origin 默认放行（实验室工具 + 预览代理兼容），
//     setOriginAllowlist() 可配置白名单；连接数/握手超时/发送队列全局限额
// 发送路径严格非阻塞（有界队列 + 轮询冲刷），慢客户端只会被丢弃、绝不拖死控制回路。
// POSIX 实现，单线程 poll() 驱动，由主循环 service() 推进。
#pragma once
#include "arm/control/control_interface.hpp"
#include "arm/control/json.hpp"
#include <functional>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace arm {

// ---------------- SHA-1（RFC 3174，握手用） ----------------
namespace wsutil {
inline uint32_t rotl32(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }
inline void sha1(const uint8_t* data, size_t len, uint8_t out[20]) {
  uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
  std::vector<uint8_t> msg(data, data + len);
  msg.push_back(0x80);
  while ((msg.size() % 64) != 56) msg.push_back(0);
  uint64_t bits = uint64_t(len) * 8;
  for (int i = 7; i >= 0; i--) msg.push_back(uint8_t((bits >> (8 * i)) & 0xFF));
  for (size_t off = 0; off < msg.size(); off += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++)
      w[i] = (uint32_t(msg[off + 4 * i]) << 24) | (uint32_t(msg[off + 4 * i + 1]) << 16) |
             (uint32_t(msg[off + 4 * i + 2]) << 8) | uint32_t(msg[off + 4 * i + 3]);
    for (int i = 16; i < 80; i++) w[i] = rotl32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; i++) {
      uint32_t f, k;
      if (i < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
      else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
      else { f = b ^ c ^ d; k = 0xCA62C1D6; }
      uint32_t tmp = rotl32(a, 5) + f + e + k + w[i];
      e = d; d = c; c = rotl32(b, 30); b = a; a = tmp;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
  }
  for (int i = 0; i < 5; i++) {
    out[4 * i] = uint8_t(h[i] >> 24);
    out[4 * i + 1] = uint8_t(h[i] >> 16);
    out[4 * i + 2] = uint8_t(h[i] >> 8);
    out[4 * i + 3] = uint8_t(h[i]);
  }
}
inline std::string base64(const uint8_t* data, size_t len) {
  static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string o;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = uint32_t(data[i]) << 16;
    if (i + 1 < len) v |= uint32_t(data[i + 1]) << 8;
    if (i + 2 < len) v |= uint32_t(data[i + 2]);
    o += T[(v >> 18) & 63];
    o += T[(v >> 12) & 63];
    o += (i + 1 < len) ? T[(v >> 6) & 63] : '=';
    o += (i + 2 < len) ? T[v & 63] : '=';
  }
  return o;
}
// RFC6455 握手：accept = base64(sha1(key + GUID))
inline std::string acceptKey(const std::string& clientKey) {
  static const char* GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  std::string s = clientKey + GUID;
  uint8_t dig[20];
  sha1(reinterpret_cast<const uint8_t*>(s.data()), s.size(), dig);
  return base64(dig, 20);
}

// ---------------- WS 帧编解码 ----------------
enum Opcode : uint8_t { kCont = 0x0, kText = 0x1, kBinary = 0x2, kClose = 0x8, kPing = 0x9, kPong = 0xA };

// 服务器 → 客户端（无 mask）
inline std::string encodeFrame(uint8_t opcode, const std::string& payload) {
  std::string f;
  f += char(0x80 | (opcode & 0x0F));
  size_t n = payload.size();
  if (n < 126) f += char(n);
  else if (n < 65536) {
    f += char(126);
    f += char((n >> 8) & 0xFF);
    f += char(n & 0xFF);
  } else {
    f += char(127);
    for (int i = 7; i >= 0; i--) f += char((uint64_t(n) >> (8 * i)) & 0xFF);
  }
  f += payload;
  return f;
}

// 解析一帧；返回 true 表示消费了一帧（consumed = 帧字节数）。
// err（可选）：true = 协议错误（长度回绕/超上限/64 位长度最高位非 0）——调用方必须断连，
// 「数据不够」与「非法帧」不可混同。单帧负载上限 kMaxFrameLen。
struct Frame {
  bool fin = true;
  bool masked = false;
  uint8_t opcode = 0;
  std::string payload;
};
static constexpr uint64_t kMaxFrameLen = 1ull << 20;   // 单帧 1 MiB
inline bool tryDecodeFrame(const std::vector<uint8_t>& buf, Frame& out, size_t& consumed,
                           bool* err = nullptr) {
  if (err) *err = false;
  if (buf.size() < 2) return false;
  uint8_t b0 = buf[0], b1 = buf[1];
  out.fin = (b0 & 0x80) != 0;
  out.opcode = b0 & 0x0F;
  out.masked = (b1 & 0x80) != 0;
  uint64_t len = b1 & 0x7F;
  size_t pos = 2;
  if (len == 126) {
    if (buf.size() < pos + 2) return false;
    len = (uint64_t(buf[pos]) << 8) | buf[pos + 1];
    pos += 2;
  } else if (len == 127) {
    if (buf.size() < pos + 8) return false;
    if (buf[pos] & 0x80) { if (err) *err = true; return false; }   // RFC6455：最高位必 0
    len = 0;
    for (int i = 0; i < 8; i++) len = (len << 8) | buf[pos + i];
    pos += 8;
  }
  if (len > kMaxFrameLen) { if (err) *err = true; return false; }   // 帧长上限（回绕/OOM 防护）
  uint8_t mask[4] = {0, 0, 0, 0};
  if (out.masked) {
    if (buf.size() < pos + 4) return false;
    for (int i = 0; i < 4; i++) mask[i] = buf[pos + i];
    pos += 4;
  }
  if (len > buf.size() - pos) return false;   // 溢出安全写法（pos ≤ buf.size() 恒成立）
  out.payload.resize(size_t(len));
  for (uint64_t i = 0; i < len; i++) {
    uint8_t c = buf[pos + i];
    out.payload[size_t(i)] = char(out.masked ? (c ^ mask[i % 4]) : c);
  }
  consumed = pos + size_t(len);
  return true;
}
// 握手 key 合法性：base64(16 字节 nonce) = 恰 24 字符（含尾部 padding）
inline bool validWsKey(const std::string& k) {
  if (k.size() != 24) return false;
  static const char* B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for (size_t i = 0; i < 24; i++) {
    char c = k[i];
    if (i >= 22 && c == '=') continue;
    if (!std::strchr(B64, c)) return false;
  }
  return true;
}
}  // namespace wsutil

// ---------------- 服务器 ----------------
class WsServer : public ControlInterface {
 public:
  WsServer(int port, std::string webRoot) : port_(port), webRoot_(std::move(webRoot)) {}
  ~WsServer() override {
    for (auto& c : clients_) if (c.fd >= 0) ::close(c.fd);
    if (listenFd_ >= 0) ::close(listenFd_);
  }

  bool begin() override {
    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) return false;
    int one = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);   // 0.0.0.0：预览代理可达
    addr.sin_port = htons(uint16_t(port_));
    if (::bind(listenFd_, (sockaddr*)&addr, sizeof addr) < 0) return false;
    if (::listen(listenFd_, 16) < 0) return false;
    setNonBlock(listenFd_);
    // 静态根 realpath 固定（symlink 逃逸防护基准前缀）
    char* rp = ::realpath(webRoot_.c_str(), nullptr);
    if (!rp) return false;
    rootReal_ = rp;
    std::free(rp);
    return true;
  }

  // P1-7：Origin 白名单（空 = 放行所有；设置后须精确匹配，"*" 条目通配）
  void setOriginAllowlist(std::vector<std::string> allow) { allowedOrigins_ = std::move(allow); }
  // 健康检查扩展钩子：宿主可注入额外字段（如安全层计数），保持 ws_server 与业务解耦
  void setHealthExtra(std::function<void(json::Value&)> fn) { healthExtra_ = std::move(fn); }

  void service(int timeoutMs) override {
    std::vector<pollfd> pfds;
    pfds.push_back({listenFd_, POLLIN, 0});
    for (auto& c : clients_) pfds.push_back({c.fd, POLLIN, 0});
    int r = ::poll(pfds.data(), nfds_t(pfds.size()), timeoutMs);
    if (r <= 0) return;
    if (pfds[0].revents & POLLIN) acceptAll();
    for (size_t i = 1; i < pfds.size(); i++) {
      if (i - 1 >= clients_.size()) break;
      if (pfds[i].revents & (POLLIN | POLLHUP | POLLERR)) onReadable(i - 1);
    }
    // slowloris/半帧超时：握手 5s 未齐、或 WS 读缓冲/分片挂 5s → 断开；健康空闲 WS 不杀
    auto now = Clock::now();
    for (auto& c : clients_) {
      if (!c.alive) continue;
      bool partial = !c.isWs || !c.buf.empty() || c.fragOpen;
      if (partial && now - c.lastAct > std::chrono::seconds(5)) c.alive = false;
    }
    for (auto& c : clients_) if (c.alive && !c.outBuf.empty()) flushOut(c);
    // 清理死连接
    for (size_t i = clients_.size(); i-- > 0;)
      if (!clients_[i].alive) {
        ::close(clients_[i].fd);
        clients_.erase(clients_.begin() + long(i));
      }
  }

  void broadcastState(const StateSnapshot& st) override { broadcast(st.toJson()); }

  void broadcast(const json::Value& msg) {
    std::string payload = msg.dump();
    std::string frame = wsutil::encodeFrame(wsutil::kText, payload);
    for (auto& c : clients_)
      if (c.alive && c.isWs) sendRaw(c, frame);
  }

  void sendToLast(const json::Value& msg) override {
    std::string frame = wsutil::encodeFrame(wsutil::kText, msg.dump());
    for (size_t i = clients_.size(); i-- > 0;)
      if (clients_[i].alive && clients_[i].isWs) { sendRaw(clients_[i], frame); return; }
  }

  std::vector<json::Value> pollCommands() override {
    std::vector<json::Value> out;
    out.swap(inbox_);
    return out;
  }

  void emergencyStop() override {
    inbox_.clear();
    json::Value v = json::Value::object();
    v.set("type", json::Value("estop"));
    v.set("on", json::Value(true));
    broadcast(v);
  }

  size_t httpClientCount() const { return clients_.size(); }
  size_t rejectedClients() const { return rejectedClients_; }
  size_t headerRejects() const { return headerRejects_; }
  size_t cacheHits() const { return cacheHits_; }
  size_t cacheMisses() const { return cacheMisses_; }
  size_t wsClientCount() const {
    size_t n = 0;
    for (auto& c : clients_) if (c.alive && c.isWs) n++;
    return n;
  }

 private:
  using Clock = std::chrono::steady_clock;
  struct Client {
    int fd = -1;
    bool isWs = false;
    bool alive = true;
    std::vector<uint8_t> buf;      // 未消化字节
    std::string frag;              // 分片续帧缓存
    bool fragOpen = false;
    uint8_t fragOp = 0;
    std::string outBuf;            // 待发送字节（有界队列，非阻塞冲刷）
    Clock::time_point lastAct;     // 最近一次收字节时刻（slowloris 超时用）
  };
  // 全局限额（DoS 面收敛）
  static constexpr size_t kMaxClients = 16;        // 连接数上限
  static constexpr size_t kMaxHttpReq = 16 << 10;  // HTTP 请求头 16 KiB
  static constexpr size_t kMaxMessage = 1 << 20;   // 重组消息 1 MiB
  static constexpr size_t kMaxBuffer = 2 << 20;    // 读缓冲 2 MiB
  static constexpr size_t kMaxSendQueue = 256 << 10;  // 每连接发送队列 256 KiB
  // C1：静态文件走内存缓存 + 体积上限——实时控制线程内的磁盘 IO 只发生一次/文件，
  // 且单文件读取量有硬上限（预读式缓存，避免客户端反复拉大文件造成控制回路抖动）。
  // 上限必须 ≤ kMaxSendQueue：否则单次响应无法完整入队（会被当作慢客户端丢弃 → 截断）。
  // 实测：直读 2 MiB 文件一次 ≈ 6.4 ms（3 倍控制周期），故大文件改为明确拒绝（413）。
  static constexpr size_t kMaxStaticFile = 192 << 10;   // 单文件 192 KiB 上限（超出 413）
  static constexpr size_t kMaxStaticCache = 8 << 20;    // 静态缓存总上限 8 MiB

  static void setNonBlock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
  }

  void acceptAll() {
    while (true) {
      int fd = ::accept(listenFd_, nullptr, nullptr);
      if (fd < 0) break;
      if (clients_.size() >= kMaxClients) {
        // C2：连接数超限——先回 503（前端/运维可感知「服务忙」）再关闭，而非静默断开
        static const char kBusy[] =
            "HTTP/1.1 503 Service Unavailable\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: 4\r\n"
            "Connection: close\r\n\r\nbusy";
        ::send(fd, kBusy, sizeof(kBusy) - 1, MSG_NOSIGNAL);
        // C2：紧接 close 会让「未读的入站请求数据」触发 RST，把上面的 503 也丢弃——
        // 前端就感知不到「服务忙」。故：半关写端 → 非阻塞读掉请求 → 正常 FIN 关闭。
        ::shutdown(fd, SHUT_WR);
        {
          int fl = ::fcntl(fd, F_GETFL, 0);
          if (fl >= 0) ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
          char sink[2048];
          for (int k = 0; k < 4; k++) {          // 上限 8 KiB / 4 次，非阻塞，绝不挂起控制线程
            ssize_t n = ::recv(fd, sink, sizeof sink, 0);
            if (n <= 0) break;
          }
        }
        ::close(fd);
        rejectedClients_++;
        continue;
      }
      setNonBlock(fd);
      int one = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
      Client c;
      c.fd = fd;
      c.lastAct = Clock::now();
      clients_.push_back(c);
    }
  }

  // 发送严格非阻塞：能发就发，发不完进有界队列、下轮冲刷；队列超限直接断开。
  // 慢客户端只可能被丢弃，绝不阻塞 50Hz 控制/广播回路。
  static void flushOut(Client& c) {
    while (!c.outBuf.empty()) {
      ssize_t n = ::send(c.fd, c.outBuf.data(), c.outBuf.size(), MSG_NOSIGNAL);
      if (n > 0) { c.outBuf.erase(0, size_t(n)); continue; }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;   // 下轮再冲
      c.alive = false;
      return;
    }
  }
  static void sendRaw(Client& c, const std::string& bytes) {
    if (!c.alive) return;
    flushOut(c);
    if (!c.alive) return;
    if (c.outBuf.size() + bytes.size() > kMaxSendQueue) { c.alive = false; return; }
    c.outBuf += bytes;
    flushOut(c);
  }

  void onReadable(size_t idx) {
    Client& c = clients_[idx];
    uint8_t tmp[4096];
    while (true) {
      ssize_t n = ::recv(c.fd, tmp, sizeof tmp, 0);
      if (n > 0) c.buf.insert(c.buf.end(), tmp, tmp + n);
      else if (n == 0) { c.alive = false; return; }
      else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        c.alive = false;
        return;
      }
    }
    c.lastAct = Clock::now();
    if (c.buf.size() > kMaxBuffer) { c.alive = false; return; }   // 读缓冲上限
    if (!c.isWs) {
      serveHttp(c);
    } else {
      serveWs(c);
    }
  }

  // ---------- HTTP ----------
  void serveHttp(Client& c) {
    // 需包含完整请求头
    std::string req(c.buf.begin(), c.buf.end());
    size_t hdrEnd = req.find("\r\n\r\n");
    // C2：请求头上限统一判定——①头未结束但已超限（拒绝 slowloris 慢慢堆）；
    // ②头已结束但头部本身超限（此前 kMaxHttpReq 只声明未使用，超长头会当正常请求服务）
    const size_t hdrLen = (hdrEnd == std::string::npos) ? req.size() : hdrEnd;
    if (hdrLen > kMaxHttpReq) {
      headerRejects_++;
      c.alive = false;
      return;
    }
    if (hdrEnd == std::string::npos) return;  // 等待更多数据
    std::istringstream is(req.substr(0, hdrEnd));
    std::string method, path, ver;
    is >> method >> path >> ver;
    std::map<std::string, std::string> hdr;
    std::string line;
    while (std::getline(is, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      size_t colon = line.find(':');
      if (colon != std::string::npos) {
        std::string k = line.substr(0, colon), v = line.substr(colon + 1);
        while (!v.empty() && v[0] == ' ') v.erase(v.begin());
        for (auto& ch : k) ch = char(::tolower(ch));
        hdr[k] = v;
      }
    }
    c.buf.clear();

    if (method != "GET") { respond(c, "405 Method Not Allowed", "text/plain", "GET only"); c.alive = false; return; }

    // WS 升级（握手三件套校验：Connection: Upgrade / Version: 13 / Key 合法）
    auto up = hdr.find("upgrade");
    if (up != hdr.end()) {
      for (auto& ch : up->second) ch = char(::tolower(ch));
      if (up->second == "websocket") {
        auto conn = hdr.find("connection");
        bool connOk = false;
        if (conn != hdr.end()) {
          for (auto& ch : conn->second) ch = char(::tolower(ch));
          connOk = conn->second.find("upgrade") != std::string::npos;
        }
        auto ver = hdr.find("sec-websocket-version");
        bool verOk = false;
        if (ver != hdr.end()) {
          std::string vv = ver->second;
          while (!vv.empty() && vv.back() == ' ') vv.pop_back();
          verOk = (vv == "13");
        }
        auto key = hdr.find("sec-websocket-key");
        std::string keyv = key != hdr.end() ? key->second : std::string();
        while (!keyv.empty() && keyv.back() == ' ') keyv.pop_back();
        if (!connOk || !verOk || !wsutil::validWsKey(keyv)) {
          respond(c, "400 Bad Request", "text/plain", "bad websocket handshake");
          c.alive = false;
          return;
        }
        // P1-7 Origin 白名单：默认（空表）放行；配置后须精确匹配（无 Origin 的非浏览器客户端放行）
        if (!allowedOrigins_.empty()) {
          auto org = hdr.find("origin");
          bool okOrg = (org == hdr.end());
          if (!okOrg)
            for (auto& a : allowedOrigins_)
              if (a == "*" || a == org->second) { okOrg = true; break; }
          if (!okOrg) { respond(c, "403 Forbidden", "text/plain", "origin not allowed"); c.alive = false; return; }
        }
        std::string resp =
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " + wsutil::acceptKey(keyv) + "\r\n\r\n";
        sendRaw(c, resp);
        c.isWs = true;
        return;
      }
    }

    if (path == "/api/health") {
      json::Value v = json::Value::object();
      v.set("ok", json::Value(true));
      v.set("ws_clients", json::Value(double(wsClientCount())));
      v.set("http_clients", json::Value(double(clients_.size())));
      v.set("rejected_clients", json::Value(double(rejectedClients_)));   // C2：连接数超限拒绝数
      v.set("header_rejects", json::Value(double(headerRejects_)));       // C2：超长请求头丢弃数
      v.set("static_cache_hits", json::Value(double(cacheHits_)));
      v.set("static_cache_misses", json::Value(double(cacheMisses_)));
      v.set("static_cache_bytes", json::Value(double(staticCacheBytes_)));
      if (healthExtra_) healthExtra_(v);   // 宿主注入（安全层计数等）
      respond(c, "200 OK", "application/json", v.dump());
      return;
    }

    // 静态文件
    std::string rel = (path == "/" || path.empty()) ? "/index.html" : path;
    size_t q = rel.find('?');
    if (q != std::string::npos) rel = rel.substr(0, q);
    if (rel.find("..") != std::string::npos) { respond(c, "403 Forbidden", "text/plain", "no"); return; }
    // P1-8：realpath 解析后强制前缀校验（symlink 逃逸如 web/link.html→/etc/passwd 一律拒绝），
    // O_NOFOLLOW 打开兜底（目标本身是 symlink 也不跟）。
    std::string file = webRoot_ + rel;
    char* rp = ::realpath(file.c_str(), nullptr);
    if (!rp) { respond(c, "404 Not Found", "text/plain", "404 " + rel); return; }
    std::string real(rp);
    std::free(rp);
    if (real.size() <= rootReal_.size() + 1 ||
        real.compare(0, rootReal_.size() + 1, rootReal_ + "/") != 0) {
      respond(c, "403 Forbidden", "text/plain", "no");
      return;
    }
    // C1：命中缓存直接回（零磁盘 IO）；未命中读一次并缓存（受 kMaxStaticFile/Cache 约束）
    auto it = staticCache_.find(real);
    if (it != staticCache_.end()) {
      cacheHits_++;
      respond(c, "200 OK", mimeOf(rel), it->second);
      c.alive = false;
      return;
    }
    cacheMisses_++;
    int fd = ::open(real.c_str(), O_RDONLY | O_NOFOLLOW);
    if (fd < 0) { respond(c, "404 Not Found", "text/plain", "404 " + rel); return; }
    std::string body;
    {
      char tmp[4096];
      ssize_t nr;
      bool tooBig = false;
      while ((nr = ::read(fd, tmp, sizeof tmp)) > 0) {
        body.append(tmp, size_t(nr));
        if (body.size() > kMaxStaticFile) { tooBig = true; break; }   // 单文件上限（防大文件阻塞）
      }
      if (tooBig) {
        ::close(fd);
        respond(c, "413 Payload Too Large", "text/plain", "static file too large");
        c.alive = false;
        return;
      }
    }
    ::close(fd);
    if (staticCacheBytes_ + body.size() <= kMaxStaticCache) {
      staticCacheBytes_ += body.size();
      staticCache_.emplace(real, body);
    }
    respond(c, "200 OK", mimeOf(rel), body);
    c.alive = false;  // 短连接（页面资源量小；WS 长连接走升级路径）
  }

  static std::string mimeOf(const std::string& path) {
    auto ends = [&](const char* s) {
      size_t n = std::strlen(s);
      return path.size() >= n && path.compare(path.size() - n, n, s) == 0;
    };
    if (ends(".html")) return "text/html; charset=utf-8";
    if (ends(".js")) return "text/javascript; charset=utf-8";
    if (ends(".css")) return "text/css; charset=utf-8";
    if (ends(".json")) return "application/json";
    if (ends(".png")) return "image/png";
    if (ends(".jpg") || ends(".jpeg")) return "image/jpeg";
    if (ends(".svg")) return "image/svg+xml";
    return "application/octet-stream";
  }

  void respond(Client& c, const std::string& status, const std::string& mime, const std::string& body) {
    std::ostringstream os;
    os << "HTTP/1.1 " << status << "\r\n"
       << "Content-Type: " << mime << "\r\n"
       << "Content-Length: " << body.size() << "\r\n"
       << "Cache-Control: no-cache\r\n"
       << "Connection: close\r\n\r\n" << body;
    sendRaw(c, os.str());
  }

  // ---------- WebSocket ----------
  void serveWs(Client& c) {
    while (true) {
      wsutil::Frame fr;
      size_t used = 0;
      bool err = false;
      if (!wsutil::tryDecodeFrame(c.buf, fr, used, &err)) {
        if (err) c.alive = false;   // 协议错误（长度回绕/超上限）→ 断连
        return;
      }
      c.buf.erase(c.buf.begin(), c.buf.begin() + long(used));

      // P1-1：客户端帧必须掩码（RFC6455 MUST），裸帧即协议错误
      if (!fr.masked) { c.alive = false; return; }
      bool isCtl = (fr.opcode & 0x8) != 0;
      // 保留 opcode 拒收
      if ((fr.opcode >= 0x3 && fr.opcode <= 0x7) || (fr.opcode >= 0xB && fr.opcode <= 0xF)) {
        c.alive = false;
        return;
      }
      // P1-2：控制帧 ≤125 且禁分片
      if (isCtl && (!fr.fin || fr.payload.size() > 125)) { c.alive = false; return; }
      // P1-3：孤立 Continuation 拒收；分片中途改开新消息拒收
      if (fr.opcode == wsutil::kCont && !c.fragOpen) { c.alive = false; return; }
      if (c.fragOpen && fr.opcode != wsutil::kCont && !isCtl) { c.alive = false; return; }

      // 分片重组（消息总长上限 kMaxMessage）
      std::string payload;
      uint8_t op = fr.opcode;
      if (op == wsutil::kCont) {
        if (c.frag.size() + fr.payload.size() > kMaxMessage) { c.alive = false; return; }
        c.frag += fr.payload;
        if (fr.fin) {
          payload = c.frag;
          op = c.fragOp;
          c.frag.clear();
          c.fragOpen = false;
        } else {
          continue;
        }
      } else if (!fr.fin && (op == wsutil::kText || op == wsutil::kBinary)) {
        if (fr.payload.size() > kMaxMessage) { c.alive = false; return; }
        c.frag = fr.payload;
        c.fragOpen = true;
        c.fragOp = op;
        continue;
      } else {
        payload = fr.payload;
      }

      switch (op) {
        case wsutil::kPing:
          sendRaw(c, wsutil::encodeFrame(wsutil::kPong, payload));
          break;
        case wsutil::kPong:
          break;
        case wsutil::kClose:
          sendRaw(c, wsutil::encodeFrame(wsutil::kClose, payload));
          c.alive = false;
          return;
        case wsutil::kText:
        case wsutil::kBinary: {
          json::Value v;
          if (json::Value::parse(payload, v) && v.isObject()) inbox_.push_back(std::move(v));
          break;
        }
        default:
          break;
      }
      if (!c.alive) return;
    }
  }

  int port_;
  std::string webRoot_;
  std::string rootReal_;
  std::vector<std::string> allowedOrigins_;
  int listenFd_ = -1;
  std::vector<Client> clients_;
  std::vector<json::Value> inbox_;
  std::map<std::string, std::string> staticCache_;   // path → 内容（C1 预读缓存）
  size_t staticCacheBytes_ = 0;
  size_t cacheHits_ = 0, cacheMisses_ = 0;
  size_t rejectedClients_ = 0;                        // C2：连接数超限被拒计数
  size_t headerRejects_ = 0;                          // C2：超长请求头丢弃计数
  std::function<void(json::Value&)> healthExtra_;     // /api/health 扩展字段
};

}  // namespace arm
