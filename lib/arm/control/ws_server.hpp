// lib/arm/control/ws_server.hpp — WebSocket 服务实装 (RFC6455) + 静态 HTTP
// 自包含 socket / SHA-1 / Base64 / JSON：
//   - HTTP GET 静态文件（web/ 目录，防路径穿越，常见 MIME）
//   - GET /ws 升级为 WebSocket（握手 SHA-1+Base64、帧编解码、ping/pong、分片、close）
//   - GET /api/health 健康检查
//   - 广播机器人状态 JSON / 接收指令 JSON；Origin 全放行（实验室工具 + 预览代理兼容）
// POSIX 实现，单线程 poll() 驱动，由主循环 service() 推进。
#pragma once
#include "arm/control/control_interface.hpp"
#include "arm/control/json.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <fstream>
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

// 解析一帧；返回 true 表示消费了一帧（consumed = 帧字节数）
struct Frame {
  bool fin = true;
  uint8_t opcode = 0;
  std::string payload;
};
inline bool tryDecodeFrame(const std::vector<uint8_t>& buf, Frame& out, size_t& consumed) {
  if (buf.size() < 2) return false;
  uint8_t b0 = buf[0], b1 = buf[1];
  out.fin = (b0 & 0x80) != 0;
  out.opcode = b0 & 0x0F;
  bool masked = (b1 & 0x80) != 0;
  uint64_t len = b1 & 0x7F;
  size_t pos = 2;
  if (len == 126) {
    if (buf.size() < pos + 2) return false;
    len = (uint64_t(buf[pos]) << 8) | buf[pos + 1];
    pos += 2;
  } else if (len == 127) {
    if (buf.size() < pos + 8) return false;
    len = 0;
    for (int i = 0; i < 8; i++) len = (len << 8) | buf[pos + i];
    pos += 8;
  }
  uint8_t mask[4] = {0, 0, 0, 0};
  if (masked) {
    if (buf.size() < pos + 4) return false;
    for (int i = 0; i < 4; i++) mask[i] = buf[pos + i];
    pos += 4;
  }
  if (buf.size() < pos + len) return false;
  out.payload.resize(size_t(len));
  for (uint64_t i = 0; i < len; i++) {
    uint8_t c = buf[pos + i];
    out.payload[size_t(i)] = char(masked ? (c ^ mask[i % 4]) : c);
  }
  consumed = pos + size_t(len);
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
    return true;
  }

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

  size_t wsClientCount() const {
    size_t n = 0;
    for (auto& c : clients_) if (c.alive && c.isWs) n++;
    return n;
  }

 private:
  struct Client {
    int fd = -1;
    bool isWs = false;
    bool alive = true;
    std::vector<uint8_t> buf;      // 未消化字节
    std::string frag;              // 分片续帧缓存
    bool fragOpen = false;
    uint8_t fragOp = 0;
  };

  static void setNonBlock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
  }

  void acceptAll() {
    while (true) {
      int fd = ::accept(listenFd_, nullptr, nullptr);
      if (fd < 0) break;
      setNonBlock(fd);
      int one = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
      Client c;
      c.fd = fd;
      clients_.push_back(c);
    }
  }

  static void sendRaw(Client& c, const std::string& bytes) {
    size_t off = 0;
    while (off < bytes.size()) {
      ssize_t n = ::send(c.fd, bytes.data() + off, bytes.size() - off, MSG_NOSIGNAL);
      if (n <= 0) { c.alive = false; return; }
      off += size_t(n);
    }
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

    // WS 升级
    auto up = hdr.find("upgrade");
    if (up != hdr.end()) {
      for (auto& ch : up->second) ch = char(::tolower(ch));
      if (up->second == "websocket") {
        auto key = hdr.find("sec-websocket-key");
        if (key == hdr.end()) { respond(c, "400 Bad Request", "text/plain", "missing key"); c.alive = false; return; }
        std::string resp =
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: " + wsutil::acceptKey(key->second) + "\r\n\r\n";
        sendRaw(c, resp);
        c.isWs = true;
        return;
      }
    }

    if (path == "/api/health") {
      json::Value v = json::Value::object();
      v.set("ok", json::Value(true));
      v.set("ws_clients", json::Value(double(wsClientCount())));
      respond(c, "200 OK", "application/json", v.dump());
      return;
    }

    // 静态文件
    std::string rel = (path == "/" || path.empty()) ? "/index.html" : path;
    size_t q = rel.find('?');
    if (q != std::string::npos) rel = rel.substr(0, q);
    if (rel.find("..") != std::string::npos) { respond(c, "403 Forbidden", "text/plain", "no"); return; }
    std::string file = webRoot_ + rel;
    std::ifstream f(file, std::ios::binary);
    if (!f) { respond(c, "404 Not Found", "text/plain", "404 " + rel); return; }
    std::ostringstream ss;
    ss << f.rdbuf();
    respond(c, "200 OK", mimeOf(rel), ss.str());
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
      if (!wsutil::tryDecodeFrame(c.buf, fr, used)) return;
      c.buf.erase(c.buf.begin(), c.buf.begin() + long(used));

      // 分片重组
      std::string payload;
      uint8_t op = fr.opcode;
      if (op == wsutil::kCont) {
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
  int listenFd_ = -1;
  std::vector<Client> clients_;
  std::vector<json::Value> inbox_;
};

}  // namespace arm
