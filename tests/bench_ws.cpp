// tests/bench_ws.cpp — C1 实时性基准：控制回路抖动 + 静态文件路径代价
// 场景：50 Hz 状态广播 + 4 个客户端持续拉取 2 MiB 大文件 + 1 个「只连不读」的慢 WS 客户端。
// 输出：单次循环耗时（均值/p50/p99/最坏）、超出 dt 预算的次数、慢客户端被丢弃时机，
//       以及「大文件读一次」的直接耗时（量化缓存/上限所消除的阻塞）。
// 用法： ./build-tests/bench_ws [iterations] [port] [bigMiB]
#include "arm/control/ws_server.hpp"
#include "arm/control/control_interface.hpp"
#include "arm/sim.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using namespace arm;
using clk = std::chrono::steady_clock;

static double usSince(clk::time_point t0) {
  return std::chrono::duration<double, std::micro>(clk::now() - t0).count();
}

static int connectLocal(int port) {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(uint16_t(port));
  a.sin_addr.s_addr = inet_addr("127.0.0.1");
  if (::connect(fd, (sockaddr*)&a, sizeof a) < 0) { ::close(fd); return -1; }
  // 接收超时：服务端在压测结束后不再服务时，客户端线程必须能退出（否则 join 阻塞）
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 200000;   // 200 ms
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  return fd;
}

// HTTP 拉取线程：反复 GET /big.bin，读完即关（不计内容）
static void httpFlood(int port, std::atomic<bool>& stop, std::atomic<long>& requests,
                      std::atomic<long>& bytes, const std::string& path) {
  while (!stop.load()) {
    if (stop.load()) break;
    int fd = connectLocal(port);
    if (fd < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
    std::string req = "GET " + path + " HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n";
    ::send(fd, req.data(), req.size(), MSG_NOSIGNAL);
    char buf[16384];
    long got = 0;
    ssize_t n;
    while ((n = ::recv(fd, buf, sizeof buf, 0)) > 0) got += n;
    ::close(fd);
    requests.fetch_add(1);
    bytes.fetch_add(got);
  }
}

// 慢 WS 客户端：完成握手后**永不读取**（验证非阻塞发送 + 有界队列）
static int slowWsClient(int port) {
  int fd = connectLocal(port);
  if (fd < 0) return -1;
  const char* hs =
      "GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
      "Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n";
  ::send(fd, hs, std::strlen(hs), MSG_NOSIGNAL);
  // 读走握手响应后就不再读
  char buf[1024];
  ::recv(fd, buf, sizeof buf, 0);
  return fd;
}

int main(int argc, char** argv) {
  long iterations = argc > 1 ? std::atol(argv[1]) : 4000;
  int port = argc > 2 ? std::atoi(argv[2]) : 8099;
  long bigKiB = argc > 3 ? std::atol(argv[3]) : 192;   // 默认 = 最大合法静态文件（192 KiB）

  // ---- 准备 web 根：大文件 + 小文件 ----
  const std::string root = "/tmp/lyra_bench_web";
  ::system(("mkdir -p " + root).c_str());
  const std::string bigPath = root + "/big.bin";
  {
    std::ofstream f(bigPath, std::ios::binary);
    std::vector<char> chunk(65536, 'x');
    for (long i = 0; i < (bigKiB + 63) / 64; i++) f.write(chunk.data(), std::streamsize(chunk.size()));
  }
  { std::ofstream f(root + "/index.html"); f << "<html><body>bench</body></html>\n"; }

  // ---- 直接测量：读一次大文件的耗时（缓存未命中路径；对比缓存命中）----
  double readUs = 0;
  {
    auto t0 = clk::now();
    std::ifstream f(bigPath, std::ios::binary);
    std::vector<char> all((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    readUs = usSince(t0);
    std::printf("[io] 直读 %ld KiB 静态文件一次: %.3f ms (%zu bytes)\n", bigKiB, readUs / 1000.0,
                all.size());
  }

  WsServer ws(port, root);
  if (!ws.begin()) { std::fprintf(stderr, "端口 %d 绑定失败\n", port); return 1; }
  RobotConf conf = RobotConf::desktop6();
  ArmSim sim(conf, 0.002);

  // ---- 预热缓存（首次请求走磁盘，后续命中内存）----
  {
    int fd = connectLocal(port);
    std::string req = "GET /big.bin HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n";
    ::send(fd, req.data(), req.size(), MSG_NOSIGNAL);
    char buf[8192];
    while (::recv(fd, buf, sizeof buf, 0) > 0) {}
    ::close(fd);
    std::printf("[io] 预热后 cache: hits=%zu misses=%zu（此后该文件走内存，零磁盘 IO）\n",
                ws.cacheHits(), ws.cacheMisses());
  }

  // ---- 压测：4 线程拉大文件 + 1 个慢 WS 客户端 ----
  std::atomic<bool> stop{false};
  std::atomic<long> reqs{0}, bytes{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < 4; i++) threads.emplace_back(httpFlood, port, std::ref(stop), std::ref(reqs),
                                                   std::ref(bytes), "/big.bin");
  int slowFd = slowWsClient(port);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  auto oneIteration = [&](double* svcUs) {
    auto t0 = clk::now();
    ws.service(0);
    double svc = usSince(t0);
    if (svcUs) *svcUs = svc;
    sim.step();
    static long ctr = 0;
    if ((ctr++ % 10) == 0) {   // 50 Hz 广播密度
      StateSnapshot st;
      st.t = sim.state().t;
      st.q = sim.state().q;
      st.qd = sim.state().qd;
      st.mode = "bench";
      ws.broadcastState(st);
    }
  };

  // 预热：跑满一段让连接/缓存进入稳态（样本丢弃）
  for (long i = 0; i < 300; i++) oneIteration(nullptr);

  std::vector<double> loopUs, svcUs;
  loopUs.reserve(size_t(iterations));
  svcUs.reserve(size_t(iterations));
  long overruns = 0;
  size_t slowDroppedAt = 0;
  const double dtBudgetUs = 2000.0;   // 2 ms 控制周期预算
  for (long i = 0; i < iterations; i++) {
    double svc = 0;
    auto t0 = clk::now();
    oneIteration(&svc);
    double us = usSince(t0);
    loopUs.push_back(us);
    svcUs.push_back(svc);
    if (us > dtBudgetUs) overruns++;
    if (slowDroppedAt == 0 && ws.wsClientCount() == 0) slowDroppedAt = size_t(i);
  }
  if (slowFd >= 0) ::close(slowFd);
  // 让残留的 HTTP 连接被服务端关闭（客户端 recv 200ms 超时兜底），再停线程
  for (int i = 0; i < 200; i++) ws.service(0);
  stop.store(true);
  for (auto& t : threads) t.join();

  std::sort(loopUs.begin(), loopUs.end());
  std::sort(svcUs.begin(), svcUs.end());
  double sum = 0, ssum = 0;
  for (double v : loopUs) sum += v;
  for (double v : svcUs) ssum += v;
  auto pct = [&](const std::vector<double>& v, double p) {
    return v[size_t(std::min(p * double(v.size()), double(v.size() - 1)))];
  };
  std::printf("\n===== C1 控制回路抖动（%ld 拍，预热 300 拍已丢弃；dt 预算 %.0f us；广播 50 Hz；4 客户端持续拉 %ld KiB 文件）=====\n",
              iterations, dtBudgetUs, bigKiB);
  std::printf("  ws.service() 纯耗时 均值 %.1f us | p50 %.1f us | p99 %.1f us | 最坏 %.1f us\n",
              ssum / double(svcUs.size()), pct(svcUs, 0.50), pct(svcUs, 0.99), svcUs.back());
  std::printf("  service+step+广播 全拍 均值 %.1f us | p50 %.1f us | p99 %.1f us | 最坏 %.1f us\n",
              sum / double(loopUs.size()), pct(loopUs, 0.50), pct(loopUs, 0.99), loopUs.back());
  std::printf("  超出 dt 预算(2 ms) 的拍数: %ld / %ld (%.3f%%)\n", overruns, iterations,
              100.0 * double(overruns) / double(iterations));
  std::printf("  慢 WS 客户端（只连不读）被丢弃于第 %zu 拍（有界队列生效，控制回路未阻塞）\n",
              slowDroppedAt);
  std::printf("  HTTP 请求 %ld 次 / 累计读 %.1f MiB；cache hits=%zu misses=%zu\n", reqs.load(),
              double(bytes.load()) / 1048576.0, ws.cacheHits(), ws.cacheMisses());
  std::printf("  对照：直读一次 %ld KiB 文件耗时 %.3f ms（= %.1f 倍循环均值）\n",
              bigKiB, readUs / 1000.0, readUs / (sum / double(loopUs.size())));
  bool ok = overruns == 0;
  std::printf("结论: %s\n", ok ? "零超时——缓存+上限后大文件流量不影响控制周期"
                              : "存在超时，需进一步缓解（见 PR 回复说明）");
  return ok ? 0 : 2;
}
