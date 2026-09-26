// app/main.cpp — 主程序：WebSocket 服务 + 仿真/调度主循环
//   ./arm_sim [--port 8080] [--web web] [--dt 0.002] [--seed 7]
//             [--serial /dev/ttyUSB0] [--demo N] [--selftest]
//
// 调度：固定步长仿真 + 轨迹播放（PTP / 笛卡尔直线 / 抓取编排 / 示教回放）
//       + 25~50Hz 状态广播 + 指令派发。安全：软件限位 + 急停锁存。
// 指令（WebSocket JSON）：
//   {"type":"joint_target","q":[6],"speed":0.3}     S 曲线 PTP
//   {"type":"ee_target","pos":[3],"rpy":[3],"speed"} IK 后关节 PTP
//   {"type":"ee_drag","pos":[3],"rpy":[3]}           拖拽流：增量 IK 位置跟随
//   {"type":"joint_vel","qd":[6]}                   直接关节速度指令
//   {"type":"grip","g":0..1}
//   {"type":"estop","on":true|false}
//   {"type":"reset"}
//   {"type":"teach_add"} / {"type":"teach_clear"} / {"type":"teach_play","speed"} / {"type":"teach_export"}
//   {"type":"grasp","pos":[3],"height":0.05,"approach":0.10,"grip_z":0.03}
#include "arm/kinematics.hpp"
#include "arm/planning.hpp"
#include "arm/robot_conf.hpp"
#include "arm/sim.hpp"
#include "arm/trajectory.hpp"
#include "arm/control/control_interface.hpp"
#include "arm/control/json.hpp"
#include "arm/control/ws_server.hpp"
#include "arm/control/serial_driver.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace arm;

namespace {

// ---------- 调度器：轨迹执行 + 模式 + 抓取状态机 ----------
class Scheduler {
 public:
  enum class Mode { Idle, Ptp, Cartesian, TeachPlay, Grasp };

  explicit Scheduler(const RobotConf& conf) : sim_(conf), conf_(conf), gate_(conf.arm) {}

  void tick(double dt) {
    (void)dt;  // 仿真步长由 sim 决定；调度按拍推进
    switch (mode_) {
      case Mode::Idle:
        break;
      case Mode::Ptp:
      case Mode::Cartesian:
      case Mode::TeachPlay: {
        playTraj();
        break;
      }
      case Mode::Grasp:
        tickGrasp();
        break;
    }
    sim_.step();
  }

  void command(const json::Value& c) {
    const std::string& type = c.get("type").asString();
    if (type == "estop") {
      bool on = c.get("on").asBool();
      gate_.eStop(on);
      sim_.setEStop(on);
      if (on) { traj_ = JointTrajectory{}; mode_ = Mode::Idle; graspPhase_ = -1; }
      return;
    }
    if (gate_.eStop()) return;  // 急停锁存：只接受解除指令

    if (type == "joint_target") {
      std::array<double, 6> q = readQ(c.get("q"));
      q = gate_.clampQ(q);
      double speed = c.get("speed").isNumber() ? c.get("speed").asNumber(0.3) : 0.3;
      planPtp(sim_.state().q, q, speed);
    } else if (type == "ee_target") {
      Mat4 T = readPose(c);
      std::array<double, 6> q;
      if (inverseKinematics(conf_.arm, T, sim_.state().q, q)) {
        double speed = c.get("speed").isNumber() ? c.get("speed").asNumber(0.3) : 0.3;
        planPtp(sim_.state().q, gate_.clampQ(q), speed);
      }
    } else if (type == "ee_drag") {
      Mat4 T = readPose(c);
      std::array<double, 6> q;
      if (inverseKinematics(conf_.arm, T, sim_.state().q, q)) {
        traj_ = JointTrajectory{};
        mode_ = Mode::Idle;
        sim_.setJointPositionTarget(gate_.clampQ(q));
      }
    } else if (type == "joint_vel") {
      std::array<double, 6> v = readQ(c.get("qd"));
      traj_ = JointTrajectory{};
      mode_ = Mode::Idle;
      sim_.setJointVelocityTarget(gate_.clampV(v));
    } else if (type == "grip") {
      sim_.setGripper(c.get("g").asNumber(0.0));
    } else if (type == "reset") {
      traj_ = JointTrajectory{};
      mode_ = Mode::Idle;
      graspPhase_ = -1;
      teach_.clear();
      gate_.eStop(false);
      sim_.setEStop(false);
      sim_.reset(conf_.home);
    } else if (type == "teach_add") {
      teach_.push_back(sim_.state().q);
    } else if (type == "teach_clear") {
      teach_.clear();
    } else if (type == "teach_play") {
      double speed = c.get("speed").isNumber() ? c.get("speed").asNumber(0.3) : 0.3;
      planTeachPlay(speed);
    } else if (type == "teach_export") {
      exportDemo();
    } else if (type == "grasp") {
      startGrasp(c);
    } else {
      // 未知指令显式报错（不再静默丢弃——客户端必须能感知拼写/协议错误）
      json::Value v = json::Value::object();
      v.set("type", json::Value("error"));
      v.set("reason", json::Value("unknown_cmd"));
      v.set("cmd", json::Value(type));
      if (io_) io_->sendToLast(v);
    }
  }

  void broadcastTo(ControlInterface& io) {
    StateSnapshot st;
    st.t = sim_.state().t;
    st.q = sim_.state().q;
    st.qd = sim_.state().qd;
    Mat4 T = forwardKinematicsT0_6(conf_.arm, st.q);
    st.eePos = T.translationV();
    toRPY(T, st.roll, st.pitch, st.yaw);
    st.grip = sim_.state().grip;
    st.estop = sim_.state().estop;
    st.sigmaMin = minSingularValue(conf_.arm, st.q);
    st.sigIdx = singularityIndex(conf_.arm, st.q);
    st.manip = manipulability(conf_.arm, st.q);
    st.mode = modeName();
    st.teachCount = int(teach_.size());
    io.broadcastState(st);
  }

  void setIo(ControlInterface* io) { io_ = io; }
  ArmSim& sim() { return sim_; }
  const std::vector<std::array<double, 6>>& teach() const { return teach_; }
  Mode mode() const { return mode_; }

 private:
  std::string modeName() const {
    switch (mode_) {
      case Mode::Ptp: return "ptp";
      case Mode::Cartesian: return "cartesian";
      case Mode::TeachPlay: return "teach";
      case Mode::Grasp: return "grasp";
      default: return "idle";
    }
  }

  static std::array<double, 6> readQ(const json::Value& arr) {
    std::array<double, 6> q{};
    for (int i = 0; i < 6; i++) q[i] = arr.numAt(i, 0.0);
    return q;
  }
  static Mat4 readPose(const json::Value& c) {
    Vec3 p{c.get("pos").numAt(0, 0), c.get("pos").numAt(1, 0), c.get("pos").numAt(2, 0)};
    double r = c.get("rpy").numAt(0, 0), y = c.get("rpy").numAt(1, 0), w = c.get("rpy").numAt(2, 0);
    return fromRPY(p, r, y, w);
  }

  void planPtp(const std::array<double, 6>& q0, const std::array<double, 6>& qf, double speed) {
    traj_ = jointPTP(conf_.arm, q0, qf, sim_.dt(), speed, conf_.amax, conf_.jmax);
    trajStartT_ = sim_.state().t;   // 轨迹时间原点（缺省 0 会让重置后 t>0 时首拍即判完成）
    playhead_ = 0;
    mode_ = traj_.empty() ? Mode::Idle : Mode::Ptp;
    sim_.setJointPositionTarget(q0);
  }

  void planTeachPlay(double speed) {
    if (teach_.empty()) return;
    JointTrajectory all;
    std::array<double, 6> q0 = sim_.state().q;
    double tOff = 0;
    auto append = [&](const JointTrajectory& seg) {
      for (size_t i = 0; i < seg.size(); i++) {
        all.ts.push_back(seg.ts[i] + tOff);
        all.qs.push_back(seg.qs[i]);
      }
      if (!seg.empty()) tOff += seg.ts.back();
    };
    for (auto& qf : teach_) {
      append(jointPTP(conf_.arm, q0, qf, sim_.dt(), speed, conf_.amax, conf_.jmax));
      q0 = qf;
    }
    traj_ = all;
    trajStartT_ = sim_.state().t;
    playhead_ = 0;
    mode_ = traj_.empty() ? Mode::Idle : Mode::TeachPlay;
  }

  void playTraj() {
    if (traj_.empty()) { mode_ = Mode::Idle; return; }
    double t = sim_.state().t - trajStartT_;
    // 找到当前时间对应的插值关节角
    size_t n = traj_.size();
    if (t >= traj_.ts.back()) {
      sim_.setJointPositionTarget(traj_.qs.back());
      traj_ = JointTrajectory{};
      mode_ = Mode::Idle;
      return;
    }
    while (playhead_ + 1 < n && traj_.ts[playhead_ + 1] < t) playhead_++;
    size_t i = playhead_;
    double t0 = traj_.ts[i], t1 = traj_.ts[std::min(i + 1, n - 1)];
    double a = (t1 > t0) ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
    std::array<double, 6> q;
    for (int k = 0; k < 6; k++) {
      const auto& qs = traj_.qs;
      q[k] = qs[i][k] + a * (qs[std::min(i + 1, n - 1)][k] - qs[i][k]);
    }
    sim_.setJointPositionTarget(gate_.clampQ(q));
  }

  void startGrasp(const json::Value& c) {
    Vec3 pos{c.get("pos").numAt(0, 0), c.get("pos").numAt(1, 0), c.get("pos").numAt(2, 0)};
    double height = c.has("height") ? c.get("height").asNumber(0.05) : 0.05;
    double approach = c.has("approach") ? c.get("approach").asNumber(0.10) : 0.10;
    double gripZ = c.has("grip_z") ? c.get("grip_z").asNumber(0.03) : 0.03;
    // 基准姿态：当前姿态（保持工具朝向）
    Mat4 T = forwardKinematicsT0_6(conf_.arm, sim_.state().q);
    GraspPlan gp = graspPlan(pos, height, approach, gripZ, T);
    graspStages_.clear();
    // 三段笛卡尔直线衔接
    std::array<double, 6> q = sim_.state().q;
    Mat4 cur = T;
    bool ok = true;
    for (auto& tgt : gp.targets) {
      JointTrajectory seg = cartesianLineTraj(conf_.arm, cur, tgt, q, sim_.dt(), 0.08);
      if (seg.empty()) { ok = false; break; }
      graspStages_.push_back(seg);
      q = seg.qs.back();
      cur = tgt;
    }
    if (!ok) return;
    graspStageIdx_ = 0;
    graspPhase_ = 0;   // 0 移动 approach，1 下降 grasp，2 闭爪等待，3 上提 leave
    traj_ = graspStages_[0];
    playhead_ = 0;
    mode_ = Mode::Grasp;
  }

  void tickGrasp() {
    if (graspPhase_ < 0) { mode_ = Mode::Idle; return; }
    bool moving = !traj_.empty();
    if (moving) playTraj();
    if (mode_ == Mode::Idle && moving) return;  // playTraj 刚结束本拍不再推进
    if (!moving) {
      // 阶段切换
      if (graspPhase_ == 0) {          // approach 到位 → 下降
        graspPhase_ = 1;
        traj_ = graspStages_[1];
        playhead_ = 0;
        mode_ = Mode::Grasp;
      } else if (graspPhase_ == 1) {   // 下降到位 → 闭爪
        graspPhase_ = 2;
        graspWaitT_ = sim_.state().t;
        sim_.setGripper(1.0);
      } else if (graspPhase_ == 2) {   // 等待闭合 → 上提
        if (sim_.state().t - graspWaitT_ > 0.6) {
          graspPhase_ = 3;
          traj_ = graspStages_[2];
          playhead_ = 0;
          mode_ = Mode::Grasp;
        }
      } else {                          // leave 到位
        graspPhase_ = -1;
        mode_ = Mode::Idle;
      }
    }
  }

  void exportDemo() {
    if (!io_) return;
    json::Value v = json::Value::object();
    v.set("type", json::Value("demo"));
    v.set("dt", json::Value(0.05));
    json::Value pts = json::Value::array();
    for (auto& q : teach_) {
      json::Value jq = json::Value::array();
      for (double x : q) jq.pushBack(json::Value(x));
      pts.pushBack(jq);
    }
    v.set("points", pts);
    io_->sendToLast(v);
  }

  ArmSim sim_;
  RobotConf conf_;
  SafetyGate gate_;
  ControlInterface* io_ = nullptr;
  Mode mode_ = Mode::Idle;
  JointTrajectory traj_;
  size_t playhead_ = 0;
  double trajStartT_ = 0;
  std::vector<std::array<double, 6>> teach_;
  // 抓取
  std::vector<JointTrajectory> graspStages_;
  int graspPhase_ = -1, graspStageIdx_ = 0;
  double graspWaitT_ = 0;
};

void printUsage() {
  std::printf(
      "用法: arm_sim [--port 8080] [--web web] [--dt 0.002] [--seed 7]\n"
      "              [--serial /dev/ttyUSB0] [--demo N] [--selftest]\n"
      "              [--allow-origin URL]...   WS Origin 白名单（可重复；缺省放行所有）\n");
}

int selftest() {
  // 快速自检：FK/IK 往返 + 解析/数值一致性
  RobotConf conf = RobotConf::desktop6();
  int fails = 0;
  std::array<double, 6> q{0.3, -1.0, 0.8, 0.5, -0.4, 1.1};
  Mat4 T = forwardKinematicsT0_6(conf.arm, q);
  std::array<double, 6> qa{}, qn{};
  bool ka = analyticIK(conf.arm, T, q, qa);
  bool kn = inverseKinematics(conf.arm, T, q, qn);
  Vec3 ea = poseErrorV(forwardKinematicsT0_6(conf.arm, qa), T);
  Vec3 en = poseErrorV(forwardKinematicsT0_6(conf.arm, qn), T);
  std::printf("[selftest] analytic: %d err=%.2e/%.2e | numeric: %d err=%.2e/%.2e\n",
              ka, ea.x, ea.y, kn, en.x, en.y);
  if (!ka || ea.x > 1e-6 || ea.y > 1e-6) fails++;
  if (!kn || en.x > 1e-6 || en.y > 1e-6) fails++;
  std::printf(fails ? "[selftest] FAIL (%d)\n" : "[selftest] PASS\n", fails);
  return fails ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  int port = 8080;
  std::string webRoot = "web";
  double dt = 0.002;
  uint64_t seed = 7;
  std::string serialDev;
  long demoSteps = -1;
  std::vector<std::string> allowOrigins;   // WS Origin 白名单（空 = 放行所有；--allow-origin 可重复）

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&](const char* what) -> const char* {
      if (i + 1 >= argc) { std::fprintf(stderr, "缺少参数 %s\n", what); std::exit(2); }
      return argv[++i];
    };
    if (a == "--port") port = std::atoi(next("--port"));
    else if (a == "--web") webRoot = next("--web");
    else if (a == "--dt") dt = std::atof(next("--dt"));
    else if (a == "--seed") seed = uint64_t(std::atoll(next("--seed")));
    else if (a == "--serial") serialDev = next("--serial");
    else if (a == "--demo") demoSteps = std::atol(next("--demo"));
    else if (a == "--allow-origin") allowOrigins.push_back(next("--allow-origin"));
    else if (a == "--selftest") return selftest();
    else if (a == "-h" || a == "--help") { printUsage(); return 0; }
    else { std::fprintf(stderr, "未知参数: %s\n", a.c_str()); printUsage(); return 2; }
  }
  (void)seed;  // 确定性：调度层无随机；RL 环境自带种子

  RobotConf conf = RobotConf::desktop6();
  Scheduler sched(conf);

  if (demoSteps >= 0) {
    // 无头演示：脚本化 PTP + 抓取，打印状态（冒烟）
    std::printf("[demo] headless %ld steps\n", demoSteps);
    json::Value c = json::Value::object();
    c.set("type", json::Value("joint_target"));
    json::Value q = json::Value::array();
    for (double x : {0.5, -0.8, 0.9, 0.2, 0.5, 0.0}) q.pushBack(json::Value(x));
    c.set("q", q);
    c.set("speed", json::Value(0.5));
    sched.command(c);
    for (long i = 0; i < demoSteps; i++) {
      sched.tick(dt);
      if (i % 100 == 0) {
        const auto& st = sched.sim().state();
        std::printf("t=%.2f q=[%.2f %.2f %.2f %.2f %.2f %.2f] mode-ok\n", st.t,
                    st.q[0], st.q[1], st.q[2], st.q[3], st.q[4], st.q[5]);
      }
    }
    const auto& st = sched.sim().state();
    std::array<double, 6> qf{0.5, -0.8, 0.9, 0.2, 0.5, 0.0};
    double dev = 0;
    for (int k = 0; k < 6; k++) dev = std::max(dev, std::abs(st.q[k] - qf[k]));
    std::printf("[demo] final |q-qf|_inf = %.4f\n", dev);
    return dev < 0.05 ? 0 : 1;
  }

  WsServer ws(port, webRoot);
  if (!allowOrigins.empty()) ws.setOriginAllowlist(std::move(allowOrigins));   // 空 = 放行所有
  if (!ws.begin()) {
    std::fprintf(stderr, "WS 服务启动失败 (port=%d)\n", port);
    return 1;
  }
  std::unique_ptr<SerialDriver> serial;
  if (!serialDev.empty()) {
    serial = std::make_unique<SerialDriver>(serialDev);
    if (!serial->begin()) {
      std::fprintf(stderr, "串口 %s 打开失败（纯仿真继续）\n", serialDev.c_str());
      serial.reset();
    }
  }
  sched.setIo(&ws);
  std::printf("Lyra arm_sim 就绪: http://0.0.0.0:%d/  (ws: /ws, web: %s)\n", port, webRoot.c_str());

  using clock = std::chrono::steady_clock;
  auto next = clock::now();
  const auto tick = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(dt));
  long frame = 0;
  const long bcastEvery = std::max(1L, long(0.02 / dt));  // 50Hz 广播

  while (true) {
    next += tick;
    ws.service(0);
    if (serial) serial->service(0);
    for (auto& c : ws.pollCommands()) {
      if (c.get("type").asString() == "estop" && c.get("on").asBool()) {
        ws.emergencyStop();
        if (serial) serial->emergencyStop();
      }
      sched.command(c);
      if (serial) serial->broadcastState(StateSnapshot{});  // 状态经主广播统一发
    }
    if (serial)
      for (auto& c : serial->pollCommands()) sched.command(c);

    sched.tick(dt);

    if (frame % bcastEvery == 0) {
      sched.broadcastTo(ws);
      if (serial) {
        StateSnapshot st;
        // 串口只关心 q/qd/grip，复用广播体
        sched.broadcastTo(*serial);
      }
    }
    frame++;
    std::this_thread::sleep_until(next);
  }
  return 0;
}
