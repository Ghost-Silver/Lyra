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
#include "arm/traj_player.hpp"
#include "arm/control/safety_monitor.hpp"
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

  explicit Scheduler(const RobotConf& conf)
      : sim_(conf), conf_(conf), gate_(conf.arm), safemon_(conf.arm) {}

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
    // ---- 安全监控层（B1）：独立终检，位于指令下发前后两端 ----
    // ① 下发前：过滤待下发目标（位置/速度硬 clamp；监控层不信任任何规划器输出）
    // ② 积分后：状态审计（位置越限→投影；单拍跃变→回滚；速度/加速度→计数）
    // ③ 越限即急停（可选开关）：锁存急停并终止轨迹
    if (safemon_.enabled()) {
      bool wantEstop = safemon_.preDispatch(sim_, sim_.dt());
      sim_.step();
      wantEstop = safemon_.postStep(sim_, sim_.dt()) || wantEstop;
      if (wantEstop && !gate_.eStop()) {
        gate_.eStop(true);
        sim_.setEStop(true);
        player_.clear();
        mode_ = Mode::Idle;
        graspPhase_ = -1;
      }
    } else {
      sim_.step();
    }
  }

  void command(const json::Value& c) {
    const std::string& type = c.get("type").asString();
    if (type == "estop") {
      bool on = c.get("on").asBool();
      gate_.eStop(on);
      sim_.setEStop(on);
      if (on) { player_.clear(); mode_ = Mode::Idle; graspPhase_ = -1; }
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
        player_.clear();
        mode_ = Mode::Idle;
        sim_.setJointPositionTarget(gate_.clampQ(q));
      }
    } else if (type == "joint_vel") {
      std::array<double, 6> v = readQ(c.get("qd"));
      player_.clear();
      mode_ = Mode::Idle;
      sim_.setJointVelocityTarget(gate_.clampV(v));
    } else if (type == "grip") {
      sim_.setGripper(c.get("g").asNumber(0.0));
    } else if (type == "reset") {
      player_.clear();
      safemon_.reset();
      mode_ = Mode::Idle;
      graspPhase_ = -1;
      teach_.clear();
      gate_.eStop(false);
      sim_.setEStop(false);
      sim_.reset(conf_.home);
    } else if (type == "teach_add") {
      // G1：示教点上限——WS 无鉴权，任何客户端都可反复调用；无上限会成为内存增长路径
      //      （每点 6×8 B；上限 4096 点 ≈ 196 KB）。超限时明确回错（不静默丢弃）。
      if (teach_.size() >= kMaxTeachPoints) {
        json::Value v = json::Value::object();
        v.set("type", json::Value("error"));
        v.set("reason", json::Value("teach_full"));
        v.set("limit", json::Value(double(kMaxTeachPoints)));
        if (io_) io_->sendToLast(v);
      } else {
        teach_.push_back(sim_.state().q);
      }
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
    const auto& sc = safemon_.counters();
    st.safTotal = sc.total();
    st.safPos = sc.posClamps + sc.posState;
    st.safVel = sc.velClamps + sc.velState;
    st.safAcc = sc.accState;
    st.safStep = sc.stepJump;
    st.safEstop = sc.estopTriggers;
    st.safEnabled = safemon_.enabled();
    io.broadcastState(st);
  }

  // 示教点上限（4096 点 × 6 关节 × 8 B ≈ 196 KB；超限回 error{teach_full}）
  static constexpr size_t kMaxTeachPoints = 4096;

  void setIo(ControlInterface* io) { io_ = io; }
  void setSafetyEnabled(bool on) { safemon_.setEnabled(on); }
  bool safetyEnabled() const { return safemon_.enabled(); }
  bool safetyEstopOnViolation() const { return safemon_.config().estopOnViolation; }
  void setSafetyEstopOnViolation(bool on) { safemon_.setEstopOnViolation(on); }
  const SafetyCounters& safetyCounters() const { return safemon_.counters(); }
  const TrajPlayer& player() const { return player_; }
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

  // 轨迹单一入口（B2）：时间原点只有这里能设；非法轨迹拒播并回包（不再静默跳终点）
  bool beginTraj(const JointTrajectory& t, Mode m) {
    std::string err;
    if (!player_.start(t, sim_.state().t, &err)) {
      mode_ = Mode::Idle;
      reportError("traj_rejected", err);
      return false;
    }
    mode_ = m;
    return true;
  }

  void reportError(const std::string& reason, const std::string& detail) {
    if (!io_) return;
    json::Value v = json::Value::object();
    v.set("type", json::Value("error"));
    v.set("reason", json::Value(reason));
    v.set("detail", json::Value(detail));
    io_->sendToLast(v);
  }

  void planPtp(const std::array<double, 6>& q0, const std::array<double, 6>& qf, double speed) {
    sim_.setJointPositionTarget(q0);
    auto t = jointPTP(conf_.arm, q0, qf, sim_.dt(), speed, conf_.amax, conf_.jmax);
    beginTraj(t, Mode::Ptp);
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
    beginTraj(all, Mode::TeachPlay);
  }

  // 播放：时间轴推进与插值全部委托 TrajPlayer（含校验/单调性/时间原点）
  void playTraj() {
    std::array<double, 6> q{};
    bool finished = false;
    if (!player_.step(sim_.state().t, q, finished)) { mode_ = Mode::Idle; return; }
    sim_.setJointPositionTarget(gate_.clampQ(q));
    if (finished) {
      player_.clear();
      mode_ = Mode::Idle;
    }
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
    if (!beginTraj(graspStages_[0], Mode::Grasp)) graspPhase_ = -1;
  }

  void tickGrasp() {
    if (graspPhase_ < 0) { mode_ = Mode::Idle; return; }
    bool moving = player_.playing();
    if (moving) playTraj();
    if (mode_ == Mode::Idle && moving) return;  // playTraj 刚结束本拍不再推进
    if (!moving) {
      // 阶段切换
      if (graspPhase_ == 0) {          // approach 到位 → 下降
        graspPhase_ = 1;
        if (!beginTraj(graspStages_[1], Mode::Grasp)) graspPhase_ = -1;
      } else if (graspPhase_ == 1) {   // 下降到位 → 闭爪
        graspPhase_ = 2;
        graspWaitT_ = sim_.state().t;
        sim_.setGripper(1.0);
      } else if (graspPhase_ == 2) {   // 等待闭合 → 上提
        if (sim_.state().t - graspWaitT_ > 0.6) {
          graspPhase_ = 3;
          if (!beginTraj(graspStages_[2], Mode::Grasp)) graspPhase_ = -1;
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
  TrajPlayer player_;          // 轨迹播放（时间原点唯一入口 + 校验）
  SafetyMonitor safemon_;      // 独立安全监控层（下发前过滤 + 积分后审计）
  std::vector<std::array<double, 6>> teach_;
  // 抓取
  std::vector<JointTrajectory> graspStages_;
  int graspPhase_ = -1, graspStageIdx_ = 0;
  double graspWaitT_ = 0;
};

void printUsage() {
  std::printf(
      "用法: arm_sim [--port 8080] [--web web] [--dt 0.002] [--seed 7]\n"
      "              [--serial /dev/ttyUSB0] [--serial-role master|pendant] [--demo N] [--selftest]\n"
      "              [--allow-origin URL]...   WS Origin 白名单（可重复；缺省放行所有）\n"
      "              [--no-safemon] [--safemon-estop]  独立安全监控层开关（默认启用）\n");
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

// Master 角色：把上层已派发的指令镜像为真机 CMD_* 帧（真机语义：本端是控制上位机）。
// Pendant 角色不转发（该角色下对端才是指令源）。
static void forwardToRobot(arm::SerialDriver& s, const arm::json::Value& c) {
  if (s.role() != arm::SerialDriver::Role::Master) return;
  const std::string t = c.get("type").asString();
  if (t == "joint_target") {
    std::array<double, 6> q{};
    auto a = c.get("q");
    for (int i = 0; i < 6; i++) q[size_t(i)] = a.numAt(size_t(i));
    s.sendCmdPos(q);
  } else if (t == "joint_vel") {
    std::array<double, 6> v{};
    auto a = c.get("qd");
    for (int i = 0; i < 6; i++) v[size_t(i)] = a.numAt(size_t(i));
    s.sendCmdVel(v);
  } else if (t == "grip") {
    s.sendCmdGrip(c.get("g").asNumber(0.0));
  } else if (t == "estop") {
    s.sendCmdEstop(c.get("on").asBool());
  }
  // 其余类型（reset/teach_*/grasp…）无对应下行帧：保持本地语义，不下发
}

int main(int argc, char** argv) {
  int port = 8080;
  std::string webRoot = "web";
  double dt = 0.002;
  uint64_t seed = 7;
  std::string serialDev;
  std::string serialRole = "master";   // --serial-role master|pendant（真机语义 / 示教器语义）
  long demoSteps = -1;
  std::vector<std::string> allowOrigins;   // WS Origin 白名单（空 = 放行所有；--allow-origin 可重复）
  bool safemonOff = false;                 // --no-safemon：关闭独立安全监控层（默认启用）
  bool safemonEstop = false;               // --safemon-estop：越限即急停锁存

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
    else if (a == "--serial-role") serialRole = next("--serial-role");
    else if (a == "--demo") demoSteps = std::atol(next("--demo"));
    else if (a == "--allow-origin") allowOrigins.push_back(next("--allow-origin"));
    else if (a == "--no-safemon") safemonOff = true;
    else if (a == "--safemon-estop") safemonEstop = true;
    else if (a == "--selftest") return selftest();
    else if (a == "-h" || a == "--help") { printUsage(); return 0; }
    else { std::fprintf(stderr, "未知参数: %s\n", a.c_str()); printUsage(); return 2; }
  }
  (void)seed;  // 确定性：调度层无随机；RL 环境自带种子

  RobotConf conf = RobotConf::desktop6();
  Scheduler sched(conf);
  if (safemonOff) sched.setSafetyEnabled(false);
  if (safemonEstop) sched.setSafetyEstopOnViolation(true);

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
  // 串口（真机路径）先建立，便于 /api/health 暴露其在线/队列/丢弃计数
  std::unique_ptr<SerialDriver> serial;
  if (!serialDev.empty()) {
    const SerialDriver::Role role = (serialRole == "pendant") ? SerialDriver::Role::Pendant
                                                              : SerialDriver::Role::Master;
    serial = std::make_unique<SerialDriver>(serialDev, 115200, role);
    if (!serial->begin()) {
      std::fprintf(stderr, "串口 %s 打开失败（纯仿真继续）\n", serialDev.c_str());
      serial.reset();
    } else {
      std::printf("[serial] %s 已连接 role=%s（%s）；待发队列上限 %zu B，丢弃策略=丢最旧+计数\n",
                  serialDev.c_str(), serialRole.c_str(),
                  role == SerialDriver::Role::Master ? "真机语义：发 CMD_* / 收 STATE_REP"
                                                     : "示教器语义：推 STATE_REP / 收 CMD_*",
                  SerialDriver::kDefaultTxLimit);
    }
  }

  // /api/health 注入安全层计数（运维可直接 curl 监控，无需连 WS）
  ws.setHealthExtra([&sched, &serial, &serialRole](json::Value& v) {
    const auto& sc = sched.safetyCounters();
    v.set("safety_enabled", json::Value(sched.safetyEnabled()));
    v.set("safety_total", json::Value(double(sc.total())));
    v.set("safety_pos", json::Value(double(sc.posClamps + sc.posState)));
    v.set("safety_vel", json::Value(double(sc.velClamps + sc.velState)));
    v.set("safety_acc", json::Value(double(sc.accState)));
    v.set("safety_step", json::Value(double(sc.stepJump)));
    v.set("safety_estop", json::Value(double(sc.estopTriggers)));
    // 串口真机路径可观测性（E 批）：在线状态 / 待发队列 / 丢弃与错误计数
    v.set("serial_attached", json::Value(serial != nullptr));
    if (serial) {
      const auto& ts = serial->txStats();
      v.set("serial_online", json::Value(serial->online()));
      v.set("serial_role", json::Value(serialRole));
      v.set("serial_tx_queued_bytes", json::Value(double(ts.queuedBytes)));
      v.set("serial_tx_queued_frames", json::Value(double(ts.queuedFrames)));
      v.set("serial_tx_written", json::Value(double(ts.written)));
      v.set("serial_tx_dropped_full", json::Value(double(ts.droppedFull)));
      v.set("serial_tx_dropped_offline", json::Value(double(ts.droppedOffline)));
      v.set("serial_tx_retries", json::Value(double(ts.retries)));
      v.set("serial_io_errors", json::Value(double(ts.ioErrors)));
      v.set("serial_rx_overflow", json::Value(double(ts.rxOverflow)));
      v.set("serial_state_frames", json::Value(double(serial->remoteState().frames)));
    }
    v.set("teach_points", json::Value(double(sched.teach().size())));   // G1：示教点水位
    v.set("teach_limit", json::Value(double(Scheduler::kMaxTeachPoints)));
  });
  if (!ws.begin()) {
    std::fprintf(stderr, "WS 服务启动失败 (port=%d)\n", port);
    return 1;
  }
  sched.setIo(&ws);
  std::printf("Lyra arm_sim 就绪: http://0.0.0.0:%d/  (ws: /ws, web: %s)\n", port, webRoot.c_str());
  std::printf("[safety] 独立监控层 %s%s（下发前过滤 + 积分后审计；状态 JSON 含 safety 计数）\n",
              sched.safetyEnabled() ? "启用" : "关闭",
              sched.safetyEstopOnViolation() ? " + 越限急停" : "");

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
      if (serial) forwardToRobot(*serial, c);   // Master：指令 → CMD_* 帧（真机语义）
    }
    if (serial) {
      if (serial->role() == SerialDriver::Role::Pendant) {
        // 示教器语义：对端是指令源 → 其 CMD_* 进主调度
        for (auto& c : serial->pollCommands()) sched.command(c);
      } else {
        // 真机语义：只解析对端 STATE_REP（经 health 暴露）；不产生 CMD 指令
        (void)serial->pollCommands();
      }
    }

    sched.tick(dt);

    if (frame % bcastEvery == 0) {
      sched.broadcastTo(ws);
      // 仅示教器语义才周期下行 STATE_REP；真机语义（Master）不向机器人推状态
      if (serial && serial->role() == SerialDriver::Role::Pendant) sched.broadcastTo(*serial);
    }
    frame++;
    std::this_thread::sleep_until(next);
  }
  return 0;
}
