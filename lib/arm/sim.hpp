// lib/arm/sim.hpp — Layer 0 仿真：状态积分器 + RL 环境
// ArmSim：速度/位置两种指令语义、一阶伺服滞后、软件限位、急停减速、
//         关节摩擦（库仑+粘性+静摩擦，真实稳态跌落/静摩擦死区/低速爬行）、
//         负载重力补偿钩子（经 RobotConf）、夹爪开合；
//         安全监控钩子（待下发指令可读 + 状态强制回写，供 SafetyMonitor 独立终检）。
// RLEnv ：观测 18 = 末端位姿误差 6 + 关节角 6 + 关节角速度 6；
//         动作 6 = 关节速度指令（语义限幅）；确定性种子、episode 终止条件、
//         固定归一化（保证可复现）。
#pragma once
#include "arm/robot_conf.hpp"
#include "arm/kinematics.hpp"
#include <array>
#include <cmath>
#include <random>

namespace arm {

struct RobotState {
  std::array<double, 6> q{}, qd{}, tau_ff{};
  double t = 0;
  double grip = 0.0;        // 0 全开 .. 1 全闭
  bool estop = false;
};

class ArmSim {
 public:
  explicit ArmSim(const RobotConf& conf, double dt = 0.002)
      : conf_(conf), dt_(dt) {
    st_.q = conf.home;
  }

  // ---- 指令语义 ----
  void setJointVelocityTarget(const std::array<double, 6>& v) {
    vcmd_ = v;
    posMode_ = false;
  }
  void setJointPositionTarget(const std::array<double, 6>& q) {
    qtarget_ = q;
    posMode_ = true;
  }
  void setGripper(double g) { gripCmd_ = std::clamp(g, 0.0, 1.0); }
  void setEStop(bool on) {
    st_.estop = on;
    if (on) vcmd_.fill(0.0);
  }
  bool eStop() const { return st_.estop; }

  // ---- 积分一拍（速度阻尼器 + 伺服滞后）----
  void step() {
    const auto& arm = conf_.arm;
    std::array<double, 6> vwant{};
    if (st_.estop) {
      vwant.fill(0.0);                              // 急停：受控减速 → 保持
    } else if (posMode_) {
      for (int i = 0; i < 6; i++) {
        double err = qtarget_[i] - st_.q[i];
        // 速度阻尼器：|v| ≤ sqrt(2·a·|err|) 保证总能刹停（临界阻尼，无超调）
        double vlim = std::sqrt(2.0 * conf_.amax * std::abs(err));
        double v = std::min(std::abs(conf_.posKp * err), vlim);
        vwant[i] = (err >= 0 ? v : -v);
      }
    } else {
      vwant = vcmd_;
    }
    // 速度指令语义限幅（±vmax）→ 指令一阶滞后（伺服电气时间常数）→ 加速度限幅跟踪
    double alpha = dt_ / (conf_.servoTau + dt_);
    double dv = conf_.amax * dt_;
    for (int i = 0; i < 6; i++) {
      vwant[i] = std::clamp(vwant[i], -arm.vmax[i], arm.vmax[i]);
      vs_[i] += alpha * (vwant[i] - vs_[i]);
      // 关节摩擦（库仑+粘性+静摩擦）：τ_f = b·qd + fc·sgn(qd)，伺服刚度 Kv 折算速度跌落
      // vEff = vs − τ_f/Kv（速度跟踪真实稳态误差）；|τ_d|≤τs 且 qd≈0 → 粘滞锁定（静摩擦死区）。
      // fc 参数为 0 时退化为无摩擦伺服（RL 黄金基线路径显式置零，动力学逐位不变）。
      double v = st_.qd[i];
      const double kVEps = 1e-3;
      double tauF = conf_.fric.visc[i] * v;
      double sgn = std::abs(v) > kVEps ? (v > 0 ? 1.0 : -1.0)
                                       : std::tanh((vs_[i] - v) * conf_.velKv /
                                                   std::max(conf_.fric.coul[i], 1e-9));
      tauF += conf_.fric.coul[i] * sgn;
      double vEff = vs_[i] - tauF / std::max(conf_.velKv, 1e-9);
      double tauD = std::abs(vs_[i] - v) * conf_.velKv;
      if (std::abs(v) < 1e-4 && tauD <= conf_.fric.stic[i]) vEff = 0.0;   // 静摩擦粘滞
      st_.qd[i] += std::clamp(vEff - st_.qd[i], -dv, dv);
    }
    // 负载重力补偿钩子：速度前馈扰动（真机为力矩前馈，仿真以柔性扰动体现）
    st_.tau_ff = conf_.gravityCompTorque(st_.q);
    for (int i = 0; i < 6; i++) {
      double disturb = 1e-4 * st_.tau_ff[i];        // 柔性负载效应
      st_.q[i] += (st_.qd[i] + disturb) * dt_;
      // 软件限位：顶墙则速度清零
      if (st_.q[i] < arm.qmin[i]) { st_.q[i] = arm.qmin[i]; if (st_.qd[i] < 0) st_.qd[i] = 0; }
      if (st_.q[i] > arm.qmax[i]) { st_.q[i] = arm.qmax[i]; if (st_.qd[i] > 0) st_.qd[i] = 0; }
    }
    st_.grip += std::clamp(gripCmd_ - st_.grip, -conf_.gripMaxSpeed * dt_, conf_.gripMaxSpeed * dt_);
    st_.t += dt_;
  }

  const RobotState& state() const { return st_; }
  const RobotConf& conf() const { return conf_; }
  double dt() const { return dt_; }
  void reset(const std::array<double, 6>& q) {
    st_ = RobotState{};
    st_.q = q;
    vcmd_.fill(0.0);
    vs_.fill(0.0);
    qtarget_ = q;
    posMode_ = true;
    gripCmd_ = st_.grip;
  }

  Mat4 eePose() const { return forwardKinematicsT0_6(conf_.arm, st_.q) * conf_.tool; }

  // ---- 安全监控钩子（SafetyMonitor 独立终检用；不改变正常控制语义）----
  // 待下发指令只读访问：监控层据此在「下发前」过滤越限目标。
  const std::array<double, 6>& pendingQTarget() const { return qtarget_; }
  const std::array<double, 6>& pendingVTarget() const { return vcmd_; }
  bool positionMode() const { return posMode_; }
  // 故障注入（**仅用于安全监控层的故障测试**：故意绕过积分器限幅，模拟积分器/派发失效）
  void injectFaultState(const std::array<double, 6>& q, const std::array<double, 6>& qd) {
    st_.q = q;
    st_.qd = qd;
  }
  // 速度清零钩子：监控层发现观测速度越限时使用（不清位置）
  void safetyZeroVelocity() {
    st_.qd.fill(0.0);
    vs_.fill(0.0);
    vcmd_.fill(0.0);
  }
  // 状态强制回写：仅监控层使用（越限/跃变时把状态投影回安全域，并按需清零速度）
  void applySafetyClamp(const std::array<double, 6>& q, bool zeroVelOnClamp) {
    for (int i = 0; i < 6; i++) {
      double c = std::clamp(q[i], conf_.arm.qmin[i], conf_.arm.qmax[i]);
      if (zeroVelOnClamp && c != q[i]) {
        st_.q[i] = c;
        st_.qd[i] = 0.0;
        vs_[i] = 0.0;
      } else {
        st_.q[i] = c;
      }
    }
  }

 private:
  RobotConf conf_;
  double dt_;
  RobotState st_;
  std::array<double, 6> vcmd_{}, qtarget_{}, vs_{};   // vs_ 平滑后的指令速度
  bool posMode_ = true;
  double gripCmd_ = 0.0;
};

// ---------------- RL 环境（REINFORCE 第一刀） ----------------
// 任务：关节空间速度控制把末端带到目标位姿（目标经随机可达位形 FK 生成）。
// 观测 18 维固定归一化：pos err/0.25、rot err/π、q/qmax、qd/vmax。
struct RLObs {
  std::array<double, 18> x{};
};

class RLEnv {
 public:
  static constexpr int kObsDim = 18;
  static constexpr int kActDim = 6;
  static constexpr double kMaxT = 4.0;          // episode 时长上限 (s)
  static constexpr double kPosTol = 0.02;       // 成功位置容差 (m)
  static constexpr double kRotTol = 0.15;       // 成功姿态容差 (rad)

  // RL 基线动力学显式去摩擦（黄金回归逐位确定性）；仿真/控制路径用 desktop6 真实摩擦。
  static RobotConf rlBaseline(const RobotConf& c) {
    RobotConf r = c;
    r.fric = JointFriction{};
    return r;
  }
  RLEnv(const RobotConf& conf, double dt = 0.002, int actionRepeat = 5,
        double maxT = kMaxT)
      : sim_(rlBaseline(conf), dt), repeat_(actionRepeat), maxT_(maxT) {}

  void reset(uint64_t seed, const std::array<double, 6>* start = nullptr,
             const std::array<double, 6>* goalQ = nullptr) {
    rng_.seed(seed);
    std::uniform_real_distribution<double> U(-1.5, 1.5);
    std::array<double, 6> q0{}, qg{};
    if (start) q0 = *start; else for (auto& x : q0) x = U(rng_);
    if (goalQ) qg = *goalQ; else for (auto& x : qg) x = U(rng_);
    clampToLimits(q0); clampToLimits(qg);
    qGoal_ = qg;
    sim_.reset(q0);
    goal_ = forwardKinematicsT0_6(sim_.conf().arm, qg) * sim_.conf().tool;
    t_ = 0;
    steps_ = 0;
    success_ = false;
  }

  // 动作 = 关节速度指令，语义限幅到 ±vmax
  RLObs step(const std::array<double, kActDim>& action, double& reward, bool& done) {
    const auto& arm = sim_.conf().arm;
    std::array<double, 6> v{};
    for (int i = 0; i < 6; i++) v[i] = std::clamp(action[i], -1.0, 1.0) * arm.vmax[i];
    sim_.setJointVelocityTarget(v);
    for (int r = 0; r < repeat_; r++) sim_.step();
    t_ = double(steps_ + 1) * repeat_ * sim_.dt();
    steps_++;

    RLObs obs = observe();
    double posE, rotE;
    poseErr(posE, rotE);
    // 奖励：负位姿误差 + 动作代价 + 成功加成
    reward = -(2.0 * posE + 0.5 * rotE) - 0.002 * actionNorm2(action);
    done = false;
    if (posE < kPosTol && rotE < kRotTol) {
      reward += 10.0;
      done = true;
      success_ = true;
    } else if (t_ >= maxT_) {
      done = true;
    }
    return obs;
  }

  const std::array<double, 6>& goalConfig() const { return qGoal_; }

  RLObs observe() const {
    RLObs o;
    double posE, rotE;
    Vec3 dp; double err[6];
    poseErrVecs(err);
    const auto& arm = sim_.conf().arm;
    for (int i = 0; i < 3; i++) o.x[i] = err[i] / 0.25;
    for (int i = 0; i < 3; i++) o.x[3 + i] = err[3 + i] / kPi;
    for (int i = 0; i < 6; i++) o.x[6 + i] = sim_.state().q[i] / std::max(arm.qmax[i], 1e-6);
    for (int i = 0; i < 6; i++) o.x[12 + i] = sim_.state().qd[i] / std::max(arm.vmax[i], 1e-6);
    (void)posE; (void)rotE; (void)dp;
    return o;
  }

  void poseErr(double& posE, double& rotE) const {
    Vec3 e = poseErrorV(sim_.eePose(), goal_);
    posE = e.x; rotE = e.y;
  }
  bool success() const { return success_; }
  const Mat4& goal() const { return goal_; }
  ArmSim& sim() { return sim_; }

 private:
  void poseErrVecs(double e[6]) const { poseErrorVec(sim_.eePose(), goal_, e); }
  void clampToLimits(std::array<double, 6>& q) const {
    const auto& arm = sim_.conf().arm;
    for (int i = 0; i < 6; i++) q[i] = std::clamp(q[i], arm.qmin[i] + 0.05, arm.qmax[i] - 0.05);
  }
  static double actionNorm2(const std::array<double, kActDim>& a) {
    double s = 0;
    for (double x : a) s += x * x;
    return s;
  }

  ArmSim sim_;
  int repeat_;
  double maxT_;
  std::mt19937_64 rng_{0};
  Mat4 goal_ = Mat4::identity();
  std::array<double, 6> qGoal_{};
  double t_ = 0;
  long steps_ = 0;
  bool success_ = false;
};

}  // namespace arm
