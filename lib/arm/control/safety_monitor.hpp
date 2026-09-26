// lib/arm/control/safety_monitor.hpp — 独立安全监控层（B1，真机前第二道防线）
// 设计约束（与 SafetyGate 的关键区别）：
//   · 独立于轨迹规划与 IK：不依赖 TrajPlayer/planning/kinematics 的任何输出是否可信，
//     即使两者都出错（旧版 P0-1 时间基准失效、P0-补-2 越限解泄漏），也不会下发越限指令；
//   · 位于 Scheduler::tick 末端、**指令下发前**：preDispatch 过滤待下发目标（位置硬 clamp /
//     速度硬 clamp），postStep 审计积分后的状态（位置越限 → 投影回安全域；速度/加速度越限 →
//     计数；单拍位置跃变 → 回滚该拍 + 清速）；
//   · 越限行为显式且可测：clamp + 计数 +（可选）急停锁存，开关由 Config 控制（CLI 暴露）；
//   · 可观测：全部计数进 StateSnapshot.safety → 前端/测试/日志直接可见。
// header-only，仅依赖 robot_conf.hpp 与 sim.hpp 的公开接口。
#pragma once
#include "arm/sim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace arm {

// 越限计数（全流程可观测）
struct SafetyCounters {
  int posClamps = 0;      // 下发前：位置目标越限被夹回
  int velClamps = 0;      // 下发前：速度目标越限被夹回
  int posState = 0;       // 积分后：状态位置越限（积分器兜底失效）
  int velState = 0;       // 积分后：观测 |qd| > vmax
  int accState = 0;       // 积分后：观测 |Δqd|/dt > amax
  int stepJump = 0;       // 积分后：单拍位置跃变（时间基准失效类 bug 的特征）
  int estopTriggers = 0;  // 触发急停锁存次数
  int total() const { return posClamps + velClamps + posState + velState + accState + stepJump; }
};

class SafetyMonitor {
 public:
  struct Config {
    bool enabled = true;            // 总开关（CLI --no-safemon 关闭）
    bool estopOnViolation = false;  // 越限即急停锁存（CLI --safemon-estop 打开）
    double stepFactor = 1.5;        // 单拍位移上限 = vmax·dt·stepFactor（理论上限 1.0 的余量）
    double velTol = 1.001;          // 观测速度容差（浮点松弛）
    double accTol = 1.05;           // 观测加速度容差
  };

  explicit SafetyMonitor(const ArmModel& arm) : arm_(arm) {}
  SafetyMonitor(const ArmModel& arm, const Config& cfg) : arm_(arm), cfg_(cfg) {}

  bool enabled() const { return cfg_.enabled; }
  void setEnabled(bool on) { cfg_.enabled = on; }
  void setEstopOnViolation(bool on) { cfg_.estopOnViolation = on; }
  const Config& config() const { return cfg_; }
  const SafetyCounters& counters() const { return cnt_; }

  // 独立拦截入口（不经过任何规划器）：把越限目标夹回安全域；返回 true = 发生拦截。
  // 供 preDispatch 使用，也供真机派发层/测试直接调用。
  bool guardTarget(std::array<double, 6>& qcmd) {
    if (!cfg_.enabled) return false;
    bool clamped = false;
    for (int i = 0; i < 6; i++) {
      double lo = arm_.qmin[i], hi = arm_.qmax[i];
      if (qcmd[i] < lo) { qcmd[i] = lo; clamped = true; }
      else if (qcmd[i] > hi) { qcmd[i] = hi; clamped = true; }
      if (!std::isfinite(qcmd[i])) { qcmd[i] = 0.0; clamped = true; }   // NaN/Inf 兜底
    }
    if (clamped) cnt_.posClamps++;
    return clamped;
  }

  // 独立拦截入口（速度域）：|v| ≤ vmax；返回 true = 发生拦截
  bool guardVelocity(std::array<double, 6>& vcmd, const ArmModel& arm) {
    if (!cfg_.enabled) return false;
    bool clamped = false;
    for (int i = 0; i < 6; i++) {
      double vm = arm.vmax[i];
      if (!std::isfinite(vcmd[i])) { vcmd[i] = 0.0; clamped = true; continue; }
      if (vcmd[i] > vm) { vcmd[i] = vm; clamped = true; }
      else if (vcmd[i] < -vm) { vcmd[i] = -vm; clamped = true; }
    }
    if (clamped) cnt_.velClamps++;
    return clamped;
  }

  // 阶段①（下发前）：过滤待下发目标。返回 true = 请求急停。
  bool preDispatch(ArmSim& sim, double /*dt*/ = 0.0) {
    if (!cfg_.enabled) return false;
    bool estop = false;
    if (sim.positionMode()) {
      std::array<double, 6> tgt = sim.pendingQTarget();
      if (guardTarget(tgt)) {
        sim.setJointPositionTarget(tgt);   // 拦截：越限目标被夹回后才可能下发
        estop = cfg_.estopOnViolation;
      }
    } else {
      std::array<double, 6> v = sim.pendingVTarget();
      if (guardVelocity(v, sim.conf().arm)) {
        sim.setJointVelocityTarget(v);
        estop = cfg_.estopOnViolation;
      }
    }
    if (estop) latch(sim);
    return estop;
  }

  // 阶段②（积分后）：状态审计 + 投影。返回 true = 请求急停。
  bool postStep(ArmSim& sim, double dt) {
    if (!cfg_.enabled) return false;
    const auto& st = sim.state();
    const auto& arm = sim.conf().arm;
    bool estop = false;

    // (a) 位置限位：积分器本应保证；一旦越限说明兜底失效 → 投影回安全域 + 计数
    std::array<double, 6> q = st.q;
    bool posBad = false;
    for (int i = 0; i < 6; i++)
      if (!(q[i] >= arm.qmin[i] && q[i] <= arm.qmax[i])) posBad = true;
    if (posBad) {
      cnt_.posState++;
      sim.applySafetyClamp(q, true);
      estop = cfg_.estopOnViolation;
    }

    // (b) 单拍位置跃变：|Δq| > vmax·dt·factor ⇒ 时间基准/派发异常特征（P0-1 类 bug 兜底）
    if (havePrev_ && dt > 0) {
      bool jump = false;
      for (int i = 0; i < 6; i++) {
        double dq = std::abs(st.q[i] - qPrev_[i]);
        if (dq > arm.vmax[i] * dt * cfg_.stepFactor) jump = true;
      }
      if (jump) {
        cnt_.stepJump++;
        sim.applySafetyClamp(qPrev_, true);   // 回滚该拍：跃变不可能传到下发/广播层
        estop = cfg_.estopOnViolation;
      }
    }

    // (c) 观测速度/加速度
    const auto& st2 = sim.state();
    for (int i = 0; i < 6; i++)
      if (std::abs(st2.qd[i]) > arm.vmax[i] * cfg_.velTol) {
        cnt_.velState++;
        sim.safetyZeroVelocity();
        estop = cfg_.estopOnViolation;
        break;
      }
    if (havePrev_ && dt > 0) {
      for (int i = 0; i < 6; i++) {
        double a = std::abs(st2.qd[i] - qdPrev_[i]) / dt;
        if (a > sim.conf().amax * cfg_.accTol) {
          cnt_.accState++;
          estop = cfg_.estopOnViolation;
          break;
        }
      }
    }

    qPrev_ = sim.state().q;
    qdPrev_ = sim.state().qd;
    havePrev_ = true;
    if (estop) latch(sim);
    return estop;
  }

  void reset() {
    cnt_ = SafetyCounters{};
    havePrev_ = false;
  }

 private:
  void latch(ArmSim& sim) {
    if (sim.eStop()) return;
    sim.setEStop(true);
    cnt_.estopTriggers++;
  }

  const ArmModel& arm_;
  Config cfg_;
  SafetyCounters cnt_;
  std::array<double, 6> qPrev_{}, qdPrev_{};
  bool havePrev_ = false;
};

}  // namespace arm
