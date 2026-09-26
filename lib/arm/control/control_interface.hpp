// lib/arm/control/control_interface.hpp — Layer 0 抽象控制接口
// 统一基类：广播状态 / 接收指令 / 急停。WS 服务与串口驱动都是它的实现。
// 指令为弱类型 JSON 字典（arm::json::Value），由主循环解释执行。
#pragma once
#include "arm/kinematics.hpp"
#include "arm/control/json.hpp"
#include <array>
#include <string>
#include <vector>

namespace arm {

// 快照：广播给所有控制端的机器人状态（JSON 友好）
struct StateSnapshot {
  double t = 0;
  std::array<double, 6> q{}, qd{};
  Vec3 eePos;
  double roll = 0, pitch = 0, yaw = 0;
  double grip = 0;
  bool estop = false;
  double sigmaMin = 0;      // σ_min(J) 原始值
  double sigIdx = 0;        // 无量纲奇异指标 η=σ_min/σ_max ∈ (0,1]（前端画表）
  double manip = 0;         // 可操控度
  std::string mode;         // idle / ptp / cartesian / teach / grasp ...
  int teachCount = 0;

  json::Value toJson() const {
    json::Value v = json::Value::object();
    v.set("type", json::Value("state"));
    v.set("t", json::Value(t));
    v.set("mode", json::Value(mode));
    v.set("estop", json::Value(estop));
    v.set("grip", json::Value(grip));
    v.set("sigma_min", json::Value(sigmaMin));
    v.set("sig_idx", json::Value(sigIdx));
    v.set("manip", json::Value(manip));
    v.set("teach_count", json::Value(double(teachCount)));
    json::Value jq = json::Value::array();
    for (double x : q) jq.pushBack(json::Value(x));
    v.set("q", jq);
    json::Value jqd = json::Value::array();
    for (double x : qd) jqd.pushBack(json::Value(x));
    v.set("qd", jqd);
    json::Value jp = json::Value::array();
    jp.pushBack(json::Value(eePos.x)); jp.pushBack(json::Value(eePos.y)); jp.pushBack(json::Value(eePos.z));
    v.set("ee_pos", jp);
    json::Value jr = json::Value::array();
    jr.pushBack(json::Value(roll)); jr.pushBack(json::Value(pitch)); jr.pushBack(json::Value(yaw));
    v.set("ee_rpy", jr);
    return v;
  }
};

class ControlInterface {
 public:
  virtual ~ControlInterface() = default;
  virtual bool begin() = 0;
  // 广播一条状态（实现可自行决定节流）
  virtual void broadcastState(const StateSnapshot& st) = 0;
  // 取回自上次以来收到的全部指令
  virtual std::vector<json::Value> pollCommands() = 0;
  // 急停（实现应立即清空输出队列并通知端侧）
  virtual void emergencyStop() = 0;
  // 服务一拍（poll/select 等），timeoutMs 为允许阻塞的毫秒数
  virtual void service(int timeoutMs) { (void)timeoutMs; }
  // 推送一条任意消息（如 teach_export 的轨迹 JSON）
  virtual void sendToLast(const json::Value& msg) { (void)msg; }
};

// 软件限位 + 急停锁存：所有下行指令过这里
class SafetyGate {
 public:
  explicit SafetyGate(const ArmModel& arm) : arm_(arm) {}
  void eStop(bool on) { estop_ = on; }
  bool eStop() const { return estop_; }
  // 关节目标限幅
  std::array<double, 6> clampQ(const std::array<double, 6>& q) const {
    std::array<double, 6> r = q;
    for (int i = 0; i < 6; i++) r[i] = std::clamp(r[i], arm_.qmin[i], arm_.qmax[i]);
    return r;
  }
  std::array<double, 6> clampV(const std::array<double, 6>& v) const {
    std::array<double, 6> r = v;
    for (int i = 0; i < 6; i++) r[i] = std::clamp(r[i], -arm_.vmax[i], arm_.vmax[i]);
    return r;
  }

 private:
  const ArmModel& arm_;
  bool estop_ = false;
};

}  // namespace arm
