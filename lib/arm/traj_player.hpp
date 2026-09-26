// lib/arm/traj_player.hpp — 轨迹播放器：时间轴推进 + 防御性校验（B2）
// 单一入口承载「轨迹时间原点」：任何新轨迹都必须经 start(traj, now) 载入，
// 杜绝「漏设起点 → 用绝对时间参与完成判定 → 首个控制周期即跳终点」这类 bug
// （历史 P0-1 的失效模式）。载入前强制校验：ts 非空、与 qs 等长、**严格单调**、
// 末值 > 0、全为有限值；非法轨迹一律拒播并给出可读原因，绝不静默跳到终点。
// header-only，无外部依赖（仅 planning.hpp 的 JointTrajectory）。
#pragma once
#include "arm/planning.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace arm {

class TrajPlayer {
 public:
  // 校验：返回空串 = 合法；否则为拒播原因
  static std::string validate(const JointTrajectory& t) {
    if (t.ts.empty() || t.qs.empty()) return "空轨迹（ts/qs 未写入）";
    if (t.ts.size() != t.qs.size()) return "ts/qs 长度不一致";
    for (size_t i = 0; i < t.ts.size(); i++)
      if (!std::isfinite(t.ts[i])) return "ts 含非有限值";
    if (t.ts.size() == 1) {
      if (t.ts[0] < 0.0) return "单点轨迹 ts < 0";
      return "";
    }
    if (!(t.ts.back() > 0.0)) return "ts 末值 <= 0（时间轴未写入 → 会首拍判完成）";
    for (size_t i = 1; i < t.ts.size(); i++)
      if (!(t.ts[i] > t.ts[i - 1])) return "ts 非严格单调（@i=" + std::to_string(i) + "）";
    return "";
  }

  // 载入轨迹并以 now 为时间原点开始播放；非法 → false + err（计入 rejects）
  bool start(const JointTrajectory& t, double now, std::string* err = nullptr) {
    std::string e = validate(t);
    if (!e.empty()) {
      if (err) *err = e;
      lastError_ = e;
      rejects_++;
      playing_ = false;
      return false;
    }
    traj_ = t;
    t0_ = now;
    playhead_ = 0;
    playing_ = true;
    lastError_.clear();
    samples_++;
    return true;
  }

  void clear() {
    playing_ = false;
    traj_ = JointTrajectory{};
    playhead_ = 0;
  }

  // 推进到绝对时间 now：返回本拍应下发的关节角；finished = 本拍到达终点
  bool step(double now, std::array<double, 6>& q, bool& finished) {
    finished = false;
    if (!playing_) return false;
    size_t n = traj_.size();
    if (n == 0) { playing_ = false; return false; }
    double t = now - t0_;
    if (n == 1 || t >= traj_.ts.back()) {
      q = traj_.qs.back();
      finished = true;
      playing_ = false;
      return true;
    }
    if (t <= traj_.ts[0]) { q = traj_.qs[0]; return true; }
    while (playhead_ + 1 < n && traj_.ts[playhead_ + 1] < t) playhead_++;
    size_t i = std::min(playhead_, n - 2);
    size_t j = i + 1;
    double t0 = traj_.ts[i], t1 = traj_.ts[j];
    double a = (t1 > t0) ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
    for (int k = 0; k < 6; k++) q[k] = traj_.qs[i][k] + a * (traj_.qs[j][k] - traj_.qs[i][k]);
    return true;
  }

  bool playing() const { return playing_; }
  double t0() const { return t0_; }
  int rejects() const { return rejects_; }
  size_t samples() const { return samples_; }
  const std::string& lastError() const { return lastError_; }
  const JointTrajectory& traj() const { return traj_; }
  size_t playhead() const { return playhead_; }

 private:
  JointTrajectory traj_;
  double t0_ = 0.0;          // 时间原点（唯一入口：start()）
  size_t playhead_ = 0;
  bool playing_ = false;
  std::string lastError_;
  int rejects_ = 0;          // 拒播次数（可观测）
  size_t samples_ = 0;       // 成功载入次数
};

}  // namespace arm
