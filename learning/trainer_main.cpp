// learning/trainer_main.cpp — REINFORCE 训练主循环（环境 = Layer 0 仿真 RLEnv）
//   ./arm_train [--episodes 120] [--hidden 64] [--lr 3e-3] [--gamma 0.98]
//               [--seed 1] [--entropy 0.01] [--demo 8] [--imitate teach.json]
//               [--out learning/checkpoints] [--log learning/logs/train.csv]
// 输出：训练曲线 CSV + 策略 checkpoint（float 拍平 + JSON 元数据）。
#include "policy.hpp"
#include "reinforce.hpp"
#include "arm/robot_conf.hpp"
#include "arm/kinematics.hpp"
#include "arm/control/json.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <algorithm>

using namespace arm;
using namespace learn;

namespace {

// 合成示教专家：默认关节空间 P 控制器（goal 构型在 episode 内固定，
// 对学习是"示教板手柄轨迹"的等价替代）；--expert dls 切换 obs-严格 DLS 版。
enum class ExpertKind { JointP, DLS };
ExpertKind g_expert = ExpertKind::JointP;

std::array<double, kAct> expertAction(RLEnv& env) {
  if (g_expert == ExpertKind::JointP) {
    std::array<double, kAct> a{};
    const auto& qg = env.goalConfig();
    const auto& q = env.sim().state().q;
    for (int i = 0; i < kAct; i++) a[i] = std::clamp((qg[i] - q[i]) * 2.5, -1.0, 1.0);
    return a;
  }
  // ---- DLS 阻尼最小二乘：笛卡尔位姿误差 → 关节速度（obs 严格可实现）----
  arm::RLObs o = env.observe();
  const auto& q = env.sim().state().q;
  double J[6][6];
  buildJacobian(env.sim().conf().arm, q, J);
  // 期望末端速度 vw = Kp · 位姿误差（反归一：pos·0.25、rot·π）
  double vw[6];
  for (int i = 0; i < 3; i++) vw[i] = std::clamp(o.x[i] * 0.25 * 2.0, -0.6, 0.6);
  for (int i = 0; i < 3; i++) vw[3 + i] = std::clamp(o.x[3 + i] * M_PI * 1.0, -0.8, 0.8);
  // DLS: qd = Jᵀ (J Jᵀ + λ²I)⁻¹ vw
  double A[6][6], y[6];
  const double lam2 = 0.05 * 0.05;
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++) {
      double s = 0;
      for (int k = 0; k < 6; k++) s += J[r][k] * J[c][k];
      A[r][c] = s + ((r == c) ? lam2 : 0.0);
      y[r] = vw[r];
    }
  // 高斯消元（列主元）解 A y = vw
  for (int i = 0; i < 6; i++) {
    int piv = i;
    for (int r = i + 1; r < 6; r++)
      if (std::abs(A[r][i]) > std::abs(A[piv][i])) piv = r;
    if (piv != i) {
      for (int c = 0; c < 6; c++) std::swap(A[i][c], A[piv][c]);
      std::swap(y[i], y[piv]);
    }
    double d = A[i][i];
    if (std::abs(d) < 1e-12) d = 1e-12;
    for (int r = i + 1; r < 6; r++) {
      double f = A[r][i] / d;
      for (int c = i; c < 6; c++) A[r][c] -= f * A[i][c];
      y[r] -= f * y[i];
    }
  }
  for (int i = 6; i-- > 0;) {
    double s = y[i];
    for (int c = i + 1; c < 6; c++) s -= A[i][c] * y[c];
    double d = A[i][i];
    if (std::abs(d) < 1e-12) d = 1e-12;
    y[i] = s / d;
  }
  std::array<double, kAct> a{};
  const auto& vmax = env.sim().conf().arm.vmax;
  for (int j = 0; j < 6; j++) {
    double qd = 0;
    for (int k = 0; k < 6; k++) qd += J[k][j] * y[k];
    a[j] = std::clamp(qd / std::max(vmax[j], 1e-6), -1.0, 1.0);
  }
  return a;
}

// 合成示教专家轨迹（DLS 专家 rollout）
Episode expertEpisode(RLEnv& env, uint64_t seed, MLPPolicy& policy, int maxSteps) {
  (void)policy;
  env.reset(seed);
  Episode ep;
  for (int t = 0; t < maxSteps; t++) {
    RLObs o = env.observe();
    auto a = expertAction(env);
    double r;
    bool done;
    RLObs o2 = env.step(a, r, done);
    (void)o2;
    ep.steps.push_back({o.x, a, r});
    ep.totalReward += r;
    if (done) { ep.success = env.success(); break; }
  }
  return ep;
}

// DAgger 预训练：用当前 mean 策略 rollout 收集 (obs, 专家动作) 对并做 BC——
// 克服纯示教 BC 的协变量漂移（covariate shift）。
void daggerPretrain(RLEnv& env, MLPPolicy& policy, REINFORCETrainer& trainer,
                    uint64_t seed, int maxSteps, int iters, int episodesPerIter,
                    std::vector<std::array<double, kObs>>& anchorObss,
                    std::vector<std::array<double, kAct>>& anchorActs) {
  std::vector<std::array<double, kObs>> obss;
  std::vector<std::array<double, kAct>> acts;
  for (int it = 0; it < iters; it++) {
    obss.clear(); acts.clear();
    for (int d = 0; d < episodesPerIter; d++) {
      env.reset(seed * 1000 + d + uint64_t(it) * 77);
      for (int t = 0; t < maxSteps; t++) {
        RLObs o = env.observe();
        auto aStar = expertAction(env);
        obss.push_back(o.x);
        acts.push_back(aStar);
        auto a = policy.deterministic(o.x);
        double r; bool done;
        env.step(a, r, done);
        if (done) break;
      }
    }
    double l = 0;
    for (int e = 0; e < 25; e++) l = trainer.bcStep(obss, acts);
    std::printf("\r[dagger] iter %2d  对 %zu  BC loss %.5f", it, obss.size(), l);
    std::fflush(stdout);
    // 记录最后一批 (obs, 专家动作) 作为 RL 的 BC 锚定数据
    anchorObss = obss;
    anchorActs = acts;
  }
  std::printf("\n");
}

bool saveCheckpoint(const MLPPolicy& p, const std::string& path, int episode, double meanR) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  auto data = p.dumpParams();
  size_t n = data.size();
  f.write(reinterpret_cast<const char*>(&n), sizeof n);
  f.write(reinterpret_cast<const char*>(data.data()), std::streamsize(n * sizeof(float)));
  json::Value meta = json::Value::object();
  meta.set("episode", json::Value(double(episode)));
  meta.set("mean_reward", json::Value(meanR));
  std::ofstream mj(path + ".meta.json");
  mj << meta.dump();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  int episodes = 300, hidden = 64, demoEpisodes = 16, evalEvery = 30;
  double lr = 5e-4, gamma = 0.995, entropy = 0.02;
  uint64_t seed = 1;
  std::string outDir = "learning/checkpoints", logPath = "learning/logs/train.csv";
  std::string imitateFile;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--episodes") episodes = std::atoi(next());
    else if (a == "--hidden") hidden = std::atoi(next());
    else if (a == "--lr") lr = std::atof(next());
    else if (a == "--gamma") gamma = std::atof(next());
    else if (a == "--entropy") entropy = std::atof(next());
    else if (a == "--seed") seed = uint64_t(std::atoll(next()));
    else if (a == "--demo") demoEpisodes = std::atoi(next());
    else if (a == "--imitate") imitateFile = next();
    else if (a == "--out") outDir = next();
    else if (a == "--log") logPath = next();
    else if (a == "--eval-every") evalEvery = std::atoi(next());
    else { std::fprintf(stderr, "未知参数 %s\n", a.c_str()); return 2; }
  }

  RobotConf conf = RobotConf::desktop6();
  RLEnv env(conf, 0.002, 5, 4.0);
  MLPPolicy policy(size_t(hidden), seed);
  REINFORCETrainer trainer(policy, lr, gamma, entropy);
  const int maxSteps = int(RLEnv::kMaxT / (5 * 0.002)) + 2;   // ≈402
  std::vector<std::array<double, kObs>> anchorObss;
  std::vector<std::array<double, kAct>> anchorActs;
  // ---- 预训练①：web 示教轨迹 BC ----
  if (!imitateFile.empty()) {
    std::ifstream f(imitateFile);
    std::stringstream ss;
    ss << f.rdbuf();
    json::Value demo;
    if (json::Value::parse(ss.str(), demo)) {
      std::vector<std::array<double, kObs>> obss;
      std::vector<std::array<double, kAct>> acts;
      teachJsonToPairs(env, demo, obss, acts);
      std::printf("[imitate] %s → %zu 对 (obs,act)\n", imitateFile.c_str(), obss.size());
      for (int e = 0; e < 30 && !obss.empty(); e++)
        std::printf("\r  BC epoch %d loss=%.5f", e, trainer.bcStep(obss, acts));
      std::printf("\n");
    }
  }

  // ---- 预训练②：合成示教（P 专家）BC ----
  if (demoEpisodes > 0) {
    std::vector<std::array<double, kObs>> obss;
    std::vector<std::array<double, kAct>> acts;
    for (int d = 0; d < demoEpisodes; d++) {
      Episode ep = expertEpisode(env, seed * 1000 + d, policy, maxSteps);
      for (auto& s : ep.steps) { obss.push_back(s.obs); acts.push_back(s.act); }
    }
    std::printf("[demo] 合成示教 BC: %zu 对\n", obss.size());
    for (int e = 0; e < 60; e++)
      std::printf("\r  BC epoch %d loss=%.5f", e, trainer.bcStep(obss, acts));
    std::printf("\n");
    // DAgger：在 mean 策略到达的状态上补专家标注，克复协变量漂移
    daggerPretrain(env, policy, trainer, seed, maxSteps, 10, 4, anchorObss, anchorActs);
    trainer.setAnchor(anchorObss, anchorActs, std::getenv("LYRA_NO_ANCHOR") ? 0.0 : 10.0);
  }

  // ---- 评估函数（确定性策略 3 个固定种子）----
  auto evaluate = [&](int& ok, double& er) {
    ok = 0; er = 0;
    for (int k = 0; k < 3; k++) {
      env.reset(seed * 31 + k);
      double rsum = 0;
      for (int t = 0; t < maxSteps; t++) {
        RLObs o = env.observe();
        auto a = policy.deterministic(o.x);
        double r;
        bool done;
        env.step(a, r, done);
        rsum += r;
        if (done) { if (env.success()) ok++; break; }
      }
      er += rsum;
    }
    er /= 3;
  };
  {
    int ok; double er;
    evaluate(ok, er);
    std::printf("[eval] BC 后起点: 成功 %d/3  平均回报 %.2f\n", ok, er);
  }

  // ---- REINFORCE 主循环（攒批 updateEvery 回合更新一次）----
  std::ofstream log(logPath);
  log << "episode,steps,total_reward,success,loss,baseline\n";
  double ma = 0;
  const int updateEvery = 16;
  for (int ep = 0; ep < episodes; ep++) {
    env.reset(seed * 7919 + ep);
    Episode e;
    for (int t = 0; t < maxSteps; t++) {
      RLObs o = env.observe();
      auto a = policy.sample(o.x);          // 高斯原样（logπ 与之对应）
      std::array<double, kAct> aExec{};
      for (int i = 0; i < kAct; i++) aExec[i] = std::clamp(a[i], -1.0, 1.0);
      double r;
      bool done;
      RLObs o2 = env.step(aExec, r, done);  // 执行限幅 ±1
      (void)o2;
      e.steps.push_back({o.x, a, r});       // buffer 存未截断采样
      e.totalReward += r;
      if (done) { e.success = env.success(); break; }
    }
    double epReward = e.totalReward;
    int epSteps = int(e.steps.size());
    int epSucc = e.success ? 1 : 0;
    trainer.addEpisode(std::move(e));
    double loss = 0;
    if ((ep + 1) % updateEvery == 0 || ep == episodes - 1) loss = trainer.update();
    ma = trainer.baseline();
    log << ep << "," << epSteps << "," << epReward << "," << epSucc << "," << loss << ","
        << ma << "\n";
    if (ep % 5 == 4 || ep == episodes - 1)
      std::printf("[train] ep %3d  R=%8.2f  滑动 %8.2f  loss %8.3f  σ0 %.3f\n",
                  ep + 1, epReward, ma, loss,
                  std::exp(double(policy.logStd().data_read<float>()[0])));
    // 周期评估：确定性策略 3 个种子
    if ((ep + 1) % evalEvery == 0 || ep == episodes - 1) {
      int ok; double er;
      evaluate(ok, er);
      std::printf("        [eval] 成功 %d/3  平均回报 %.2f\n", ok, er);
    }
  }

  // checkpoint
  std::string ck = outDir + "/policy_final.bin";
  if (!saveCheckpoint(policy, ck, episodes, ma))
    std::fprintf(stderr, "[warn] checkpoint 写入失败（请先 mkdir -p %s）\n", outDir.c_str());
  else
    std::printf("[save] %s (%zu float)\n", ck.c_str(), policy.dumpParams().size());
  std::printf("[done] 训练完成，日志 %s\n", logPath.c_str());
  return 0;
}
