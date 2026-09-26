# 揽星 · Lyra — 桌面 6 轴机械臂

**运动学 / 规划 / 轨迹 / 仿真 / 控制 · Web 实时示教 · 串口真机路径 · CTorch 学习层**

Lyra 是一个面向桌面 6 轴机械臂的完整软件栈：确定性内核（DH 正逆运动学、数值/解析 IK、
SCurve 轨迹、纯 C++ 物理仿真）+ WebSocket 实时控制与 Three.js 数字孪生示教台 +
基于 [CTorch](https://github.com/ShengFlow/CTorch) 的模仿学习/强化学习训练闭环 +
真机串口（serial）通信路径（含非阻塞有界队列、PTY 从机闭环验证）。构建零第三方依赖
（CTorch 为可选的学习层依赖）。

```
┌────────────────┐   WebSocket/JSON    ┌──────────────────────────┐
│  web/index.html │ ◄────────────────► │  app/main.cpp (arm_sim)   │
│  Three.js 孪生  │   50 Hz state 帧    │  Scheduler 指令派发 2 kHz │
└────────────────┘                     ├──────────────────────────┤
                                       │ arm/ 内核：FK/IK·轨迹·仿真│
                                       ├──────────────────────────┤
                                       │ arm_train（学习层，可选） │
                                       │  REINFORCE+BC · CTorch   │
                                       └──────────────────────────┘
```

---

## 当前状态

| 能力 | 状态 |
| --- | --- |
| 运动学 / 规划 / 轨迹 / 仿真 | 完整，数学经独立数值验证（解析 IK 300/300、几何雅可比 vs 数值微分 1e-9） |
| Web 示教台（孪生 / 拖拽 / 示教回放） | 可用 |
| 安全层（限位硬 clamp / 速度·加速度审计 / 急停锁存） | 已实现并有回归 |
| 串口真机路径 | 代码路径完整（有界队列 / 部分写续传 / EAGAIN 重试 / 硬错误离线），经 PTY 从机闭环验证；**未接真实硬件** |
| CTorch 学习层 | 机制正确（梯度 vs 有限差分吻合）；**短程 REINFORCE 改善不显著**（方差主导，见第 4 节） |
| 认证 / TLS / 限速 / 审计 | **未实现**，见 `docs/SECURITY.md` |

**定位**：研究原型 / 技术验证平台。仿真是可信的，真机路径的**逻辑**经闭环验证，
但电气层、时序、下位机固件行为均未验证。

**安全边界**：默认无认证，面向实验室局域网。不可直接暴露公网（无认证、无 TLS、
Origin 默认放行、无速率限制）。真机接入前必须补独立的硬件急停回路与看门狗——
本服务只可作为上层指令源，**不得作为唯一安全链路**。

## 快速开始

### 构建

基础构建（内核 + `arm_sim` 服务 + 测试，无第三方依赖）：

```bash
cmake -B build -S .
cmake --build build -j
```

带学习层（`arm_train` / `test_learn`，需 vendored CTorch）：

```bash
cmake -B build-learn -S . -DARM_ENABLE_CTORCH=ON
cmake --build build-learn -j
```

| CMake 选项 | 默认 | 说明 |
| --- | --- | --- |
| `ARM_ENABLE_CTORCH` | `OFF` | `ON` 时构建 `arm_train` 与 `test_learn`（需 `third_party/CTorch`） |
| `CTORCH_ROOT` | `third_party/CTorch` | CTorch 源码路径（vendored 或自行 clone） |
| `ARM_BUILD_TESTS` | `ON` | 构建 `tests/` 下单元测试 |

工具链要求：g++ ≥ 10（C++17；学习层用 C++20）、CMake ≥ 3.16。GCC 12 及以下会自动
启用 `cmake/compat_bf16.h` 垫片（CTorch 需要 `__bf16` 类型，GCC 13+ 为内建）。
学习层集成固定 `CT_ENABLE_MLIR=OFF`（免 LLVM 依赖，走 eager+autograd，数值等价）。

### 运行

```bash
./build/arm_sim --port 8080          # WebSocket + 静态 web 同端口服务
./build/arm_sim --serial /dev/ttyUSB0 --serial-role master   # 真机语义（发指令/收状态）
./build/arm_sim --demo               # 离线演示：PTP + 直线 + 圆弧 + 抓取
./build/arm_sim --selftest            # 内核自检
./build-learn/learning/arm_train --episodes 300 --seed 7 --eval-every 30 \
    --log learning/logs/train.csv --out learning/checkpoints
```

浏览器打开 `http://localhost:8080/`（本机）——数字孪生 + 6 滑杆 + 拖拽目标框 + 示教/回放。

### 测试

```bash
bash tests/run_tests.sh              # Layer 0（8 项，纯 C++ 无依赖）
WITH_LEARN=1 bash tests/run_tests.sh # + 学习层 4 项（构建 CTorch）+ Runtime 探针
```

`run_tests.sh` 如实分层汇总：**Layer0**（8 项，含 `serial_loopback` PTY 闭环与 `test_longrun`
长时数值）/ **Learn**（4 项：test_learn、test_ctorch_transpose_grad、test_train_regression、
test_policy_mechanics）/ **Runtime**（`ws_probe.py`：第 17 连接 503、半开连接回收、超长请求头
丢弃、越限指令不变式与 `/api/health` 安全计数）/ **Long-run**（`soak_longrun.py`，默认跳过；
`SOAK_MINUTES=12` 启用三段式 RSS 曲线，实测见 `docs/soak-longrun-20260926.csv`）。未构建且未请求 → Learn 显示
`SKIPPED(not built)` 且退出码 0；**已构建/`WITH_LEARN=1` 却缺二进制 → FAIL 且退出码非 0**
（不把「没跑」伪装成「通过」）。

| 测试 | 覆盖 |
| --- | --- |
| `test_kinematics` | FK·DH·数值/解析 IK（300 随机位形，**解析 300/300、均 6.8 分支**）、**限位硬约束回归**、雅可比、可操作度、**η 奇异指标**、slerp、位姿误差 |
| `test_planning` | PTP 关节规划（**ts/qs 同步、时间戳严格单调**）、直线/圆弧笛卡儿轨迹、repel 排斥场、grasp 抓取门形轨迹（**独立抬升离开段**） |
| `test_sim` | 单步伺服→关节收敛、速度跟踪、**关节摩擦（静摩擦死区+稳态跌落）**、**RL 基线去摩擦**、**RLEnv**（obs 归一/奖励/done/成功） |
| `test_json_ws` | JSON 全特性往返 + **深度上限/严格数字/越界 API** 回归 + **RFC6455 帧编解码**（掩码/分片/ping/16/64 位长度、**回绕与超限帧协议错误**） |
| `test_serial` | 帧编解码 CRC16-CCITT + 注入假串口回环 |
| `test_safety` | **独立安全监控层**（B1：越限目标拦截+计数、速度/加速度审计、单拍跃变、急停锁存、关闭旁路）+ **轨迹时间基准防御**（B2：空/非单调/末值≤0 轨迹拒播、t=300 s 漂移回归） |
| `test_learn`（需 CTorch） | Linear 前向手算+解析梯度、Adam 单步手算、**logπ 图梯度 vs 有限差分**、REINFORCE 一次更新、BC 下降 |

---

## 1. 内核 `lib/arm/`

| 头文件 | 内容 |
| --- | --- |
| `kinematics.hpp` | `forwardKinematics`（DH+tool）·`numericIK`（阻尼最小二乘 DLS+关节限位投影）·`analyticIK`（Pieper 6R 闭式 8 解）·`buildJacobian`·`manipulability`（**无量纲行列式** `det(J̃J̃ᵀ)`，线部按特征长度 L 归一）·`singularityIndex`（**η=σ_min/σ_max**，行标定 Lchar=0.5 m 后的无量纲雅可比）·`isSingularNear`/`singularityScale`·`rotationSlerp`·`poseErrorVec` |
| `trajectory.hpp` | `SCurveProfile`：7 段 S 曲线速度规划，单段支持巡航、加/减速不对称、段长不足自动缩放 `vmax`；`scurvePlan` |
| `planning.hpp` | `jointPTP`（关节空间 PTP，SCurve 整段归一化时标定）·`cartesianLineTraj`（直线位姿插值+姿态 slerp）·`circlePoses`·`repelObstacle`（球形排斥场）·`graspPlan`（下降→闭合→**抬升离开段**→门形搬运→放置）·`fromRPY/toRPY` |
| `robot_conf.hpp` | `RobotConf::desktop6()` 桌面 6 轴参数（DH/限位/vmax/amax/jmax/PID/摩擦）·`Payload`·`gravityCompTorque`（负载重力补偿钩子） |
| `sim.hpp` | `ArmSim`：单周期 `step` = 重力矩前馈 + 位置/速度伺服 + 速度环半隐式欧拉 + **关节摩擦（库仑+粘性+静摩擦）** + 6 状态关节软限位（撞限位速度清零）；摩擦模型 `τ_f = b·qd + fc·sgn(qd)`，伺服刚度 `velKv` 折算稳态跌落 `τ_f/Kv`、`|τ_d|≤τs` 时粘滞锁定（静摩擦死区）——低速爬行/跟踪误差均为真实效应；**`RLEnv` 显式去摩擦（`rlBaseline`）保黄金回归逐位确定性**；`RLEnv`：RL 环境包装（obs 18 维：位姿误差 6（pos/0.25、rot/π）+ q/qmax 6 + qd/vmax 6；act 6 = 归一化关节速度限幅；`maxT` 时域参数、`goalConfig()` 访问器） |

**奇异软降速**：`singularityScale(η)`——η ≥ **0.05** 满速，η → 0 沿 smoothstep 降至 floor 0.2；
`isSingularNear(q, 0.02)` 供 UI/规划触发重规划。阈值按本臂 η 分布重标（3000 位形、关节限位内均匀采样实测，**仓内可复现**——
`./build/tests/test_kinematics --eta-stats`（seed=20260926；见 `docs/claims-audit.md`）：
p5=0.0054 / 中位 0.0700 / p95=0.1941；行归一 Lchar=0.5 m）：旧阈值 0.12 会让 **70.9%** 位形
进入降速区，重标 0.05 后 **37.2%**——且是 smoothstep 平滑过渡（η≥0.04 时 scale≥0.92、
占 **69.0%** 实际近乎满速；深度降速仅 η<0.02 的 **15.8%**）。`manipulability` 返 Lchar 归一无量纲 `|det J̃|`（线部 ÷L），
奇异→0。

**伺服律**（`ArmSim`，与 web 实时行为一致，勿改）：
`v_want = sign(err)·min(posKp·|err|, sqrt(2·amax·|err|))`；平滑限速 `vs += α(v_want−vs)`；
`qd += clamp(vs−qd, ±amax·dt)`；急停锁存 → 速度目标强制为 0。急停优先级最高（锁存直到
`estop{on:false}`），软件限位永远在伺服内层。

## 2. 通信层 `lib/arm/control/`

| 头文件 | 内容 |
| --- | --- |
| `control_interface.hpp` | `ArmController` 接口：`enqueue(指令)`→`poll()`→`StateSnapshot`；50 Hz 控制节拍，`StateSnapshot` 含 `sigmaMin`/`sigIdx`（η）/`manip` |
| `json.hpp` | 自含 JSON 解析/序列化（UTF-8、转义、\uXXXX、**严格 JSON 数字语法**、**嵌套深度上限 200 层**、越界安全取值 API） |
| `ws_server.hpp` | 自含 RFC6455 服务端（握手 SHA1+Base64、帧编解码、分片、ping/pong、close）。**安全硬化**：客户端帧强制掩码、控制帧 ≤125 禁分片、孤立 Continuation 拒收、握手三件套校验（Connection/Version/Key）、单帧/消息/缓冲/连接数/发送队列全限额、slowloris 超时回收、**发送严格非阻塞**（慢客户端只被丢弃，不拖死 50 Hz 回路：持续不读取的客户端实测在 **132.5 s** 时因发送缓冲耗尽被丢弃）、静态文件 realpath 前缀校验 + `O_NOFOLLOW`（symlink 逃逸拒绝）、Origin 白名单可选 |
| `serial_driver.hpp` | **串口真机路径**：帧协议 `[0xAA][0x55][type:u8][len:u8][payload][crc16:u16]`（**无序号字段**；CRC16-CCITT poly 0x1021 / init 0xFFFF，覆盖 type..payload）。`ByteIo` 抽象（真机 = `FdByteIo` 包 termios fd；测试 = 假串口 / PTY 从机）；**非阻塞有界待发队列**（部分写续传 / EAGAIN 重试 / 硬错误离线 / 队列满丢最旧且计数 / 已部分发送的帧绝不覆盖或丢弃）；角色 `master`（发 `CMD_*` 收 `STATE_REP`）或 `pendant`；统计经 `txStats()` 与 `/api/health.serial_*` 观测 |
| `safety_monitor.hpp` | **独立安全监控层**（不依赖规划/IK）：下发前位置硬 clamp + 计数、速度/加速度审计、单拍位移跃变回滚+清速；`--no-safemon` 关闭、`--safemon-estop` 越限急停；计数见 `/api/health` 的 `safety_*` |

**协议**（浏览器 ↔ 服务端，JSON 文本帧）：

- 服务端 → 浏览器：50 Hz `{"type":"state","t":…,"q":[…6],"qd":[…6],"tcp":[x,y,z],"pose":[rx,ry,rz],"sigma_min":…,"sig_idx":…,"manip":…,"estop":bool,"teach":[[…q6],…]}`
- 浏览器 → 服务端：`joint_target{q,speed}`（关节 PTP）·`ee_target{pos,rpy}`（数值 IK 直线）·
  `ee_drag{pos}`（IK 直线拖拽）·`joint_vel{qdot}`（速度模式）·`grip{width}`·
  `estop{on}`（锁存/解锁）·`reset`（回零+清示教+解锁）·
  `teach_add{q}`（**上限 4096 点**，超出回 `error{teach_full,limit}`）/`teach_clear`/`teach_play{speed}`/`teach_export`（→`teach_export` 响应，
  轨迹 JSON 可直接喂 `arm_train --imitate`）·`grasp{pos,height}`（门形抓取轨迹）

## 3. Web 前端 `web/`

纯静态三件套（`index.html` / `style.css` / `app.js`），Three.js（CDN）孪生体：

- **实时数字孪生**：three.js Link6+Joint6 圆柱构建（带 mesh 时优先真 mesh），TCP 发光点、
  目标框可**拖拽**（Z 平面拖动）、急停红罩、地板网格。
- **6 关节滑杆**：手动拖动 → `joint_target`（限幅 0.5 rad/s 等效速度）。
- **示教-回放**：`teach_add` 打点（可连续自动打点）、`teach_play` 回放、`teach_export`
  导出 JSON（供 RL `--imitate` 预训练）。
- **奇异指示条**：`sig_idx`（η）<0.02 红 / <0.06 黄 / 满宽 /0.2——无量纲、经行标定。
- 沙盒/反代安全：WS 一律 `wss(s)://当前host/ws` 相对路径，不写死 localhost；`arm_sim`
  单端口同服静态+WS；Origin 缺省放行（实验室工具/预览代理），`--allow-origin URL`（可重复）
  可收紧为白名单；浏览器不直连其他端口。

**安全边界（PR #1 审查后）**：默认面向实验室/局域网（无认证）——完整**威胁模型、未实现
防护清单（及其后果）、部署建议与已实现防护开关**见 **`docs/SECURITY.md`**。已内置：DoS 硬化
（帧/消息/缓冲/连接数/请求头/发送队列全限额、握手三件套校验、JSON 深度上限、realpath+symlink
逃逸防护、非阻塞发送——慢客户端不拖死控制回路）、**独立安全监控层**（下发前位置硬 clamp+计数、
速度/加速度审计、单拍跃变回滚；`--safemon-estop` 可选越限急停、`--no-safemon` 关闭；计数见
`/api/health` 的 `safety_*` 与状态 JSON 的 `safety` 对象）、轨迹时间基准拒播、`--allow-origin`
白名单。**公网暴露仍需**认证、WSS、强制 Origin 白名单与资源配额。

## 4. 学习层 `learning/` + `ctorch_ext/`（需 `ARM_ENABLE_CTORCH=ON`）

| 文件 | 内容 |
| --- | --- |
| `ctorch_ext/linear.hpp` | `Linear` 全连接（W(out,in)+`x·Wᵀ`+ones·b 外积偏置，Xavier/He 种子化 init，前向全 CTorch 可微算子） |
| `ctorch_ext/adam.hpp` | `Adam`（m/v 偏置修正 + `gradNorm`/`clipGradNorm` 全局范数裁剪） |
| `learning/policy.hpp` | `MLPPolicy`：obs18→64→64→6 tanh 均值 + 可学习 `logStd`（6 维叶子）；`logProb`/`loss` 图内建（含熵正则、优势加权） |
| `learning/reinforce.hpp` | `REINFORCETrainer`：折扣回报 return-to-go − **滑动平均 baseline（同量纲·EMA）** + 批内中心化/白化 + BC 锚定项；`teachJsonToPairs`（web 示教 JSON → (obs,act) 对） |
| `learning/trainer_main.cpp` | `arm_train`：`--episodes/--hidden/--lr/--gamma/--entropy/--seed/--demo N/--imitate teach.json/--expert jointp\|dls/--out/--log/--eval-every`；预训练=合成示教 BC → **DAgger**（mean 策略 rollout 上补专家标注）→ REINFORCE 微调；输出训练 CSV + `policy_final.bin`（size_t n + n×float）+ `.meta.json` |
| `tests/test_learn.cpp` | 学习层单测（见测试表；logπ/loss 图梯度均与中央差分逐位吻合） |
| `tests/test_policy_mechanics.cpp` | **机制单测（F1）**：`logStd` 进入优化器且熵项下梯度 = −β·B、熵项定量关系、`clipGradNorm` 裁到阈值且等比缩放、advantage 白化后均值≈0/σ≈1、**零方差批走 1.0 兜底不除零** |

**训练验证（本仓实测）**：

- test_learn 全绿：Linear/Adam 手算对照、logπ 与 loss 的图梯度 vs 有限差分吻合
  （如 `loss-grad: analytic=-0.094902 numeric=-0.094891`，**量级参考**：硬门槛是逐元素
  吻合容差，数值随初值/seed 变化）、BC loss 单调下降。
- 预训练曲线真实改善（**量级参考**——数值随 seed/epoch/示教集而变；仓库回归基准见
  `tests/test_train_regression.cpp`，复现：`WITH_LEARN=1 bash tests/run_tests.sh`）：
  **多种子口径**（F2）：短程 REINFORCE 的「显著/不显著」本身就带统计噪声——5 个种子实测
  1/5 显著改善、4/5 不显著（`./build-learn/tests/test_train_regression --seed-scan`），
  故默认测试改用**稳健断言**（末段均值 ≥ 首段 − 1σ；回报 σ 比值 ∈[0.2, 5]），
  只在「统计显著退化」时硬失败。
  示教 BC loss 0.34 → 0.06（60 epoch）；DAgger 后确定性策略
  从随机初始化的 ≈ −1200 提升到 ≈ −640（合成专家水平 −253 ~ −274，3/3 成功）。
- REINFORCE 端到端（采样→return-to-go→EMA baseline→白化→图梯度→Adam→checkpoint/CSV）
  全通，训练曲线随 `--log` 落盘。**如实说明**：400 步量级 horizon 上 REINFORCE 的
  信用分配方差占主导，本配置下策略改善不显著（第一刀的固有边界）；机制正确性由
  test_learn 的数值梯度对照保证。调参入口（lr/entropy/γ/锚定 λ/多步批）均已留 CLI/常量。

**CTorch 集成**：vendored 于 `third_party/CTorch`（v0.2.10，MIT），固定
`CT_ENABLE_MLIR=OFF`（免 LLVM）与 `CT_ENABLE_LTO=OFF`，C3 图融合/JIT 管线在无 MLIR
后端时编译期关闭（eager+autograd 数值等价）。**七类最小 vendored 补丁**（`ctorch-lyra.patch`
12 文件 + `c3-lyra.patch` 2 文件；含一处**转置梯度错位的关键正确性修复**，回归见
`tests/test_ctorch_transpose_grad.cpp`：回退补丁即 FAIL、恢复即 PASS）全部记录于
`third_party/PATCHES.md`——更新依赖前必读。

## 5. 修复工单（相对第一版工单，逐条完成）

1. ~~`planning.hpp` 缺 `<string>`~~ —— 已补，独立 TU 编译通过。
2. ~~include 路径混乱~~ —— 统一 `-Ilib -I.`（CMake `target_include_directories`）。
3. ~~IK 数学 bug~~ —— 数值 IK 阻尼最小二乘 + **逐步限位投影**；解析 IK **300/300** 随机位形全通
   （均 6.8 分支）。历史 291/300 的 9 个失败样本**均在限位内**，根因是腕翻转分支漏 `θ6+π`
   的退化漏解（已修复，见 PR #1 审查 P0-补-1）；越限解一律**硬过滤**（不返回、不交给下游）。
4. ~~可操作度语义~~ —— `det(J̃J̃ᵀ)` 无量纲化（线部按特征长度归一），奇异→0。
5. ~~`graspPlan` 缺离开段~~ —— 闭合后**独立抬升离开段**（高于 approach 至少 `max(approachD/2, 2cm)`，`leaveLift` 可配）+ 门形搬运 + 放置张开，测试回归覆盖。
6. ~~死代码~~ —— 声明/定义对齐（`manipulability` 等），`-Wall -Wextra` 零警告。
7. ~~`jointPTP` 速度~~ —— SCurve 整段归一化时标定，**`ts` 与 `qs` 同步写入且严格单调**（播放端按 `(ts,qs)` 时间轴推进；缺 ts 会让首拍即判完成——PR #1 审查 P0-1，已修复），测试覆盖峰值速度约束与时间戳单调性。
8. ~~FK 热路径分配~~ —— `forwardKinematics` 可传入 scratch 缓冲复用 `Mat4`。
9. ~~`.gitignore` 截断~~ —— 补齐（构建/IDE/前端/学习产物分类）。
10. ~~README 与 CMake 不一致~~ —— 本文件即以当前 CMake 为准（含 `ARM_ENABLE_CTORCH`/
    `CTORCH_ROOT`）。

## 6. 串口真机路径

`serial_driver.hpp` 帧协议 `[0xAA][0x55][type][len][payload][crc16]`（无序号字段，CRC16-CCITT）。
真机接入的**代码路径已是生产级**：termios 打开 `/dev/tty*`（raw/8N1/无流控）→ `ByteIo` 抽象 →
有界待发队列与非阻塞写（部分写续传 / EAGAIN 重试 / 硬错误离线），角色由 `--serial-role`
选择 `master`（发 `CMD_POS`/`CMD_VEL`/`CMD_GRIP`/`CMD_ESTOP`，收 `STATE_REP`，状态经
`/api/health.serial_*` 观测）或 `pendant`（推 `STATE_REP`，收回指令进主调度）。

**闭环验证方式（无硬件）**：`tests/serial_loopback.cpp` 用 `posix_openpt` 起 PTY，本仓内的
从机模拟器解析 `CMD_*` 并回 `STATE_REP`，端到端验证往返数值、坏 CRC 拒收与链路自愈、分片重组
与急停通道（实测输出见 PR #1 回复「已完成批次 E–H」）。

**仍未验证**：真实串口的电气层、波特率容差、长线噪声与下位机固件行为（本仓无硬件）；
`SECURITY.md` 如实标注真机安全回路仍需独立硬件实现。

## 目录

```
app/main.cpp        arm_sim 服务主循环（HTTP+WS 同端口、Scheduler、指令派发、--demo/--selftest）
lib/arm/            运动学/轨迹/规划/仿真/robot_conf（纯 C++ 头）
lib/arm/control/    接口/JSON/WS/serial/安全监控层
web/                静态前端（index.html/style.css/app.js）
tests/              单元测试 + run_tests.sh + PTY/WS/长时探针
ctorch_ext/         CTorch 扩展（Linear、Adam）
learning/           策略/REINFORCE/trainer_main（arm_train）
third_party/CTorch/ vendored CTorch（见 third_party/PATCHES.md）
docs/               SECURITY.md（威胁模型/防护清单）、claims-audit.md（声称-实现对照）
cmake/              GCC 12 __bf16 兼容垫片
```

## 协议一览（速查）

| 用途 | 命令/字段 |
| --- | --- |
| 启动服务 | `./build/arm_sim --port 8080`（`0.0.0.0`；Origin 缺省放行，`--allow-origin` 可收紧） |
| 关节 PTP | `{"type":"joint_target","q":[6],"speed":0.1…1}` |
| 直线/拖拽 | `{"type":"ee_target"/"ee_drag","pos":[3],"rpy":[3]?}` |
| 速度模式 | `{"type":"joint_vel","qd":[6]}` |
| 急停/复位 | `{"type":"estop","on":true}`（锁存）/ `{"type":"reset"}` |
| 示教 | `teach_add`/`teach_clear`/`teach_play`/`teach_export` |
| 抓取 | `{"type":"grasp","pos":[3],"height":0.06}` |
| RL 训练 | `arm_train --episodes 300 --demo 16 --imitate teach.json --log … --out …` |
| RL checkpoint | `policy_final.bin`（size_t n + n×float）+ `.meta.json` |
