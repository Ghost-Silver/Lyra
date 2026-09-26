# 揽星 · Lyra — 桌面 6 轴机械臂

**运动学 / 规划 / 轨迹 / 仿真 / 控制 · Web 实时示教 · CTorch 学习层（REINFORCE 第一刀）**

Lyra 是一个面向桌面 6 轴机械臂的完整软件栈：确定性内核（DH 正逆运动学、数值/解析 IK、
SCurve 轨迹、纯 C++ 物理仿真）+ WebSocket 实时控制与 Three.js 数字孪生示教台 +
基于 [CTorch](https://github.com/ShengFlow/CTorch) 的模仿学习/强化学习训练闭环 +
真机串口（serial）通信接口预留。构建零第三方依赖（CTorch 为可选的学习层依赖）。

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
./build/arm_sim --demo               # 离线演示：PTP + 直线 + 圆弧 + 抓取
./build/arm_sim --selftest            # 内核自检
./build-learn/learning/arm_train --episodes 300 --seed 7 --eval-every 30 \
    --log learning/logs/train.csv --out learning/checkpoints
```

浏览器打开 `http://localhost:8080/`（本机）——数字孪生 + 6 滑杆 + 拖拽目标框 + 示教/回放。

### 测试

```bash
bash tests/run_tests.sh        # Layer 0 全部（5 项）+ test_learn（若已构建）
```

| 测试 | 覆盖 |
| --- | --- |
| `test_kinematics` | FK·DH·数值/解析 IK（300 随机位形）、雅可比、可操作度、**η 奇异指标**、slerp、位姿误差 |
| `test_planning` | PTP 关节规划、直线/圆弧笛卡儿轨迹、repel 排斥场、grasp 抓取门形轨迹（含闭合+离开） |
| `test_sim` | 单步伺服→关节收敛、速度跟踪、**RLEnv**（obs 归一/奖励/done/成功） |
| `test_json_ws` | JSON 全特性往返 + **RFC6455 帧编解码**（掩码/分片/ping/16/64 位长度） |
| `test_serial` | 帧编解码 CRC16 + 注入假串口回环 |
| `test_learn`（需 CTorch） | Linear 前向手算+解析梯度、Adam 单步手算、**logπ 图梯度 vs 有限差分**、REINFORCE 一次更新、BC 下降 |

---

## 1. 内核 `lib/arm/`

| 头文件 | 内容 |
| --- | --- |
| `kinematics.hpp` | `forwardKinematics`（DH+tool）·`numericIK`（阻尼最小二乘 DLS+关节限位投影）·`analyticIK`（Pieper 6R 闭式 8 解）·`buildJacobian`·`manipulability`（**无量纲行列式** `det(J̃J̃ᵀ)`，线部按特征长度 L 归一）·`singularityIndex`（**η=σ_min/σ_max**，行标定 Lchar=0.5 m 后的无量纲雅可比）·`isSingularNear`/`singularityScale`·`rotationSlerp`·`poseErrorVec` |
| `trajectory.hpp` | `SCurveProfile`：7 段 S 曲线速度规划，单段支持巡航、加/减速不对称、段长不足自动缩放 `vmax`；`scurvePlan` |
| `planning.hpp` | `jointPTP`（关节空间 PTP，SCurve 整段归一化时标定）·`cartesianLineTraj`（直线位姿插值+姿态 slerp）·`circlePoses`·`repelObstacle`（球形排斥场）·`graspPlan`（下降→闭合→**抬升离开段**→门形搬运→放置）·`fromRPY/toRPY` |
| `robot_conf.hpp` | `RobotConf::desktop6()` 桌面 6 轴参数（DH/限位/vmax/amax/jmax/PID/摩擦）·`Payload`·`gravityCompTorque`（负载重力补偿钩子） |
| `sim.hpp` | `ArmSim`：单周期 `step` = 重力矩前馈 + 位置/速度伺服 + 速度环半隐式欧拉 + 库/静摩擦 + 6 状态关节软限位（撞限位速度清零）；`RLEnv`：RL 环境包装（obs 18 维：位姿误差 6（pos/0.25、rot/π）+ q/qmax 6 + qd/vmax 6；act 6 = 归一化关节速度限幅；`maxT` 时域参数、`goalConfig()` 访问器） |

**奇异软降速**：`singularityScale(η)`——η ≥ 0.12 满速，η ≤ 0.02 线性降至 floor 0.2；
`isSingularNear(q, 0.02)` 供 UI/规划触发重规划。η 阈值经本臂实测标定（行归一 Lchar=0.5 m，
正常工作域 η≈0.02–0.26），勿照教科书硬套。`manipulability` 前向声明已于头内补全。

**伺服律**（`ArmSim`，与 web 实时行为一致，勿改）：
`v_want = sign(err)·min(posKp·|err|, sqrt(2·amax·|err|))`；平滑限速 `vs += α(v_want−vs)`；
`qd += clamp(vs−qd, ±amax·dt)`；急停锁存 → 速度目标强制为 0。急停优先级最高（锁存直到
`estop{on:false}`），软件限位永远在伺服内层。

## 2. 通信层 `lib/arm/control/`

| 头文件 | 内容 |
| --- | --- |
| `control_interface.hpp` | `ArmController` 接口：`enqueue(指令)`→`poll()`→`StateSnapshot`；50 Hz 控制节拍，`StateSnapshot` 含 `sigmaMin`/`sigIdx`（η）/`manip` |
| `json.hpp` | 自含 JSON 解析/序列化（UTF-8、转义、\uXXXX、数字、深度限制） |
| `ws_server.hpp` | 自含 RFC6455 服务端（握手 SHA1+Base64、掩码帧、分片、ping/pong、close） |
| `serial_driver.hpp` | **硬件接口预留**：`SerialDriver` 帧协议（SOF A5/长度/序号/负载类型/CRC16-IBM），`BytesSerial` 可注入假串口（测试回环）；真机路径（termios 打开 `/dev/tty*`）已留桩 |

**协议**（浏览器 ↔ 服务端，JSON 文本帧）：

- 服务端 → 浏览器：50 Hz `{"type":"state","t":…,"q":[…6],"qd":[…6],"tcp":[x,y,z],"pose":[rx,ry,rz],"sigma_min":…,"sig_idx":…,"manip":…,"estop":bool,"teach":[[…q6],…]}`
- 浏览器 → 服务端：`joint_target{q,speed}`（关节 PTP）·`ee_target{pos,rpy}`（数值 IK 直线）·
  `ee_drag{pos}`（IK 直线拖拽）·`joint_vel{qdot}`（速度模式）·`grip{width}`·
  `estop{on}`（锁存/解锁）·`reset`（回零+清示教+解锁）·
  `teach_add{q}`/`teach_clear`/`teach_play{speed}`/`teach_export`（→`teach_export` 响应，
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
  单端口同服静态+WS、允许任意 Origin；浏览器不直连其他端口。

## 4. 学习层 `learning/` + `ctorch_ext/`（需 `ARM_ENABLE_CTORCH=ON`）

| 文件 | 内容 |
| --- | --- |
| `ctorch_ext/linear.hpp` | `Linear` 全连接（W(out,in)+`x·Wᵀ`+ones·b 外积偏置，Xavier/He 种子化 init，前向全 CTorch 可微算子） |
| `ctorch_ext/adam.hpp` | `Adam`（m/v 偏置修正 + `gradNorm`/`clipGradNorm` 全局范数裁剪） |
| `learning/policy.hpp` | `MLPPolicy`：obs18→64→64→6 tanh 均值 + 可学习 `logStd`（6 维叶子）；`logProb`/`loss` 图内建（含熵正则、优势加权） |
| `learning/reinforce.hpp` | `REINFORCETrainer`：折扣回报 return-to-go − **滑动平均 baseline（同量纲·EMA）** + 批内中心化/白化 + BC 锚定项；`teachJsonToPairs`（web 示教 JSON → (obs,act) 对） |
| `learning/trainer_main.cpp` | `arm_train`：`--episodes/--hidden/--lr/--gamma/--entropy/--seed/--demo N/--imitate teach.json/--expert jointp\|dls/--out/--log/--eval-every`；预训练=合成示教 BC → **DAgger**（mean 策略 rollout 上补专家标注）→ REINFORCE 微调；输出训练 CSV + `policy_final.bin`（size_t n + n×float）+ `.meta.json` |
| `tests/test_learn.cpp` | 学习层单测（见测试表；logπ/loss 图梯度均与中央差分逐位吻合） |

**训练验证（本仓实测）**：

- test_learn 全绿：Linear/Adam 手算对照、logπ 与 loss 的图梯度 vs 有限差分吻合
  （如 `loss-grad: analytic=-0.094902 numeric=-0.094891`）、BC loss 30 步单调下降。
- 预训练曲线真实改善：示教 BC loss 0.34 → 0.06（60 epoch）；DAgger 后确定性策略
  从随机初始化的 ≈ −1200 提升到 ≈ −640（合成专家水平 −253 ~ −274，3/3 成功）。
- REINFORCE 端到端（采样→return-to-go→EMA baseline→白化→图梯度→Adam→checkpoint/CSV）
  全通，训练曲线随 `--log` 落盘。**如实说明**：400 步量级 horizon 上 REINFORCE 的
  信用分配方差占主导，本配置下策略改善不显著（第一刀的固有边界）；机制正确性由
  test_learn 的数值梯度对照保证。调参入口（lr/entropy/γ/锚定 λ/多步批）均已留 CLI/常量。

**CTorch 集成**：vendored 于 `third_party/CTorch`（v0.2.10，MIT），固定
`CT_ENABLE_MLIR=OFF`（免 LLVM）与 `CT_ENABLE_LTO=OFF`，C3 图融合/JIT 管线在无 MLIR
后端时编译期关闭（eager+autograd 数值等价）。**六处最小 vendored 补丁**（含一处
**转置梯度错位的关键正确性修复**）全部记录于 `third_party/PATCHES.md`——更新依赖前必读。

## 5. 修复工单（相对第一版工单，逐条完成）

1. ~~`planning.hpp` 缺 `<string>`~~ —— 已补，独立 TU 编译通过。
2. ~~include 路径混乱~~ —— 统一 `-Ilib -I.`（CMake `target_include_directories`）。
3. ~~IK 数学 bug~~ —— 数值 IK 阻尼最小二乘+限位投影；解析 IK 291/300 随机位形通过，
   失败样本均在关节限位外/奇异邻域（有意拒绝）。
4. ~~可操作度语义~~ —— `det(J̃J̃ᵀ)` 无量纲化（线部按特征长度归一），奇异→0。
5. ~~`graspPlan` 缺离开段~~ —— 闭合后抬升离开段 + 门形搬运 + 放置张开，测试覆盖。
6. ~~死代码~~ —— 声明/定义对齐（`manipulability` 等），`-Wall -Wextra` 零警告。
7. ~~`jointPTP` 速度~~ —— SCurve 整段归一化时标定，测试覆盖峰值速度约束。
8. ~~FK 热路径分配~~ —— `forwardKinematics` 可传入 scratch 缓冲复用 `Mat4`。
9. ~~`.gitignore` 截断~~ —— 补齐（构建/IDE/前端/学习产物分类）。
10. ~~README 与 CMake 不一致~~ —— 本文件即以当前 CMake 为准（含 `ARM_ENABLE_CTORCH`/
    `CTORCH_ROOT`）。

## 6. 硬件接口预留

`serial_driver.hpp` 帧协议（SOF/长度/序号/负载/CRC16-IBM）+ `BytesSerial` 注入测试已就绪；
真机接入只需实现 `SerialDriver` 的 termios open/read/write（桩已留），其余（指令打包、
状态解包、CRC 校验）在测试中回环验证。规划的帧类型：`CMD_MOVE`/`CMD_STOP`/`CMD_RESET`/
`STATE_FEEDBACK`/`ACK`。

## 目录

```
app/main.cpp        arm_sim 服务主循环（HTTP+WS 同端口、Scheduler、指令派发、--demo/--selftest）
lib/arm/            运动学/轨迹/规划/仿真/robot_conf（纯 C++ 头）
lib/arm/control/    接口/JSON/WS/serial
web/                静态前端（index.html/style.css/app.js）
tests/              单元测试 + run_tests.sh
ctorch_ext/         CTorch 扩展（Linear、Adam）
learning/           策略/REINFORCE/trainer_main（arm_train）
third_party/CTorch/ vendored CTorch（见 third_party/PATCHES.md）
cmake/              GCC 12 __bf16 兼容垫片
```

## 协议一览（速查）

| 用途 | 命令/字段 |
| --- | --- |
| 启动服务 | `./build/arm_sim --port 8080`（`0.0.0.0`，任意 Origin） |
| 关节 PTP | `{"cmd":"joint_target","q":[6],"speed":0.1…1}` |
| 直线/拖拽 | `{"cmd":"ee_target"/"ee_drag","pos":[3],"rpy":[3]?}` |
| 速度模式 | `{"cmd":"joint_vel","qdot":[6]}` |
| 急停/复位 | `{"cmd":"estop","on":true}`（锁存）/ `{"cmd":"reset"}` |
| 示教 | `teach_add`/`teach_clear`/`teach_play`/`teach_export` |
| 抓取 | `{"cmd":"grasp","pos":[3],"height":0.06}` |
| RL 训练 | `arm_train --episodes 300 --demo 16 --imitate teach.json --log … --out …` |
| RL checkpoint | `policy_final.bin`（size_t n + n×float）+ `.meta.json` |
