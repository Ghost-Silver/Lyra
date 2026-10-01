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
| CTorch 学习层 | 机制正确（梯度 vs 有限差分吻合）；短程 REINFORCE 改善不显著（方差主导） |
| 认证 / TLS / 限速 / 审计 | **开发中**（即将实现基于 Token 的基础认证拦截） |
| 连续轨迹与圆弧插补 | **新增支持**（多段笛卡尔折线的圆弧倒角融合，保持非零速度；支持标准圆弧插补） |

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

工具链要求：g++ ≥ 10（C++17；学习层用 C++20）、CMake ≥ 3.16。

### 运行

```bash
./build/arm_sim --port 8080          # WebSocket + 静态 web 同端口服务
./build/arm_sim --serial /dev/ttyUSB0 --serial-role master   # 真机语义
./build/arm_sim --demo               # 离线演示
./build/arm_sim --selftest            # 内核自检
```

浏览器打开 `http://localhost:8080/`（本机）——数字孪生 + 6 滑杆 + 拖拽目标框 + 示教/回放。

### 测试

```bash
bash tests/run_tests.sh              # Layer 0 纯 C++ 测试
WITH_LEARN=1 bash tests/run_tests.sh # + 学习层测试 + Runtime 探针
```

---

## 1. 内核 `lib/arm/`

| 头文件 | 内容 |
| --- | --- |
| `kinematics.hpp` | FK·数值 IK（阻尼最小二乘 DLS）·解析 IK（Pieper 6R 闭式）·雅可比与可操作度指标 |
| `trajectory.hpp` | 7 段 S 曲线速度规划（SCurveProfile） |
| `planning.hpp` | 关节 PTP·笛卡尔直线（`cartesianLineTraj`）·**笛卡尔圆弧**（`cartesianCircleTraj`）·**连续笛卡尔折线融合**（`cartesianBlendPathTraj`）·抓取编排 |
| `robot_conf.hpp` | `RobotConf::desktop6()` 桌面 6 轴参数（DH/限位/vmax/amax/jmax/PID/摩擦） |
| `sim.hpp` | 单周期伺服仿真，含关节库仑/粘性/静摩擦模型；RLEnv 强化学习环境包装 |

**奇异软降速**：奇异指标 η ≥ **0.05** 时满速，η → 0 沿 smoothstep 平滑降速。

## 2. 通信层 `lib/arm/control/`

| 头文件 | 内容 |
| --- | --- |
| `ws_server.hpp` | 自含 RFC6455 服务端（无依赖）。安全硬化：禁分片、握手校验、单帧/消息/缓冲全限额、**发送严格非阻塞**（慢客户端不拖死 50 Hz 回路） |
| `serial_driver.hpp` | **串口真机路径**：帧协议 `[0xAA][0x55][type][len][payload][crc16]`。非阻塞有界待发队列（部分写续传 / EAGAIN 重试 / 硬错误离线）。 |
| `safety_monitor.hpp` | 独立安全监控层：下发前位置硬 clamp、速度/加速度审计、单拍位移跃变回滚+清速。 |

## 3. Web 前端 `web/`

纯静态三件套（`index.html` / `style.css` / `app.js`），集成 Three.js 数字孪生：
- 实时同步机械臂位姿，TCP 目标框可拖拽。
- 关节滑杆与示教/回放功能，可直接导出轨迹供 RL 预训练。

## 4. 学习层 `learning/` + `ctorch_ext/`（可选）

集成 vendored CTorch，实现了全连接策略网络（MLPPolicy）、Adam 优化器以及端到端 REINFORCE 训练循环。机制正确（图梯度与有限差分吻合），可运行于短程任务。

---

## 目录与协议速查

```
app/main.cpp        arm_sim 服务主循环（HTTP+WS、指令派发调度）
lib/arm/            运动学/轨迹/规划/仿真/robot_conf 内核
lib/arm/control/    接口/JSON/WS/serial/安全监控层
web/                静态前端界面
tests/              单元测试与自动化验证脚本
learning/           策略/REINFORCE 学习层
```

| 功能 | WebSocket 指令格式 |
| --- | --- |
| 关节 PTP | `{"type":"joint_target","q":[6],"speed":0.3}` |
| 笛卡尔直线 | `{"type":"ee_line","pos":[3],"speed":0.12}` |
| 笛卡尔圆弧 | `{"type":"ee_circle","center":[3],"normal":[3],"radius":0.1,"sweep":3.14}` |
| 连续笛卡尔路径 | `{"type":"ee_path","poses":[...],"blend_radius":0.02}` |
| IK 直线打靶/拖拽 | `{"type":"ee_target"/"ee_drag","pos":[3],"rpy":[3]?}` |
| 直接速度控制 | `{"type":"joint_vel","qd":[6]}` |
| 夹爪 / 急停 | `{"type":"grip","g":1.0}` / `{"type":"estop","on":true}` |
