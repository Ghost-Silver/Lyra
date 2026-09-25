# 揽星 · Lyra — 桌面级 6 轴机械臂

> 名字含义：Lyra（天琴座）六颗主星呼应六个自由度，"揽星"取伸手触达星辰之意。

一个**从仿真到控制界面**的桌面 6 轴机械臂软硬件一体化项目。

> 完整设计背景尚未整理成独立文档，暂以本文为准，后续会定期回填到 `docs/`。

## 目标形态

全流程完整项目：仿真引擎 → 控制接口（硬件预留）→ 学习层（CTorch / 可选）→ 实时 3D 控制界面。
纯仿真为主，硬件驱动接口以抽象基类 + 串口骨架形式预留，便于后续烧录真机。

## 架构总览（三层）

| 层 | 选型 | 职责 | 依赖 |
|---|---|---|---|
| **Layer 0 · 实时内核** | 0 依赖纯 C++17 | FK/IK / 雅可比 / 奇异检测、轨迹·路径规划、抓取编排、状态积分器、抽象控制接口 | 无 |
| **Layer 1 · 学习层** | 团队的 **CTorch**（`add_subdirectory` 引用） | RL 策略训练与推理、神经逆解残差、抓取位姿预测 | CTorch（可选，CMake 开关隔离） |
| **Layer 2 · 前端** | Three.js 实时 3D | 可视化 / 拖拽控制 / 滑杆 / 示教回放 / 面板 | 浏览器 |

**混合架构要点**：实时内核保持 0 依赖自包含，断开 CTorch 也能编译、也能直接驱动真机固件；
学习层作为可插拔模块存在，二者互不拖累。

## 关键技术决策（来自设计讨论）

- 机械臂为**独立工程**，通过 `add_subdirectory` 引入 CTorch（路径默认 `~/LuoJin/CTorch-optimize-AutoDiff`，CMake 变量可改）。
- 学习层第一刀：**RL 控制策略**（端到端关节空间速度控制），算法起步 **REINFORCE + baseline**。
- 给 CTorch 补 **`Linear` 全连接层 + `Adam` 优化器**作为前置工作包（当前 CTorch 尚无这两者）。
- 硬件阶段：**纯仿真为主**，预留串口/舵机/步进驱动接口与协议。

## 目录结构（规划）

```
6axis_arm/
├── CMakeLists.txt            # 顶层构建
├── README.md
├── app/
│   └── main.cpp              # 主程序：WebSocket 服务 + 仿真/调度主循环
├── lib/arm/                  # Layer 0 实时内核（0 依赖）
│   ├── kinematics.hpp        # DH/FK / 解析+数值 IK / 雅可比 / 奇异度量   [已建]
│   ├── planning.hpp          # 关节 PTP / 笛卡尔直线·圆弧 / 势场避障 / 抓取编排 [已建]
│   ├── trajectory.hpp        # S 形速度曲线 / 平滑时间缩放             [规划]
│   ├── sim.hpp               # 状态积分器 / RL 仿真环境(step/reset)    [规划]
│   ├── robot_conf.hpp        # DH 参数 / 关节限位 / 负载 / 工具系       [规划]
│   └── control/
│       ├── control_interface.hpp  # 抽象控制接口（发送状态/接收指令）   [规划]
│       ├── ws_server.hpp          # WebSocket 服务实装 (RFC6455)      [规划]
│       └── serial_driver.hpp      # 串口驱动骨架（真机预留）            [规划]
├── ctorch_ext/               # 给 CTorch 补的前置功（独立小包）
│   ├── linear.hpp            # Linear 全连接层（affine + 激活）
│   └── adam.hpp              # Adam 优化器
├── learning/                 # Layer 1 CTorch 学习层（可选，ARM_ENABLE_CTORCH=ON）
│   ├── CMakeLists.txt
│   ├── policy.hpp            # MLP 策略 (obs 18 → act 6)
│   ├── reinforce.hpp         # REINFORCE + baseline 训练器
│   └── trainer_main.cpp      # 训练主循环（环境 = Layer0 仿真）
└── web/                      # Layer 2 前端
    ├── index.html
    ├── app.js
    └── style.css
```

## 构建

### 仅实时内核（默认，不依赖 CTorch）

```bash
cmake -B build -DARM_ENABLE_CTORCH=OFF
cmake --build build
./build/arm_sim
```

### 启用 CTorch 学习层

```bash
cmake -B build-learn -DARM_ENABLE_CTORCH=ON -DCTORCH_ROOT="$HOME/LuoJin/CTorch-optimize-AutoDiff"
cmake --build build-learn
```

> `CTORCH_ROOT` 指向 CTorch 仓库根；未设置时默认回退到上述路径。

## 功能与设计补充点

- **运动学**：标准 DH 建模；解析 + 多初值数值 IK（阻尼最小二乘）并用 FK 自校验；几何雅可比可操控度奇异检测。
- **规划**：关节 PTP 梯形曲线 → 升级 S 形（加加速度受限）；笛卡尔直线/圆弧路径；水平面势场避障；抓取"接近→下降→离开"编排。
- **奇异软处理**：manipulability 阈值触发末端降速，训练/执行时规避奇异。
- **工具系与负载**：预留 tool frame 与末端负载重力补偿参数（真机与 RL 补偿的钩子）。
- **安全底线**：软件限位 + 急停接口。
- **RL**：观测 18 维（末端位姿误差 6 + 关节角 6 + 关节角速度 6），动作 6 维（关节速度指令，语义限幅）；观测归一化 + episode 长度/终止条件；baseline 采用滑动平均 return（首版零成本），可后升级 critic。
- **前端**：实时渲染 + 末端拖拽逆解跟随 + 滑杆关节控制 + **示教-回放**模式（轨迹可喂给 RL 预训练）。
- **确定性**：随机种子贯通运动学与 RL，保证实验可复现。
- **控制接口**：WebSocket 服务实装（广播状态 / 接收指令），串口骨架预留，统一抽象基类。

## 路线图（分阶段，可独立验收）

1. **内核落地**：运动学 + 规划 + 状态积分器 + 抽象控制接口（纯 C++，可编译运行）。
2. **前端联通**：Three.js 实时 3D + 控制面板，WebSocket 连后端。
3. **CTorch 前置包**：补 `Linear` + `Adam`（独立小包）。
4. **RL 第一刀**：REINFORCE + baseline 训练关节空间速度控制策略（环境 = Layer 0 仿真）。
5. **硬件接口**：串口/舵机/步进驱动协议补全（纯仿真为主阶段仅保留接口）。

## 状态

- [x] 设计案讨论与定版
- [ ] 内核落地
- [ ] 前端联通
- [ ] CTorch 前置包
- [ ] RL 第一刀
- [ ] 硬件接口预留

---
团队：ShengFlow / 鹤归潮