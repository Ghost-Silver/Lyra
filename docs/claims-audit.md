# 声称—实现—证据对照表（claims audit）

本表逐条核对 README 中的**强断言**（带具体数字、绝对化用词、或作为交付卖点的说法），
每一条都给出实现位置与**可在本仓库内复现**的证据或测试；不成立/无法验证的条目如实标注。

- 方法：`git grep` 定位实现 → 找对应测试/命令 → 实际运行取输出 → 记入"证据"列。
- 原则：**没有仓内复现路径的数字不写成强断言**；宁可标"量级参考"并附采样条件。
- 审计基准：`arena/01a0d869-lyra`（本表随代码更新，最近一次审计见 PR #1 回复「已完成批次 D」）。
- 复现全部：`bash tests/run_tests.sh`（Layer0 6 项）→ `WITH_LEARN=1 bash tests/run_tests.sh`
  （+ 学习层 3 项 + Runtime 探针）。

## 1. 运动学 / 规划 / 仿真

| 声称（README 位置） | 实现 | 证据（仓内可复现） | 结论 |
| --- | --- | --- | --- |
| 解析 IK 300/300、均 6.8 分支（测试表 / §5 工单 3） | `lib/arm/kinematics.hpp` `analyticIK`（Pieper 8 解，含腕翻转分支） | `tests/test_kinematics.cpp`（本地随机序列 300 位形）；输出 `[kin] analytic IK: 300/300 solved, avg branches 6.8` | ✅ 成立（不随 seed 波动：越限解被硬过滤，统计口径为"限位内可解"） |
| 越限解一律硬过滤、不交给下游（§5 工单 3） | `analyticIK` 结果逐个 `qmin/qmax` 校验后丢弃 | `tests/test_kinematics.cpp`（越限断言）+ `tests/test_planning.cpp`（规划器输入合法性） | ✅ 成立 |
| `manipulability` = 线部按 Lchar 归一的 `det(J̃J̃ᵀ)`，"无量纲"（§1 表 / §1 说明） | `kinematics.hpp:301-315`（线速度行 ÷Lchar=0.5 m，角速度行不变） | `tests/test_kinematics.cpp` 可操控度断言；量纲推导：线行 [m/rad]→[1/rad]，rad 按工程惯例视为无量纲 | ✅ 成立（措辞含惯例说明） |
| `singularityIndex` η = σ_min/σ_max，行标定 Lchar=0.5 m（§1 表） | `kinematics.hpp` `singularityIndex`（与 manipulability 同一行标定） | `tests/test_kinematics.cpp`（腕奇异 σ_min 塌缩断言） | ✅ 成立 |
| η 分布数字（p5=0.0059 / 中位 0.0753 / p95=0.1953；旧阈值 0.12→70.9%、0.05→34.0%、η<0.02→14.5%）（§1 说明） | 原数字来自**未入仓探针**（不同 RNG 序列） | **本轮新增仓内入口**：`tests/test_kinematics.cpp --eta-stats`（seed=20260926，3000 位形、限位内均匀采样）→ 实测 p5=0.0054 / 中位=0.0700 / p95=0.1941；0.12→75.9%、0.05→37.2%、η<0.02→15.8% | ⚠️→✅ 已修：README 数字替换为仓内复现值（差异 ≤5 个百分点，来源为采样序列不同） |
| PTP `ts` 与 `qs` 同步写入且**严格单调**（§5 工单 7） | `lib/arm/trajectory.hpp` 写入 + `lib/arm/traj_player.hpp` 播放前断言（B2） | `tests/test_planning.cpp`（时间戳单调/峰值速度）+ `tests/test_safety.cpp`（空/非单调/末值≤0 拒播） | ✅ 成立 |
| 关节摩擦 τ_f = b·qd + fc·sgn(qd)，含静摩擦死区；RL 基线显式去摩擦（§1 表 sim 行） | `lib/arm/sim.hpp:71-86`（库仑+粘性+静摩擦，`|τ_d|≤τs` 粘滞锁定；`rlBaseline` 走无摩擦路径） | `tests/test_sim.cpp`（稳态跌落=τ_f/Kv、静摩擦死区、RL 基线逐位回归） | ✅ 成立 |

## 2. 通信 / 安全硬化

| 声称（README 位置） | 实现 | 证据（仓内可复现） | 结论 |
| --- | --- | --- | --- |
| 串口帧协议 "SOF A5 / 长度 / **序号** / CRC16-**IBM**"（§2 表 + §6） | 实际代码：`[0xAA][0x55][type:u8][len:u8][payload][crc16:u16]`，**无序号字段**；CRC16-**CCITT**（poly 0x1021，init 0xFFFF），覆盖 type..payload（`lib/arm/control/serial_driver.hpp:3-11,38-55`） | `tests/test_serial.cpp` 回环编解码；`git grep` 原文 | ❌→✅ 已修：README 两处改为代码实测描述 |
| 单帧/消息/缓冲/连接数/发送队列全限额（§2 表 ws_server 行） | `ws_server.hpp`：16 连接 / 16 KiB 请求头 / 1 MiB 消息 / 2 MiB 缓冲 / 256 KiB 发送队列 / 5 s slowloris 超时 | `tests/ws_probe.py` 三场景实测：第 17 连接 → **503 + busy 体，1.9 ms**；超长请求头（32 KiB）→ 丢弃 **3.8 ms**；半开连接（RST）回收 **13.9 ms**；`/api/health` 暴露 `rejected_clients`/`header_rejects` | ✅ 成立（且补齐两处）：①请求头上限此前**只声明未使用**，超长头会被当正常请求服务；②连接数超限原为静默关闭（RST 会截断响应），现改为"半关写端→读掉入站→FIN"的 503 |
| 静态文件 realpath 前缀校验 + `O_NOFOLLOW`（symlink 逃逸拒绝）（§2 表） | `ws_server.hpp:484-505` | `tests/ws_probe.py`：`web/_probe_link.html→/etc/passwd` 返回 **403** | ✅ 成立（无回归） |
| 发送严格非阻塞、慢客户端只被丢弃不拖死 50 Hz 回路（§2 表） | `ws_server.hpp` 有界发送队列 + 非阻塞冲刷 | `tests/bench_ws.cpp`（`./build-tests/bench_ws 1500 <port> 192`）：50 Hz 广播 + 4 客户端持续拉 192 KiB 文件（81 MiB 流量）→ **0/1500 拍超 2 ms 预算**，`ws.service()` 均值 4.4 µs / p99 12.2 µs / 最坏 995 µs（最坏拍=首次缓存未命中）；只连不读的 WS 客户端第 0 拍即被丢弃 | ✅ 成立 |
| 静态文件读取不阻塞控制回路（本轮加固，§2 表新增） | `ws_server.hpp`：内存缓存（8 MiB 上限）+ 单文件 **192 KiB 上限（超出 413）**，上限 ≤ 发送队列 | `tests/ws_probe.py`：300 KiB 文件 → **413**；缓存命中计数递增；`bench_ws` 对照：直读 192 KiB=0.685 ms，而直读 2 MiB=**6.4 ms ≈ 3 个控制周期**（改前的大文件路径量级；上限取值依据） | ✅ 新增 |
| 越限指令的防御纵深（本轮新增） | 指令入口 `gate_.clampQ/clampV`（第一层）+ `SafetyMonitor` 下发前/积分后（第二层） | `tests/ws_probe.py` 场景4：WS 注入 `joint_target q=[5×6]` → 状态 `max|q|=0.49 ≤ 2.967` 不越限（第一层夹回，故安全层计数不增——**预期**）；安全层自身拦截由 `tests/test_safety.cpp` 直接注入验证（posClamps 递增、旧规划器越限轨迹 400 拍零泄漏）。`/api/health` 暴露 `safety_enabled/safety_*` 计数 | ✅ 成立（分层防御，语义已注明） |
| 50 Hz 状态广播（§2 协议） | `app/main.cpp:467`：`bcastEvery = 0.02/dt`（dt=2 ms 物理步长） | `tests/bench_ws.cpp` 按同一节拍压测；`/api/health` 可查实时客户端数 | ✅ 成立 |

## 3. 学习层（CTorch 集成）

| 声称（README 位置） | 实现 | 证据（仓内可复现） | 结论 |
| --- | --- | --- | --- |
| vendored 补丁"**六处**"（§4 末） | `third_party/PATCHES.md` 实际列 **7 类**；`third_party/patches/` 两文件（`ctorch-lyra.patch` 12 文件 + `c3-lyra.patch` 2 文件） | `git grep -c '^[0-9]\+\. ' third_party/PATCHES.md` → 7 | ❌→✅ 已修：改为"七类最小 vendored 补丁（12+2 文件）" |
| 转置梯度错位已修复（PATCHES §7） | `TransposeNode::backward` + `GradAccumulator` 物化 `.contiguous()` | `tests/test_ctorch_transpose_grad.cpp`：正向（解析 vs 中央差分，偏差 1.1e-5）+ 负向判别力断言。**实测回退补丁 → 解析梯度变为 [1 1 2 2 3 3]（与 PATCHES 记录的历史错误值逐位一致）→ 测试 FAIL；恢复 → [1 2 3 1 2 3] → PASS** | ✅ 成立（本轮补齐可执行回归） |
| Linear 前向全 CTorch 可微算子、logπ/loss 图内建（§3 表） | `ctorch_ext/linear.hpp`、`learning/policy.hpp` | `tests/test_learn.cpp`：手算对照 + 图梯度 vs 中央差分吻合（`loss-grad: analytic=-0.094902 numeric=-0.094891`） | ✅ 成立（数值标"量级参考"，硬门槛是逐元素容差） |
| REINFORCE baseline 与 advantage 同量纲（折扣回报滑动平均·EMA）（§3 表） | `learning/reinforce.hpp:76-85`（`bDecay=0.95`，首用批均值预热） | `tests/test_learn.cpp`（一次更新手算对照） | ✅ 成立 |
| 训练曲线"BC loss 0.34 → 0.06（60 epoch）"（§3 训练验证） | `learning/trainer_main.cpp`（BC→DAgger→REINFORCE） | **本轮新增** `tests/test_train_regression.cpp`（seed=7、100 epoch、4 示教：0.548→0.156，首段均值 0.427→末段 0.185）+ `arm_train` 同种子两次运行 **CSV 与 `policy_final.bin` 逐字节一致** | ⚠️→✅ 已修：README 标注"量级参考"+复现命令；硬门槛改为"末值 <0.5×初始 + 同种子逐位一致" |
| REINFORCE 长程（400 步）"改善显著" | **未声称**（如实边界） | `tests/test_train_regression.cpp` 短程 12 回合断言"改善**不显著**（Δ=184、合并 σ=320，方差主导）"——若未来显著，测试会 FAIL 提醒更新文档 | ✅ 边界如实 |

## 4. 运维边界（配套 docs/SECURITY.md）

| 声称（README 位置） | 实现 | 证据 | 结论 |
| --- | --- | --- | --- |
| "默认面向实验室/局域网（无认证）"（§2 安全边界） | 无认证/无 TLS/无速率限制 | `docs/SECURITY.md`：威胁模型 + 未实现防护清单及后果 + 部署建议 + 已实现防护（含开关/默认值） | ✅ 成立（D2 交付） |
| 未实现防护的后果可判定（本轮新增） | 同上 | 同上 | ✅ 新增 |

## 4.5 真机路径 / 稳定性（批次 E–H 新增）

| 声称（README/PR 位置） | 实现 | 证据（仓内可复现） | 结论 |
| --- | --- | --- | --- |
| 「串口帧协议 / CRC 校验在测试中回环验证」 | `serial_driver.hpp` | `tests/test_serial.cpp`（协议层）+ `tests/serial_loopback.cpp`（**PTY 端到端**：往返数值、连续 20 帧无丢失、坏 CRC 拒收且链路自愈、分片重组、急停通道） | ✅ 成立（从「编解码回环」升级为**完整 IO 闭环**） |
| 真机写路径可靠性（原实现 `::write` 返回值丢弃、`txBuf_` 无限增长） | 有界待发队列 + 部分写续传 + EAGAIN 重试 + 硬错误离线 | `test_serial`：部分写（每次 3 B）单拍内 6 次续传后 18 B 完整；EAGAIN 首拍 0 B → 次拍补齐；EIO → `online()=false`、在途与后续帧计入 `droppedOffline`；**E2 回归：50 Hz × 1 小时（180000 帧）队列峰值 31 B**（旧实现推算 +5.6 MB/h 且无上限） | ✅ 已修（并有回归） |
| 命令/状态方向（原实现为示教器语义，与「真机=控制上位机」相反） | `Role::{Master,Pendant}` + `--serial-role`；Master 下发 `CMD_*`、解析 `STATE_REP` 至 `/api/health.serial_*` | `test_serial`（角色段落）+ `serial_loopback`（Master↔从机往返） | ✅ 已修正（语义显式，非隐式假设） |
| 「真机路径未验证」（原 `SECURITY.md` 边界声明） | — | PTY 闭环只覆盖**协议与软件路径**；电气层/波特率容差/下位机固件仍未验证 | ⚠️ 边界更新：软件路径已闭环验证，**硬件仍未知** |
| 策略机制（`logStd`/熵项/裁剪/白化）此前无测试 | `learning/policy.hpp`、`reinforce.hpp` | `tests/test_policy_mechanics.cpp`：`logStd` 梯度 = −β·B（偏差 0）、ΔlogStd=1.0e-2（=lr）、裁剪后范数 1.000000001（=阈值）、白化 postMean=1.2e-9/postStd=1.0000、零方差批 floor 分支 postMean=0 | ✅ 覆盖补齐 |
| 短程 REINFORCE「改善不显著」 | 稳健口径（F2） | `--seed-scan`：5 种子 = 显著改善 1/5、不显著 4/5（Δ/σ = +2.80, +0.57, +0.81, +0.29, +0.09）→ 旧口径会随种子翻转；默认改用「不劣化界限 + σ 同量级」断言 | ⚠️→✅ 口径修正并如实说明局限 |
| 长时稳定性（无覆盖） | — | `tests/test_longrun.cpp`：**1000 万拍 = 20000 s（5.56 h 仿真）**：偏移包络 8 桶恒为 1.7e-3 rad（不随时间增长 → 非漂移）、时间基相对误差 1.6e-10、安全层计数 0、t≈20000 s 处轨迹播放终点偏差 1.1e-16 | ✅ 新增（结论为「有界慢收敛/粘滑」而非「零偏移」） |
| 长时运行内存（G1） | 示教点上限 4096 + 串口队列上限 + 静态缓存上限 | `tests/soak_longrun.py` 三段各 12 min（CSV 入仓 `docs/soak-longrun-20260926.csv`）：A 无客户端 1896→1896 KB；B 1 客户端 1896→1896 KB；C 4 客户端+压测 1896→**4284 KB 一次性阶跃**后 11 min 恒定（期间 66898 次操作、teach 触顶 4096）→ 高水位而非泄漏 | ✅ 新增（并修掉 teach_ 无上限增长路径） |
| 「慢客户端只被丢弃」的量化（G1 顺带） | 发送队列 256 KiB + 内核 socket 缓冲 | 不读取客户端实测 **132.5 s** 被丢弃；持续读取（真实浏览器语义）则长期稳定（soak 中 `ws_clients=1` 保持 12 min 无掉落） | ✅ 量化 + 压测语义修正（harness 曾因不读帧而测到 churn） |

## 5. 本轮 README 修正清单

1. §2 表 + §6：串口协议 `SOF A5/序号/CRC16-IBM` → `[0xAA][0x55][type][len][payload][crc16]`、无序号、CRC16-CCITT（依据 `serial_driver.hpp:3-11,38-55`）。
2. §4 末："六处补丁" → "七类补丁（`ctorch-lyra.patch` 12 文件 + `c3-lyra.patch` 2 文件）"（依据 `PATCHES.md` 7 条 + 补丁文件清单）。
3. §1 说明：η 分布/阈值占比数字 → 仓内 `--eta-stats`（seed=20260926）复现值，并给出复现命令。
4. §3 训练验证：BC/loss-grad 数字 → 标注"量级参考"+ 复现命令（`test_train_regression.cpp`）。
5. 测试节：Layer 0 项数 5 → 6，并说明 `run_tests.sh` 三层汇总与 SKIPPED/FAIL 语义。
6. §2 表：新增"静态文件 192 KiB 上限 + 内存缓存"与 Runtime 探针说明。

## 6. 已知未验证 / 不作为强断言的项（诚实边界）

- **真机（串口/termios）**：`SerialDriver` 真机路径仅留桩，未在真串口验证；回环测试只覆盖编解码与状态机。
- **长时间运行**：无 24 h 级 soak 测试；`tests/bench_ws.cpp` 为分钟级压测。
- **学习层泛化能力**：短程 REINFORCE 的改善在统计上不显著（方差主导），文档如实声明，不承诺收敛性。
- **多客户端并发下的端到端延迟**：Runtime 探针覆盖资源上限行为，未做网络抖动/丢包注入。
- **CTorch 上游**：补丁随上游版本漂移需重放；未做上游回归测试对比（仅本仓测试）。
