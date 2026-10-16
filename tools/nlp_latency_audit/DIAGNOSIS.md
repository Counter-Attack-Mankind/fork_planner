# 多叉车死锁至 NLP 恢复运动端到端耗时诊断

## 结论

在 WSL Ubuntu 20.04 / ROS Noetic 中按当前代码、四车、seed=2022、
`simulation_speed=5.0`、无 RViz 完成了一次端到端复现。最终事务为
`recovery_id=13`，参与车辆 `V0/V1/V3`，求解与独立验证均通过，规划器接收
轨迹后首次车辆状态变化也已直接观测到。

本次复现没有出现“进入 NLP 后额外等待约 40 秒”的现象：

- `NLP_PREP -> 首次车辆运动` 为 **7.565865 s wall**。
- 其中 `初始解保存 -> solver 完成日志` 的完整求解包络为 **7.533907 s**。
- CasADi `solver(arguments)` 的 `t_wall_total` 为 **7.499033 s**，占上述包络
  **99.54%**。
- 结果保存 **6.634 ms**，保存后到 trajectory topic 被审计器收到
  **0.438 ms**，topic 到规划器接收 **0.449 ms**，规划器接收到首次状态变化
  **2.266 ms**。

因此，当前证据表明 NLP_PREP 之后的主耗时是数值求解调用本身，不是 AABB、
初始解、磁盘保存、ROS 传输、timer 或 RViz。原先约 50 秒若确为真实墙钟，
不能由当前 seed=2022 事务复现，必须用同一审计器对原运行再次采集；不能用
`50 - 9.51` 倒推缺失阶段。

## 实测阶段表

| 阶段 | Wall Time | 证据与准确性 |
| --- | ---: | --- |
| 最终三车候选持续到确认 | `UNKNOWN`（外部观测下界 0.566 s） | 诊断文件直接给出 `sim_t=2411.6 -> 2415.5`，即 **3.9 s simulation time**；候选墙钟只能在文件刷新后观察，不能当精确值 |
| 确认到 NLP_PREP | 0.172 ms | rosout ROS 时间，`/use_sim_time=false` |
| NLP_PREP 到全局冻结 | 1.144 ms | rosout ROS 时间 |
| NLP_PREP 到 Snapshot 发布/收到 | 1.405 ms | Snapshot header 时间 |
| 输入/AABB + 初始解生成 | 6.715 ms | Snapshot header 到 initial-guess summary；两者无法外部分拆 |
| 初始解文件保存 | 14.052 ms | initial-guess summary 到 saved log |
| CasADi 建模 + nlpsol + 数值调用 + 提取 + 独立验证 | 7.533907 s | initial-guess saved 到 solver summary 的外部包络 |
| CasADi `solver(arguments)` | 7.499033 s | `solver_report.yaml:t_wall_total` |
| IPOPT 内部计时 | 4.720 s | IPOPT 控制台 `Total seconds in IPOPT`；没有独立开始/结束时间戳 |
| 结果保存 | 6.634 ms | solver summary 到 result-saved log |
| 保存后到 trajectory topic 收到 | 0.438 ms | rosout + 只读 topic subscriber |
| trajectory topic 到规划器接收 | 0.449 ms | 只读 topic subscriber + `[NLP_EXEC]` ROS 时间 |
| 规划器接收到首次车辆运动 | 2.266 ms | `[NLP_EXEC]` 到下一份发生位姿/速度变化的 Snapshot header |
| NLP_PREP 到首次车辆运动 | **7.565865 s** | 直接墙钟兼容 ROS 时间 |
| 候选到首次车辆运动 | 精确值 `UNKNOWN`；观测下界约 **8.133 s** | 候选文件刷新使墙钟起点偏晚；仿真候选确认阈值为 3.9 s |

完整自动生成表位于
[`runs/20261015T070013Z_seed2022/latency_report.md`](runs/20261015T070013Z_seed2022/latency_report.md)。

## 50 秒期间实际发生了什么

### 本次最终事务没有固定回退重试

最终死锁是 `V0 -> V1 -> V3 -> V0` 三车等待环。`DeadlockManager` 在候选持续
达到 `deadlock_confirm_time=4.0 s` 后，对 `cycle.size() >= 3` 直接进入
`NLP_PREP`。诊断日志中确认和 NLP_PREP 都发生在 `sim_t=2415.5 s`，不存在
RETREAT、RETREAT_HOLD、PASS 或三轮退避。

两车事务的代码路径确实不同：固定回退最多三次，每次回退完成后有一个
`rolling_refresh_period=2.0 s` 的 RETREAT_HOLD，PASS 成功还要求一辆车连续
活动 `2 * rolling_refresh_period=4.0 s`。但这套多轮逻辑不是本次最终三车
事务的耗时来源。运行中较早的两车确认死锁被该机制正常解除，它们是独立
事务，也不应合并进 recovery_id=13。

最终事务出现前，V3 从 `sim_t=2393.6 s` 开始 YIELD/CREEP，直到
`sim_t=2407.6 s` 才首次进入 STOP；三车循环候选在 `2411.6 s` 才成立。
这说明界面上的“某车正在等待”早于严格的“死锁候选”。即便从最早的
YIELD/CREEP 算起，到 NLP_PREP 也只有 21.9 s simulation time；以本次实测
约 5.0x 仿真倍率折合约 4.4 s wall，再加 7.57 s NLP_PREP 后耗时，仍远低于
50 s wall。

### 四种时间不能混用

- **Wall/monotonic time**：本报告端到端延迟的主时间基准。
- **ROS time**：本次 `/use_sim_time=false`，所以和系统墙钟兼容；不是仿真累计
  时间。
- **Simulation time (`sim_t`)**：正常仿真以约 5.0x wall 推进；全局冻结等待
  求解时不推进，NLP 执行时每个 10 Hz wall timer 回调推进 5 个 0.1 s 步。
- **NLP trajectory time (`T=11.753496481 s`)**：优化轨迹的时间轴，不是求解
  延迟，也不是运动前等待。以 5.0x 推进时，完整轨迹理论上约 2.35 s wall
  播放，但首次运动在接收后约 2.3 ms 已发生。

因此，RViz 首次肉眼看到明显位移、车辆从更早的 YIELD/CREEP 开始等待、
`sim_t`、NLP 的 `T` 和 wall time 都不是同一个起点/时间轴。

## 调用链与阻塞关系

1. `DeadlockManager::update()` 累加候选仿真时长；候选有位姿进展会重置。
2. 三车等待环确认后同一仿真步进入 `NLP_PREP`。
3. 规划器下一处检查立即 `latchNlpRecoveryFreeze()`，清零全部车辆速度，并在
   每个 10 Hz wall timer 末尾发布一次 RecoverySnapshot。
4. `recovery_visualizer_node` 使用单线程 `ros::spin()`。Snapshot 回调内同步串行
   执行 `OptimizationInputBuilder`（包含 CorridorBuilder）、debug input 写盘、
   InitialGuessGenerator、case 写盘、`NlpSolver::solve()`、独立验证和结果写盘。
5. `solveAndSave()` 返回后才执行 `submitLockedTrajectory()`。所以保存结构上会
   阻塞发布，但本次仅 6.6 ms，不是瓶颈。
6. 规划器是另一个进程，恢复节点求解时其 timer 仍运行并持续发布冻结快照。
   规划器在 `nlpTrajectoryCallback()` 内立即置
   `nlp_execution_active_=true`；下一次 timer 执行
   `executeNlpRecovery()`。本次轨迹恰好在下一 timer 到期前到达，因此约
   2.3 ms 后已经更新状态。

相关代码位置：

- `forklift_planner/src/multi_vehicle/deadlock/deadlock_manager.cpp`：候选确认
  519-528；RETREAT_HOLD/PASS/retry 308-408。
- `forklift_planner/src/multi_vehicle_patrol_node.cpp`：冻结 2485-2505；Snapshot
  2586-2625；轨迹接收 2732-2750；执行 2820-2905；timer 与
  `simulation_step_credit_` 3007-3051。
- `forklift_nlp_recovery/src/recovery_visualizer_node.cpp`：提交 535-577；同步保存
  683-696；auto 回调链 787-803；`ros::spin()` 940。
- `forklift_nlp_recovery/src/nlp_solver.cpp`：`nlpsol()` 376；数值调用 384；
  `t_wall_total` 390；独立验证 430；结果保存 453 以后。

## 求解性能判断

本次问题与用户提供的实例具有相同的 `N=30`、`T=11.753496481 s`、70 次
迭代，且均验证 PASS；当前求解器输出为 601 个变量、435 个等式约束、531 个
不等式约束。当前 run 的 CasADi wall time 是 7.499 s，而已有 run 是
9.5085 s，相差约 2.01 s。迭代数和最优轨迹时间完全一致，说明差异更像 WSL
调度、CPU 负载/频率或缓存状态导致的运行时波动，而不是不同优化路径；要证实
仍需重复多次并同时记录系统负载。

存在一个已证实、值得继续拆分的求解侧区间：CasADi `t_wall_total=7.499 s`，
而 IPOPT 自身报告 4.720 s，差值 **2.779 s（CasADi 数值调用的 37.1%）**。
该差值在 `solver(arguments)` 内，但当前证据无法再归因到函数/Jacobian/Hessian
求值、CasADi 包装、线性求解器或调度等待。符号建模、`nlpsol()` 初始化、解提取
和独立验证的总外部余量只有约 35 ms，不是明显瓶颈。

## 按证据排序的建议（本轮未实施）

1. **优先细分 `solver(arguments)` 内的 2.779 s 差值。** 保存完整 CasADi
   `solver.stats()` 计时字段，并对恢复进程做一次 `perf stat`/`pidstat` 采样；
   这是唯一已测得的数秒级非 IPOPT 内部区间。
2. **重复同一 seed 5-10 次。** 记录 `t_wall_total`、IPOPT internal seconds、
   CPU time、CPU frequency/steal/load，判断 7.50 到 9.51 s 的波动是否来自 WSL
   调度。当前单次结果不足以宣称算法性能回归。
3. **若 50 s 再现，必须以 recovery_id 同时采集 Topic 和 rosout。** 特别确认
   观察起点是 YIELD/CREEP、严格 CANDIDATE、NLP_PREP，还是 RViz 明显位移。
   当前完整复现已经排除 snapshot、AABB、写盘、ROS 传输和 planner timer 为
   数十秒瓶颈。
4. **没有证据支持优先改文件保存或 ROS 发布。** 它们总共仅毫秒级。虽然当前
   顺序是保存后发布，但直接调整顺序的收益上限约 6.6 ms。
5. **单独测量建模/初始化/验证仅用于精细画像。** 现有外部包络已将它们合计
   限定在约 35 ms；若需要精确值，可加仅记录 `steady_clock` 的临时埋点，但
   本轮没有修改业务源文件。

## 验证与产物

- WSL ROS Noetic 四车 seed=2022 无头端到端运行：PASS（求解、独立验证、提交、
  接收、首次运动均观测到）。
- 审计脚本 Python 语法、shell `bash -n`、`git diff --check`：PASS。
- 此前当前工作树四个 Catkin 包编译通过，CTest 15/15、88 tests、0 failures；
  本轮未改任何 `.cpp`、`.h`、`.msg`、既有 launch 或 YAML。

原始证据目录：[`runs/20261015T070013Z_seed2022`](runs/20261015T070013Z_seed2022)。
