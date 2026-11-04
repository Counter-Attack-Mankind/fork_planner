# forklift_nlp_recovery 启动与案例说明

本包负责单车及三车联合 NLP 死锁恢复、独立安全验证、案例保存和 RViz 展示。离线入口只读取冻结案例并调用现有优化接口，不启动多车仿真，也不会向底盘或在线协调节点发布执行命令。

## 目录

- `case/<编号>/scene.yaml`：在线进入有效 `NLP_PREP` 后自动保存的冻结现场。编号采用原子目录创建，进程重启后继续递增。
- `results/<案例编号>/<求解编号>/`：该案例每次在线或离线求解的输入、实际初猜、轨迹、求解报告及 `run_status.yaml`。每次求解创建新目录，不覆盖旧记录。
- `src/visualization`：案例读写、统一离线入口和回放控制。
- `src/optimization`：STC、初猜、单车及三车 NLP 数学模型与验证。

案例使用 `schema_version: 1`。场景保存车辆真实 ID、冻结后轴位姿、路径及进度、`path_gen`、模式、任务阶段、转角和参与车辆集合；地图、货架及默认车辆参数继续从现有 YAML 配置读取。

## 在线运行与自动保存

正常多车仿真及 NLP 恢复：

```bash
roslaunch forklift_nlp_recovery recovery_debug.launch
```

进入完整且静止的 `NLP_PREP` 冻结事务后自动创建一个案例。同一 `recovery_id` 的重复快照不会重复建案例。STC、初猜、IPOPT 或验证失败时仍保留已产生的数据和明确失败阶段；不会伪造缺失的轨迹。

## 统一离线入口

读取案例 1、重新求解并打开 RViz：

```bash
roslaunch forklift_nlp_recovery nlp_offline_solve.launch case:=1
```

求解使用案例中的原始冻结状态：恰好三辆参与车时直接调用三车联合求解器，其他数量保持现有单车候选流程。成功且独立验证通过后自动播放新轨迹；失败时保留静态现场及失败状态，不播放失败轨迹。

查看案例 1 的第 2 次历史求解，不重新求解：

```bash
roslaunch forklift_nlp_recovery nlp_offline_solve.launch case:=1 mode:=replay run:=2
```

回放支持旧单车逐节点 AABB，也支持三车完整 STC（轨迹节点 `aabb_index=-1`）。历史 IPOPT 失败结果只要保存了完整轨迹和报告，也可通过 replay 模式检查。可用参数还包括 `playback_rate`、`start_rviz`、`case_root` 和 `results_root`。

原 `nlp_replay.launch` 暂时保留用于兼容已有脚本；新调用统一使用 `nlp_offline_solve.launch`。

## 其他启动文件

- `coordinate_mapping.launch`：坐标映射及路径几何检查。
- `recovery_debug.launch`：在线多车仿真、自动 NLP 恢复和恢复专用 RViz。

离线 RViz 使用 `/forklift_nlp_recovery/markers` 显示冻结现场、STC、起终点及求解状态，使用 `/forklift_nlp_recovery/offline_playback_markers` 播放本次成功结果，replay 模式使用 `/forklift_nlp_recovery/replay_markers`。
