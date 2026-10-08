# 全仓库代码轻量化只读审计

审计日期：2026-10-08  
审计范围：`D:\desktop\叉车\src` 当前工作区中除 `.git` 内部对象外的全部现存文件，并补充登记 Git 索引中已被用户删除、当前无法读取的文件。  
审计性质：静态、只读取证；本报告是本轮唯一新增文件，未删除或修改任何现有功能代码、配置、launch、测试或日志。

## 1. 分类口径与结论摘要

- **KEEP**：正式主链、公共接口、安全链或被正式/实验运行入口直接消费；当前不应删除。
- **OPTIONAL**：不参与正式运行，但作为诊断、回归、可视化、示例或开发配置有明确价值；可移出最小部署包，不建议从开发仓库直接删除。
- **REVIEW**：存在实验边界、默认关闭、只写不读、未接入构建、内部无调用、重复实现或 deprecated 证据；删除前必须由负责人确认并完成指定回归。
- **REMOVE**：仓库证据表明是生成物、聊天/临时文件、未被构建或引用的归档，或整组无引用的遗留副本；仍建议通过独立清理提交执行，便于回滚。

核心结论：

1. 正式仿真最小主链不能简单按目录裁剪，至少包括 `forklift_map`、`forklift_planner` 和公共消息包 `experiment/sandbox_msgs`。
2. `experiment` 不能整体删除。`pure_pursuit`、`lqr_controller`、`nokov_localization_v2.py`、`chassis`、`vrpn_client_ros` 被实车 launch 串联，但当前环境的完整实车运行验证为：**未知，需要确认**。其中 `chassis/CATKIN_IGNORE` 与 `vrpn_client_ros/CATKIN_IGNORE` 是明确的环境隔离措施，必须保留。
3. 第一批低风险清理候选是：根目录 `1.txt`、两个已提交 `.pyc`、`experiment/third_party/tinyproto.zip`，以及仓库内无任何消费者的 `forklift_map/include/forklift_map/common/math/*`、`common/util/*` 副本。后者删除前仍需做一次干净构建，防止仓外消费者依赖导出的头文件。
4. `TaskAllocator::containsRecent()`、`deterministicJitter()`、`activeTargetCount()` 只有声明和定义，没有调用；`recent_targets/recent_rows` 目前只写不读，因此 `recent_target_memory`、`recent_row_memory` 不影响任务选择。不能把它们描述成已经生效的“去重复选择”策略。
5. `RuleEngine::applyFollowingSuggestions()` 只有声明和定义，没有调用。`arbitrateResources()` 也未执行：`RuleEngine::decide()` 中调用被注释停用；但资源地图/跨度仍用于诊断和冲突 marker，不能直接整包删除。
6. A2 两个 route 源文件几乎逐行重复，仅函数入口名/方向不同；它们被 CMake、route dispatch 和路径目录调试入口编译调用，但只属于下一阶段能力，分类为 REVIEW，禁止接入正式业务。
7. 当前已有两套长时运行基础能力：无头快速 `runBatch()`，以及实时 marker 的 rolling rosbag + RViz 回放。二者尚未闭环：batch 不发布 marker，`multi_vehicle_batch.launch` 也不启动 recorder；所以 batch 中即使触发 snapshot topic，也没有可回放的可视化数据。

## 2. 取证方法、边界与限制

静态取证覆盖：

- 递归文件清单、Git 跟踪/未跟踪/删除状态；
- 所有 `CMakeLists.txt`、`package.xml`、launch、YAML、RViz 配置；
- C++ include/定义/引用、Python import/launch 入口；
- ROS publisher/subscriber/topic、参数声明与读取点；
- 仿真入口 `planner_node`、`two_vehicle_sim_node`、`multi_vehicle_patrol_node`、batch；
- 实车实验入口 `realbridge_exp.launch`、`realbridge_a1_cycle.launch`、控制器启动器、定位与底盘；
- 测试/诊断 target 是否实际接入 CMake。

限制：当前 Windows 审计环境未发现 `catkin_make` 或 `roscore`，因此没有执行 ROS 构建、launch 或运行回归。这里的 KEEP/REMOVE 是静态证据分级，不等于运行验收结论。动态加载、仓外依赖和目标实车环境状态均为：**未知，需要确认**。

工作区在审计前已存在 `planner_param.yaml` 修改、历史日志删除和未跟踪 `1.txt`；本轮全部原样保留。Git 中标记删除的历史文件当前没有内容可读，见第 8 节。

## 3. 构建与运行入口图

```text
正式仿真：
map_param.yaml + planner_param.yaml
  -> forklift_map/map_node
  -> forklift_planner/multi_vehicle_patrol_node
  -> /forklift_map/markers + /forklift_planner/markers + logs

快速回归：
multi_vehicle_batch.launch / multi_vehicle_phase2_batch.launch
  -> multi_vehicle_patrol_node(batch_minutes > 0)
  -> runBatch() 固定 dt 紧循环，不创建实时 timer，不发布正常 marker

实车实验：
VRPN -> nokov_localization_v2.py -> /object
  -> multi_vehicle_patrol_node(real_mode=true)
  -> /traj_<id> + /coord_speed_<id> + /coord_state_<id>
  -> pure_pursuit 或 lqr_controller -> /chassis -> chassis_node
```

关键证据：`forklift_planner/CMakeLists.txt`、`forklift_planner/src/multi_vehicle_patrol_node.cpp` 的构造函数、`tick()`、`runBatch()`、`setupRealIO()`，以及 `realbridge_*.launch`。

## 4. 逐文件分类：仓库根、文档与公共接口

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `.vscode/settings.json` | OPTIONAL | 仅编辑器工作区配置，不参与 CMake/ROS | 最小源码包可排除；团队若共享编辑器设置则保留 |
| `AGENTS.md` | KEEP | 项目级安全、架构、验证和 deprecated 约束 | 删除会失去正式边界和 AI/开发约束，禁止删 |
| `docs/PROJECT_OVERVIEW.md` | KEEP | 正式架构说明；与主链代码互相取证 | 存在 `single_vehicle_patrol.launch` 已不存在等陈旧描述，后续应修文档，不应直接删除 |
| `docs/code_audit.md` | KEEP | 本轮审计产物 | 后续清理提交应更新状态和验证证据 |
| `1.txt` | REMOVE | 未跟踪；内容是本次用户请求副本，无代码/构建/launch 引用 | 无功能风险；确认不作为个人备忘后删除 |
| `experiment/sandbox_msgs/CMakeLists.txt` | KEEP | `add_message_files()`/`generate_messages()`；planner 与控制器直接依赖 | 公共接口生成入口，禁止删 |
| `experiment/sandbox_msgs/package.xml` | KEEP | 声明 message generation/runtime 与消息依赖 | 删除破坏 catkin 包 |
| `experiment/sandbox_msgs/msg/AprilObject.msg` | KEEP | 定位发布，planner/控制器订阅 | 公共接口；字段变化须全链回归 |
| `experiment/sandbox_msgs/msg/Trajectory.msg` | KEEP | planner 发布，控制器订阅 | 同上 |
| `experiment/sandbox_msgs/msg/TrajectoryPoint.msg` | KEEP | `Trajectory` 元素，planner/控制器共同使用 | 同上 |
| `experiment/sandbox_msgs/msg/ChassisCommand.msg` | KEEP | 控制器/键盘发布，底盘订阅 | 同上 |

## 5. 逐文件分类：实车适配实验模块

### 5.1 chassis

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `experiment/chassis/CATKIN_IGNORE` | KEEP | AGENTS.md 明确指定为当前虚拟机环境隔离措施 | 不得为全量编译擅删 |
| `experiment/chassis/CMakeLists.txt` | KEEP | 构建 `chassis_node` 与 `steering_limit_calibration_node` | 实车实验入口依赖；虽被当前环境隔离，不能据此判无用 |
| `experiment/chassis/package.xml` | KEEP | catkin 包清单 | 删除破坏目标实车环境构建 |
| `experiment/chassis/config/vehicle_chassis_gains.yaml` | KEEP | `realbridge_*.launch` 与校准 launch 加载 | 安全/控制参数，禁止无实验记录修改 |
| `experiment/chassis/launch/steering_limit_calibration.launch` | OPTIONAL | 直接启动定位、底盘与校准节点 | 非正式仿真；保留作受控标定工具 |
| `experiment/chassis/src/chassis_node.cpp` | KEEP | `realbridge_*.launch` 启动；订阅 `/chassis`，串口输出 | 实车底盘链；删除风险高，目标环境验证未知 |
| `experiment/chassis/src/steering_limit_calibration_node.cpp` | OPTIONAL | 由标定 launch 直接启动 | 安全阈值标定工具；仅在负责人批准的实验中使用 |
| `experiment/chassis/scripts/keyop.py` | REVIEW | 仓内无 launch/CMake 安装引用，发布 `/chassis`；Git 模式也非 executable | 可能是人工 rosrun 工具；确认操作手册/外部使用后决定迁移到 tools 或删除 |
| `experiment/chassis/main.ino` | REVIEW | Arduino 固件源；无 CMake/launch 引用 | 可能属于底盘 MCU 仓外烧录链，未知，需要确认；不应仅凭 ROS 无引用删除 |

### 5.2 lqr_controller

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `experiment/lqr_controller/CMakeLists.txt` | KEEP | 构建 `lqr_controller_node`；`one_controller.launch` 可选择它 | 实车实验控制器入口 |
| `experiment/lqr_controller/package.xml` | KEEP | catkin/Eigen/消息依赖 | 删除破坏构建 |
| `experiment/lqr_controller/include/lqr_controller.h` | KEEP | `lqr_controller.cpp` 与 node 使用 | 控制算法接口 |
| `experiment/lqr_controller/include/planning_core/datatypes.h` | KEEP | LQR 数据类型依赖 | 与控制器共同编译 |
| `experiment/lqr_controller/include/planning_core/frenet_frame.h` | KEEP | `lqr_controller` 使用 | 路径坐标转换 |
| `experiment/lqr_controller/include/planning_core/math_utils.h` | KEEP | `frenet_frame.cpp` 使用 `normalize_angle` | 直接依赖 |
| `experiment/lqr_controller/include/planning_core/numpy.h` | KEEP | `frenet_frame.h` 使用向量工具 | 直接依赖 |
| `experiment/lqr_controller/include/planning_core/spline.h` | KEEP | `frenet_frame`/`spline.cpp` 使用 | 直接依赖 |
| `experiment/lqr_controller/src/frenet_frame.cpp` | KEEP | CMake target 源文件 | 删除导致链接失败 |
| `experiment/lqr_controller/src/lqr_controller.cpp` | KEEP | CMake target 源文件 | 同上 |
| `experiment/lqr_controller/src/lqr_controller_node.cpp` | KEEP | ROS node 主入口；`one_controller.launch` 调用 | 同上 |
| `experiment/lqr_controller/src/spline.cpp` | KEEP | CMake target 源文件 | 同上 |

### 5.3 nokov_localization

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `experiment/nokov_localization/CMakeLists.txt` | REVIEW | 构建并导出 `nokov_localization` C++ 库，但仓内无消费者；实际 launch 调 Python v2 | 仓外消费者未知；若确认无外部链接，可连同 C++ subscriber 与 nlohmann vendoring 删除/拆包 |
| `experiment/nokov_localization/package.xml` | KEEP | Python v2 定位脚本以该 ROS 包被 launch 查找 | 包仍是实车实验定位入口 |
| `experiment/nokov_localization/include/nokov_localization/subscriber.h` | REVIEW | 只被对应 `.cpp` 引用；导出库仓内无人实例化 | 与 CMake/export 一并决策 |
| `experiment/nokov_localization/src/nokov_localization/subscriber.cpp` | REVIEW | 仅解析 `/nokov_info` JSON，`boundary` 分支仍是 TODO；无仓内调用 | 很可能是未完成旧方案；确认仓外 ABI 后删除 |
| `experiment/nokov_localization/script/nokov_localization_v2.py` | KEEP | 三个实车/标定 launch 直接启动；发布 `/object` | 当前实车实验定位入口 |
| `experiment/nokov_localization/script/nokov_localization.py` | REVIEW | 旧定位实现；无 launch 引用；与 v2 功能重叠 | 先对照目标 Nokov topic 和标定流程，确认 v2 完全替代后删除 |
| `experiment/nokov_localization/script/vehicle_param.py` | KEEP | 定位脚本导入车辆参数 | 删除会造成 Python import 失败 |
| `experiment/nokov_localization/script/visualization.py` | KEEP | v1/v2 与可视化工具导入 | 实验链共享工具 |
| `experiment/nokov_localization/script/bag_visualizer.py` | OPTIONAL | 独立订阅 `/traj` 的离线/调试可视化，无 launch | 可移入 tools；保留对实车 bag 排查有价值 |
| `experiment/nokov_localization/script/footprint_visualizer.py` | OPTIONAL | 独立订阅 `/chassis`、`/object`，无 launch | 同上 |
| `experiment/nokov_localization/script/__pycache__/vehicle_param.cpython-38.pyc` | REMOVE | Python 生成物；源码存在 | 删除无功能风险，应加入 `.gitignore` |
| `experiment/nokov_localization/script/__pycache__/visualization.cpython-38.pyc` | REMOVE | Python 生成物；源码存在 | 同上 |

`experiment/nokov_localization/thirdparty/nlohmann/` 的每个文件逐项登记如下。它们共同只由未被仓内实例化的 C++ subscriber 库消费，统一分类 **REVIEW**；若保留该导出库则全部 KEEP，若移除该库则整组 REMOVE，不能零散删除：

| 文件 | 分类 | 依据/建议 |
|---|---|---|
| `adl_serializer.hpp` | REVIEW | nlohmann JSON vendored 组成文件；随 C++ subscriber 整组决策 |
| `byte_container_with_subtype.hpp` | REVIEW | 同上 |
| `json.hpp` | REVIEW | `subscriber.cpp` 的直接 include；单头入口 |
| `json_fwd.hpp` | REVIEW | vendored 依赖 |
| `ordered_map.hpp` | REVIEW | vendored 依赖 |
| `detail/conversions/from_json.hpp` | REVIEW | vendored 依赖 |
| `detail/conversions/to_chars.hpp` | REVIEW | vendored 依赖 |
| `detail/conversions/to_json.hpp` | REVIEW | vendored 依赖 |
| `detail/exceptions.hpp` | REVIEW | vendored 依赖 |
| `detail/hash.hpp` | REVIEW | vendored 依赖 |
| `detail/input/binary_reader.hpp` | REVIEW | vendored 依赖 |
| `detail/input/input_adapters.hpp` | REVIEW | vendored 依赖 |
| `detail/input/json_sax.hpp` | REVIEW | vendored 依赖 |
| `detail/input/lexer.hpp` | REVIEW | vendored 依赖 |
| `detail/input/parser.hpp` | REVIEW | vendored 依赖 |
| `detail/input/position_t.hpp` | REVIEW | vendored 依赖 |
| `detail/iterators/internal_iterator.hpp` | REVIEW | vendored 依赖 |
| `detail/iterators/iter_impl.hpp` | REVIEW | vendored 依赖 |
| `detail/iterators/iteration_proxy.hpp` | REVIEW | vendored 依赖 |
| `detail/iterators/iterator_traits.hpp` | REVIEW | vendored 依赖 |
| `detail/iterators/json_reverse_iterator.hpp` | REVIEW | vendored 依赖 |
| `detail/iterators/primitive_iterator.hpp` | REVIEW | vendored 依赖 |
| `detail/json_pointer.hpp` | REVIEW | vendored 依赖 |
| `detail/json_ref.hpp` | REVIEW | vendored 依赖 |
| `detail/macro_scope.hpp` | REVIEW | vendored 依赖 |
| `detail/macro_unscope.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/call_std/begin.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/call_std/end.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/cpp_future.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/detected.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/identity_tag.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/is_sax.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/type_traits.hpp` | REVIEW | vendored 依赖 |
| `detail/meta/void_t.hpp` | REVIEW | vendored 依赖 |
| `detail/output/binary_writer.hpp` | REVIEW | vendored 依赖 |
| `detail/output/output_adapters.hpp` | REVIEW | vendored 依赖 |
| `detail/output/serializer.hpp` | REVIEW | vendored 依赖 |
| `detail/string_concat.hpp` | REVIEW | vendored 依赖 |
| `detail/string_escape.hpp` | REVIEW | vendored 依赖 |
| `detail/value_t.hpp` | REVIEW | vendored 依赖 |
| `thirdparty/hedley/hedley.hpp` | REVIEW | vendored 依赖 |
| `thirdparty/hedley/hedley_undef.hpp` | REVIEW | vendored 依赖 |

### 5.4 pure_pursuit

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `experiment/pure_pursuit/CMakeLists.txt` | KEEP | 构建 `pure_pursuit_node`；实车 controller launcher 使用 | 实车实验控制入口 |
| `experiment/pure_pursuit/package.xml` | KEEP | catkin 依赖 | 删除破坏构建/rosrun |
| `experiment/pure_pursuit/config/pure_pursuit.yaml` | KEEP | `one_controller.launch` 加载 | 控制参数，安全相关，禁止无记录修改 |
| `experiment/pure_pursuit/config/vehicle_controller_gains.yaml` | REVIEW | 仓内 launch 未发现加载；可能为旧现场配置 | 与 `pure_pursuit.yaml` 对照并确认目标环境后合并或删除 |
| `experiment/pure_pursuit/src/pure_pursuit.cpp` | KEEP | CMake node 源；订阅轨迹/定位/协调速度并发底盘命令 | 实车实验主链 |
| `experiment/pure_pursuit/launch/online.launch` | REVIEW | 启动 `apriltag_localization`、`chassis`、`online_trajectory_publisher.py`；属于旧独立实验链 | 非正式主链；外部包依赖未知，确认是否仍复现实验后删除/归档 |
| `experiment/pure_pursuit/launch/run.launch` | REVIEW | 引用仓内不存在的 `online_planner` 包 | 当前仓库无法闭环，未知，需要确认 |
| `experiment/pure_pursuit/launch/rviz.rviz` | OPTIONAL | 仅旧 pure-pursuit launch 使用 | 随旧 launch 去留 |
| `experiment/pure_pursuit/script/online_trajectory_publisher.py` | REVIEW | `online.launch` 直接调用 | 旧实验链仍有入口，不能单独删 |
| `experiment/pure_pursuit/script/trajectory_publisher.py` | REVIEW | 无 launch/CMake 引用；导入 Reeds-Shepp | 可能为手工 rosrun 的旧仿真工具 |
| `experiment/pure_pursuit/script/delayed_execute.py` | REVIEW | 无仓内调用 | 疑似一次性实验辅助，确认后删除 |
| `experiment/pure_pursuit/script/spline_test.py` | OPTIONAL | 独立算法试验，不参与运行 | 建议迁至 test/tools 或删除 |
| `experiment/pure_pursuit/script/planner/reeds_shepp_path_planning.py` | REVIEW | 被 `trajectory_publisher.py` 导入；自身也可独立运行 | 随旧 publisher 去留 |
| `experiment/pure_pursuit/script/planner/dubin_path_planning.py` | REVIEW | 仓内无导入者；独立 demo | 确认无手工实验后删除 |
| `experiment/pure_pursuit/__init__.py` | OPTIONAL | 空文件；旧 Python 包标记 | 若仍支持 Python 2/旧 ROS import 则保留 |
| `experiment/pure_pursuit/script/__init__.py` | OPTIONAL | 空包标记 | 同上 |
| `experiment/pure_pursuit/script/planner/__init__.py` | KEEP | `trajectory_publisher.py` 的 `planner.*` import 包边界 | 随旧 publisher 整组决策 |

### 5.5 VRPN 与第三方归档

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `experiment/vrpn_client_ros/CATKIN_IGNORE` | KEEP | AGENTS.md 明确的当前环境隔离措施 | 禁止擅删 |
| `experiment/vrpn_client_ros/CMakeLists.txt` | KEEP | 构建 VRPN library、tracker/client nodes | 实车实验定位上游 |
| `experiment/vrpn_client_ros/package.xml` | KEEP | VRPN/ROS 依赖声明 | 目标环境构建需要 |
| `experiment/vrpn_client_ros/CHANGELOG.rst` | OPTIONAL | 上游组件版本记录 | 可从最小部署排除，不建议从 vendored 包删 |
| `experiment/vrpn_client_ros/include/vrpn_client_ros/vrpn_client_ros.h` | KEEP | 三个 C++ target 直接使用 | 删除破坏构建 |
| `experiment/vrpn_client_ros/src/vrpn_client_ros.cpp` | KEEP | library 实现 | 同上 |
| `experiment/vrpn_client_ros/src/vrpn_client_node.cpp` | KEEP | `sample.launch` 启动 | 同上 |
| `experiment/vrpn_client_ros/src/vrpn_tracker_node.cpp` | OPTIONAL | CMake 构建，但当前 sample launch 使用 client node | 可能由手工 rosrun 使用；确认后可从最小构建裁剪 |
| `experiment/vrpn_client_ros/launch/sample.launch` | KEEP | realbridge 与校准 launch include | 实车实验直接依赖 |
| `experiment/third_party/tinyproto.zip` | REMOVE | CMake 链接系统/工作区中的 `tinyprotocol`，不解压也不引用该 zip | 归档不参与构建；若需保存来源，转 release artifact 并记录版本/哈希 |

## 6. 逐文件分类：forklift_map

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `forklift_map/CMakeLists.txt` | KEEP | 构建/导出 map library、map node、路径目录调试节点 | 核心构建入口 |
| `forklift_map/package.xml` | KEEP | planner 直接依赖该包 | 核心包清单 |
| `forklift_map/config/map_param.yaml` | KEEP | 所有正式仿真/实车 planner launch 加载 | 地图、车辆几何和安全单一真值源之一；阈值变更需批准 |
| `forklift_map/config/forklift.rviz` | KEEP | 正式仿真、debug replay、实车 launch 使用 | 当前统一 RViz 布局 |
| `forklift_map/launch/map.launch` | OPTIONAL | 单车 `planner_node` + map + RViz 入口 | 非正式多车主流程，但有独立调试价值 |
| `forklift_map/launch/path_catalog_debug.launch` | OPTIONAL | 启动路径目录诊断节点和 RViz，覆盖 A1/A2 | 路径回归要求依赖；A2 仅下一阶段诊断 |
| `forklift_map/launch/path_catalog_debug_raw.launch` | OPTIONAL | include 上一 launch 并覆盖调试参数 | 调试预设，可合并但删除前核对使用说明 |
| `forklift_map/include/forklift_map/forklift_map.h` | KEEP | planner/map/诊断节点直接实例化 `ForkliftMap` | 核心地图 API；其中 `set_occupied()`、`free_slots()` 是未使用候选，见第 9 节 |
| `forklift_map/include/forklift_map/map_param.h` | KEEP | 所有地图/规划几何读取 | 核心参数接口 |
| `forklift_map/include/forklift_map/map_types.h` | KEEP | `Slot/ShelfBlock/RoadSegment` 跨包使用 | 核心数据契约 |
| `forklift_map/include/forklift_map/common/clothoid.h` | KEEP | map 与 planner 路径生成共同使用 | 核心几何实现 |
| `forklift_map/include/forklift_map/path_debug_visualizer.h` | OPTIONAL | 仅路径目录调试节点使用 | 诊断组件，随调试 target 去留 |
| `forklift_map/src/clothoid.cpp` | KEEP | `forklift_map_lib` 编译源 | 核心路径几何 |
| `forklift_map/src/forklift_map.cpp` | KEEP | `forklift_map_lib` 编译源 | 核心地图构造 |
| `forklift_map/src/map_node.cpp` | KEEP | 正式 launch 直接启动，发布 `/forklift_map/markers` | 核心可视化输出 |
| `forklift_map/src/path_catalog_debug_node.cpp` | OPTIONAL | CMake/launch 直接接入；路径目录回归工具 | 不进入运行主链，但验证价值高 |
| `forklift_map/src/path_debug_visualizer.cpp` | OPTIONAL | 上一节点直接调用 | 同上 |

以下文件在全仓库中没有任何外部 include；`.cc` 也未列入任何 CMake target。它们构成一套独立复制的旧几何/颜色工具，逐文件分类如下：

| 文件 | 分类 | 依据/风险 |
|---|---|---|
| `include/forklift_map/common/math/aabox2d.h` | REMOVE | 无仓内消费者；删除前干净构建并确认无仓外导出 API 使用 |
| `include/forklift_map/common/math/box2d.h` | REMOVE | 同上 |
| `include/forklift_map/common/math/box2d.cc` | REMOVE | 未由 CMake 编译，也未被 include |
| `include/forklift_map/common/math/line_segment2d.h` | REMOVE | 无仓内消费者 |
| `include/forklift_map/common/math/line_segment2d.cc` | REMOVE | 未编译/未 include |
| `include/forklift_map/common/math/math_utils.h` | REMOVE | 无仓内消费者；大量函数仅声明/自身定义 |
| `include/forklift_map/common/math/pose.h` | REMOVE | 无仓内消费者 |
| `include/forklift_map/common/math/vec2d.h` | REMOVE | 只被本组遗留头引用，无主链消费者 |
| `include/forklift_map/common/util/color.h` | REMOVE | map/node 使用的是本地 ROS ColorRGBA helper，不使用此类 |
| `include/forklift_map/common/util/color.cc` | REMOVE | 未由 CMake 编译 |
| `include/forklift_map/common/util/vector.h` | REMOVE | 无仓内消费者 |

## 7. 逐文件分类：forklift_planner

### 7.1 构建、配置、launch 与脚本

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `forklift_planner/CMakeLists.txt` | KEEP | 全部 planner runtime/diag/test target 的权威入口 | 核心构建文件 |
| `forklift_planner/package.xml` | KEEP | 声明 `forklift_map`、`sandbox_msgs` 等直接依赖 | 核心包清单 |
| `forklift_planner/config/planner_param.yaml` | KEEP | 所有 planner launch 加载 | 当前有用户修改，必须保护；其中 4 个重复/失效键和 2 个只写不读键需 REVIEW，见第 10 节 |
| `launch/multi_vehicle_patrol.launch` | KEEP | 正式多车仿真入口 | 保留 |
| `launch/multi_vehicle_a1_rviz.launch` | KEEP | 正式 A1 可视化仿真，并可启动 rolling bag recorder | 第二部分方案的现有实时入口 |
| `launch/multi_vehicle_batch.launch` | KEEP | `runBatch()` 无头快速回归入口 | 长时测试核心；当前无可视化记录闭环 |
| `launch/multi_vehicle_phase2_batch.launch` | KEEP | 固定 seed/start slots 的可复现 batch | 确定性回归价值高 |
| `launch/multi_vehicle_debug_replay.launch` | KEEP | 启动 snapshot rosbag 慢速/暂停回放和 RViz | 已有故障可视化基础 |
| `launch/path_curvature_audit.launch` | OPTIONAL | 路径曲率离线审计入口 | 路径回归工具 |
| `launch/simulation.launch` | OPTIONAL | 单车 planner + map + RViz，与 `forklift_map/map.launch` 高度重叠 | 建议确认唯一入口后合并；保留兼容风险 |
| `launch/realbridge_exp.launch` | KEEP | 实车实验全链入口之一 | 安全/实车实验，目标环境验证未知 |
| `launch/realbridge_a1_cycle.launch` | KEEP | 正式 B-A1-B 语义的实车实验入口 | 同上 |
| `launch/one_controller.launch` | KEEP | controller launcher include；选择 pure pursuit/LQR | 实车实验直接依赖 |
| `launch/realbridge_keyboard_control.launch` | OPTIONAL | 启动人工 start/estop 工具 | 实车实验操作工具，删除前核对 SOP |
| `scripts/realbridge_controller_launcher.py` | KEEP | 两个 realbridge launch 直接启动，按 vehicle IDs 启控制器 | 实车实验编排入口 |
| `scripts/realbridge_keyboard_control.py` | OPTIONAL | 对应 launch 直接调用 | 实车人工操作工具；安全相关，不应草率删 |
| `scripts/rolling_rviz_bag_recorder.py` | KEEP | A1 RViz launch 可选启动；滚动录 marker，收到 trigger 后归档 | 现有故障前情回放基础 |
| `scripts/replay_rviz_snapshot.py` | KEEP | debug replay launch 直接启动 | 回放基础 |

### 7.2 路径、节点与诊断源文件

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `include/forklift_planner/path_generator.h` | KEEP | 所有路径 target 和 allocator 使用 | 核心 API |
| `include/forklift_planner/path_generator_internal.h` | KEEP | route 实现共享内部函数 | 核心内部 API |
| `include/forklift_planner/planner_param.h` | KEEP | 节点/生成器读取 planner 参数 | 核心配置接口 |
| `include/forklift_planner/common/geometry2d.h` | KEEP | 正式 A1/B route 使用 | 核心几何 helper |
| `src/path_generator.cpp` | KEEP | 所有 planner/diag target 编译 | 核心实现 |
| `src/path_generator_internal.cpp` | KEEP | 同上 | 核心实现 |
| `src/path_generator_route.cpp` | KEEP | 通用/dispatch 路线实现 | 正式路线仍依赖 |
| `src/path_generator_routes/path_generator_route_dispatch.cpp` | KEEP | A1/A2 route mode 分派 | 正式 A1 路线依赖；A2 分支属 planned |
| `src/path_generator_routes/b_to_a1/path_generator_route.cpp` | KEEP | 正式 `B -> A1` 生成 | 正式业务核心 |
| `src/path_generator_routes/a1_to_b/path_generator_route.cpp` | KEEP | 正式 `A1 -> B` 生成 | 正式业务核心 |
| `src/path_generator_routes/b_to_a2/path_generator_route.cpp` | REVIEW | CMake/dispatch/调试使用，但仅下一阶段；与反向文件高度复制 | 冻结，不接入业务；A2 规格确认后再决定抽取共享实现 |
| `src/path_generator_routes/a2_to_b/path_generator_route.cpp` | REVIEW | 同上 | 同上 |
| `src/planner_node.cpp` | OPTIONAL | `simulation.launch`、`map.launch` 启动的单车交互节点 | 不属于正式多车流程；保留作路径交互调试或合并入口 |
| `src/two_vehicle_sim_node.cpp` | OPTIONAL | CMake 构建/安装，但无 launch | 独立两车起步延迟实验；确认是否仍作回归后可移 tools |
| `src/path_debug.cpp` | OPTIONAL | CMake 构建 `path_debug`，命令行路径探针 | AGENTS 要求路径目录诊断，建议保留 |
| `src/path_generator_routes/path_curvature_audit.cpp` | OPTIONAL | CMake + launch 直接接入 | 只读曲率回归工具 |
| `tools/conflict_zone_geometry_diag.cpp` | OPTIONAL | CMake 构建离线冲突区探针 | 协调几何回归工具，不进 runtime |

### 7.3 多车核心

| 文件 | 分类 | 调用/引用证据 | 删除风险与建议 |
|---|---|---|---|
| `include/.../vehicle_agent.h` | KEEP | 正式状态机、动作、路径和诊断数据 | 核心契约；`recent_*` 与 resource spans 子字段需单独 REVIEW |
| `include/.../multi_vehicle_config.h` | KEEP | 全部协调模块共享配置 | 安全参数接口；不得未经批准删改阈值 |
| `src/multi_vehicle/multi_vehicle_config.cpp` | KEEP | 从 ROS 参数加载并规范化配置 | 核心配置读取点 |
| `include/.../path_track.h` | KEEP | agent、碰撞、定位、协调共同使用 | 核心路径弧长表示 |
| `src/multi_vehicle/path_track.cpp` | KEEP | runtime 与多个 test/diag 编译 | 核心实现 |
| `include/.../footprint.h` | KEEP | 路径验证、冲突、安全硬保护共同使用 | 安全核心 |
| `src/multi_vehicle/footprint.cpp` | KEEP | runtime/test/diag 编译 | 安全核心 |
| `include/.../task_allocator.h` | KEEP | patrol node 直接持有/调用 | 正式任务分配；未用私有函数见第 9 节 |
| `src/multi_vehicle/task_allocator.cpp` | KEEP | 正式 B-A1-B 分配/缓存/路径验证 | 核心；deprecated `replanFromPose()` 保留冻结 |
| `include/.../rule_engine.h` | KEEP | patrol node 调用 `decide()` | 正式协调权威入口 |
| `src/multi_vehicle/rule_engine.cpp` | KEEP | CMake runtime/test 编译 | 安全/协调核心；禁止因局部未用函数整文件裁剪 |
| `include/.../marker_publisher.h` | KEEP | patrol node 持有 | 正式可视化与故障回放数据源 |
| `src/multi_vehicle/marker_publisher.cpp` | KEEP | CMake runtime 编译 | 同上 |
| `include/.../spatiotemporal_interaction.h` | KEEP | rule engine、动态速度、deadlock 使用 | 正式预测几何 |
| `src/multi_vehicle/spatiotemporal_interaction.cpp` | KEEP | runtime/test 编译 | 同上 |
| `include/.../dynamic_speed_coordination.h` | KEEP | rule engine 使用 | 当前滚动动态速度协调 |
| `src/multi_vehicle/dynamic_speed_coordination.cpp` | KEEP | runtime/test 编译 | 同上 |
| `include/.../bridge_ttc_correction.h` | KEEP | rule engine 使用 | 当前冲突修正链 |
| `src/multi_vehicle/bridge_ttc_correction.cpp` | KEEP | runtime/test 编译 | 同上 |
| `include/.../a1/a1_coordinator.h` | KEEP | rule engine 直接持有 | 正式 A1 协调；两个未用 accessor 见第 9 节 |
| `src/multi_vehicle/a1/a1_coordinator.cpp` | KEEP | runtime/test 编译 | 正式 A1 协调 |
| `include/.../future_a1_policy.h` | KEEP | A1 coordinator 和测试使用的 header-only 策略 | 正式 A1 预测策略；一个未用格式函数可裁剪 |
| `include/.../deadlock/deadlock_manager.h` | KEEP | rule engine 直接持有 | 当前代码启用的新 deadlock manager；不得与 deprecated 旧策略混同 |
| `src/multi_vehicle/deadlock/deadlock_manager.cpp` | KEEP | runtime 编译且 `deadlock_enabled: true` | 当前运行链；修改需完整协调回归 |
| `include/.../real_state_estimation.h` | KEEP | patrol real mode 与测试使用 | 实车实验状态估计；目标环境验证未知 |
| `include/.../conflict_zone_closure.h` | KEEP | rule engine、diag 与测试使用 | header-only 真实冲突区闭合算法 |
| `include/.../traffic_resource_map.h` | REVIEW | patrol 构造；span/诊断 marker 仍使用，但粗粒度仲裁停用 | 先拆分“诊断几何”和 deprecated 仲裁，再决定能否裁剪 |
| `src/multi_vehicle/traffic_resource_map.cpp` | REVIEW | runtime 构建，生成 resource spans | 同上 |
| `include/.../traffic_resource.h` | REVIEW | agent/rule engine/diagn断引用；token 仲裁主体只服务已停用 `arbitrateResources()`，但 snapshot 仍保存 token | 不可直接删；先删除 deprecated 调用域并更新 snapshot/marker/test |
| `src/multi_vehicle_patrol_node.cpp` | KEEP | 正式仿真、batch、实车实验统一入口 | 绝对核心；已过大（约 191 KB），建议后续按 IO/runner/diagnostics 拆分而非行为重写 |

### 7.4 测试

| 文件 | 分类 | 构建证据 | 建议 |
|---|---|---|---|
| `test/real_state_estimation_test.cpp` | OPTIONAL | CMake `CATKIN_ENABLE_TESTING` 已接入 | 保留；实车估计回归价值高 |
| `test/bridge_ttc_correction_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/conflict_zone_closure_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/future_a1_policy_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/spatiotemporal_interaction_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/dynamic_speed_coordination_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/prediction_execution_consistency_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/rolling_decision_timing_test.cpp` | OPTIONAL | 已接入 CMake/CTest | 保留 |
| `test/deadlock_manager_test.cpp` | REVIEW | 文件存在但 CMake 未添加 target/test | 不应删除测试；应先接入 CMake 并修到可重复运行 |
| `test/dynamic_speed_rule_engine_test.cpp` | REVIEW | 文件存在但 CMake 未添加 target/test | 同上；40 KB 测试不执行会造成虚假覆盖感 |

## 8. Git 中已删除、当前无法逐内容审计的文件

以下删除发生在本轮之前，本轮未恢复也未进一步修改。由于文件当前不存在，只能按路径和 Git 状态分类为 **REVIEW**，内容价值为：**未知，需要确认**。

- `forklift_planner/launch/error.md`
- `forklift_planner/logs/EXP-A1-FIX-20260914/README.md`
- `forklift_planner/logs/EXP-A1-FIX-20260914/coordination.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/debug/forklift_onset.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run2/README.md`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run2/coordination.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run2/debug/forklift_onset.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run3/README.md`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run3/coordination.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run3/debug/forklift_onset.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run4/README.md`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run4/coordination.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run4/debug/forklift_onset.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run5/README.md`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run5/coordination.log`
- `forklift_planner/logs/EXP-A1-FIX-20260914/run5/debug/forklift_onset.log`
- `forklift_planner/logs/EXP-A1-REVIEW-20260914/README.md`
- `forklift_planner/logs/EXP-A1-REVIEW-20260914/coordination.log`
- `forklift_planner/logs/EXP-A1-REVIEW-20260914/debug/forklift_onset.log`
- `forklift_planner/logs/EXP-A1-REVIEW-20260914/screen.log`
- `forklift_planner/logs/EXP-RB-ESTOP-20260924/README.md`
- `forklift_planner/logs/EXP-VEHICLE-IDS-20260924/README.md`
- `log/forklift_onset.log`

历史日志通常不应进入运行时最小包，但 AGENTS.md 要求实验记录不可仅依赖临时位置。建议保留每个 EXP 的 README、参数、commit、命令和汇总结果；大体积逐拍日志可转受版本管理的实验制品存储。是否接受当前这些删除，必须由负责人确认。

## 9. 疑似无用/废弃函数与引用证据

| 定义位置 | 证据 | 分类与建议 |
|---|---|---|
| `forklift_map/include/forklift_map/forklift_map.h:19` `set_occupied()` | 全仓仅定义 1 处，无调用 | REVIEW；若占用状态完全由 `VehicleAgent/TaskAllocator` 管理，可删除 |
| `forklift_map/src/forklift_map.cpp:11` `free_slots()` | 仅头声明 + 源定义，无调用 | REVIEW；与上项一并裁剪 |
| `task_allocator.cpp:781` `containsRecent()` | 仅声明/定义，无调用 | REMOVE 候选；删除前确认未计划恢复近期记忆策略 |
| `task_allocator.cpp:786` `deterministicJitter()` | 仅声明/定义，无调用 | REMOVE 候选；当前确定性来自 `task_rng_` 固定 seed，不来自此函数 |
| `task_allocator.cpp:799` `activeTargetCount()` | 仅声明/定义，无调用 | REMOVE 候选 |
| `rule_engine.cpp:1930` `applyFollowingSuggestions()` | 仅声明/定义，无调用 | REVIEW；属于协调规则，删除前运行完整固定 seed batch |
| `rule_engine.cpp:2126` `arbitrateResources()` | 唯一调用在 `decide()` 中被注释；AGENTS.md 已定为 deprecated 粗粒度资源仲裁 | REVIEW/冻结；负责人批准前不得删除或扩展 |
| `task_allocator.cpp` `replanFromPose()`（声明/实现/兼容调用域） | AGENTS.md 明确为 deprecated 的“当前位置改道到其他库位” | REVIEW/冻结；不得作为新死锁方案；删除需负责人先批准并梳理兼容分支 |
| `a1_coordinator.h:113` `A1Result::a1Related()` | 全仓仅 inline 定义 | REMOVE 候选；结构其余字段仍 KEEP |
| `a1_coordinator.h:116` `A1Result::protectedOwner()` | 全仓仅 inline 定义 | REMOVE 候选 |
| `future_a1_policy.h:169` `futureA1TransitionReason()` | 全仓仅 inline 定义 | REMOVE 候选；确认日志未计划调用 |
| `real_state_estimation.h:163` `sampleCount()` | 全仓仅 inline 定义 | OPTIONAL/REMOVE 候选；测试也未使用 |
| `traffic_resource.h` `heldBy()`、`grantable()`、`heldLongEnough()`、`releaseAllOf()` | 每个仅 inline 定义；正式 `decide()` 不执行粗粒度仲裁 | REVIEW；与 deprecated token 仲裁整组处理 |

另外，`VehicleAgent::recent_targets/recent_rows` 只在 `TaskAllocator::rememberTask()` 写入和裁剪，没有读取；不能只删字段而保留序列化/快照假设，建议与三个未用私有函数、两个配置键作为一个清理单元。

## 10. 配置与入口一致性问题

1. `planner_param.yaml` 中 `forklift_planner/wheel_base`、`max_steer_angle`、`max_steer_rate`、`path_resolution` 没有被 `PlannerParam::fromROSParam()` 读取。注释说明这些量已迁移到 `forklift_map/MapParam`；唯一例外是标定节点会把 `/forklift_planner/wheel_base` 当 fallback 读取。分类：**REVIEW**。建议目标实车标定入口改为显式读取唯一真值源后，再删除 planner 下重复键。该动作涉及安全参数，必须负责人批准和实验记录。
2. `recent_target_memory`、`recent_row_memory` 被读取并限制缓存长度，但缓存不参与候选评分/过滤。状态是“已配置、已读取、未用于任务选择”。分类：**REVIEW**。
3. 多个 launch 默认指向未跟踪的 `config/a1_cycle_path_catalog.yaml`。`TaskAllocator::ensureA1LegCache()` 在文件不存在时会从 generator 构建，并在 `save_a1_cycle_catalog=true` 时写出，因此不是硬缺失依赖；但会修改源码包目录。建议将生成目录移至实验产物目录，并明确缓存版本/参数哈希。
4. `simulation.launch` 与 `forklift_map/map.launch` 都启动 `map_node + planner_node + RViz`，参数加载作用域略有不同。分类：**REVIEW**；先选定规范入口并验证等价后合并。
5. `docs/PROJECT_OVERVIEW.md` 引用不存在的 `single_vehicle_patrol.launch`，属于文档漂移，应修正文档。
6. Python 实验脚本大多未用 `catkin_install_python()` 安装且 Git 模式为 100644；source workspace 中 rosrun/roslaunch 的行为依赖环境。目标安装空间是否可运行：**未知，需要确认**。

## 11. 长时失活/死锁测试的最优方案

### 11.1 不建议“始终开 RViz，死锁时切回 1 倍”作为主方案

原因不是 RViz 不能倍速，而是该方案不能回看触发前的因果过程：检测到死锁时再降速，只能看到已经形成的现场。高速状态下 RViz/marker/rosbag 还会引入渲染和序列化开销，改变 wall-time 调度，却不应改变固定 `dt` 的算法语义；一旦丢帧，也无法保证视觉上看到每个关键状态。

### 11.2 推荐：两阶段“快速判定 + 事件回放”

```text
固定 seed 的 headless batch（尽 CPU 速度）
  -> 每 tick 做安全/进展判定
  -> 内存环形缓冲保存最近 60~120 s 的可回放状态
  -> HARD_GUARD / NO_PROGRESS / WAIT-CYCLE / 异常退出
  -> 冻结触发前缓冲，再采集触发后 10~30 s（若继续安全）
  -> 写出事件包：状态帧 + 参数 + seed + 日志 + 触发原因
  -> 单独启动 RViz，以 0.25x/1x/单步离线回放
```

这同时满足：长时运行不占真实时长；问题发生时保留前因后果；回放不会继续影响规划器；相同 seed 可再次精确复现。

### 11.3 基于现有代码的最小实施路径

当前已有：

- `runBatch()`：固定 `dt=1/update_rate` 的无头紧循环；
- `stress_watchdog_enabled`：`HARD_GUARD`、单车 `NO_PROGRESS`、基础设施中断判定；
- `stress_ring`：最近 120 秒文本状态；
- `snapshot_debug_enabled`：实时模式下等待超过 25 秒时发布 trigger；
- `rolling_rviz_bag_recorder.py`：约 120 秒 marker rolling window；
- `replay_rviz_snapshot.py`：支持 `--clock`、倍率和暂停回放。

缺口与建议优先级：

1. **P0：先把现有 stress watchdog 暴露到 batch launch。** 为 batch launch 增加 `stress_watchdog_enabled`、`stress_progress_timeout`、`stress_result_file`、`stress_failure_file` 参数，并用固定 seed 批量运行。这里只改变诊断入口，不改变协调规则。
2. **P0：事件判定同时覆盖碰撞、无进展和等待环。** 不能只用 `wait_time > 25s`；应保存 deadlock cluster/wait graph、blocker、reason、path generation、task count。阈值应沿用现有配置/经批准新增，不在本审计中修改。
3. **P1：为 batch 增加“状态帧环形缓冲”，不要每 tick 发布 ROS marker。** 帧保存生成 marker 所需的 agent pose/path、conflict marker、A1 commitment、deadlock directive 和时间戳。触发时序列化为专用事件包或 rosbag。这样快跑开销可控。
4. **P1：离线回放器从状态帧重建 marker。** 地图 marker 只需保存/发布一次，动态 planner marker 按记录的仿真时间发布 `/clock`。回放可 0.25x、1x、4x、暂停和单步。
5. **P2：可选 live fast-view。** 若确实想边跑边看，可让 runner 每个 wall timer tick 执行 N 个固定仿真 step，只按 5~10 Hz wall rate 抽样发布 marker；检测异常后立即暂停仿真并自动启动/提示离线回放。不要在异常后继续 1x 运行来代替前情缓存。

### 11.4 为什么现有 rolling rosbag 不能直接用于 batch

`runBatch()` 在循环中不调用 `MarkerPublisher::publish()`；`multi_vehicle_batch.launch` 也不启动 `rolling_rviz_bag_recorder.py`。虽然 batch 末端会调用 `updateSnapshotWedgeTrigger()`，但没有订阅 trigger 的 recorder，也没有 marker 流可录。因此必须增加 batch 状态记录/离线重建，而不是只在 launch 中打开现有 recorder。

### 11.5 建议的验收指标

- 同一 commit、配置、seed 重复两次，任务序列、首次异常 tick、异常类型一致；
- 120 仿真分钟的 wall time 明显小于 120 分钟，并报告倍率而不是承诺固定倍率；
- 事件包至少包含触发前 60 秒、触发后 10 秒或安全停止点；
- 回放中车辆 pose、action、blocker、reason、冲突位置、deadlock cluster 与文本日志同 tick 对齐；
- 快跑开/关记录功能时，算法输出序列一致；
- 碰撞硬保护、路径校验、急停/实车保护完全不因快跑模式改变；
- 该能力仅用于仿真；`real_mode=true` 禁止倍速推进。

## 12. 建议的轻量化执行顺序

1. 独立清理提交：`1.txt`、`.pyc`、`tinyproto.zip`，补 `.gitignore`；不涉及功能。
2. 删除无引用 `forklift_map/common/math` 与 `common/util` 前，执行核心包干净构建、路径目录诊断、B-A1/A1-B 回归。
3. 将未接入的两个测试加入 CMake，先让证据变强，再清函数。
4. 删除明确未调用的小函数与 recent-memory 空壳；对任务序列做固定 seed 重复比较。
5. 对 deprecated resource arbitration、普通 B-B、旧死锁/当前位置改道只做“标识和隔离”，负责人批准前不删除。
6. 对 experiment 按“仍用实车链 / 历史实验 / 工具”拆包；必须在目标实车环境构建和运行后才能给出最终 REMOVE 结论。
7. 最后再做大文件拆分：优先把 `multi_vehicle_patrol_node.cpp` 的 batch runner、实时 IO、marker/debug recorder 拆成独立组件，保持状态机与规则顺序不变。

## 13. 本轮验证状态

- 已完成：全仓文件递归清单、CMake/package/launch/YAML/topic/参数/静态引用审计；审计开始时现存的 215 个非 `.git` 文件均在本报告逐项或在明确的 vendored 文件逐项表中登记（其中 214 个可由 `rg --files` 枚举，另 1 个是被 ignore 的 `.vscode/settings.json`）。
- 已完成：用户已有 Git 修改/删除识别与保护。
- 未执行：catkin 构建、CTest、路径目录运行、多车 batch、RViz/实车运行。原因：当前 Windows 环境未发现 ROS/catkin 命令，且本轮目标是只读审计报告。
- 风险：仓外消费者、手工 rosrun 工具、目标实车依赖和历史实验用途无法由仓内静态引用完全证明，均已标记 REVIEW 或“未知，需要确认”。
