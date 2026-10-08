# Warehouse Forklift Multi-Vehicle Coordination Planning System

中文名称：仓储叉车多车协同规划系统

文档状态：项目基线概览  
适用范围：当前仓库中的核心规划系统、公共消息接口和实车适配实验模块

## 状态标记

本文统一使用以下状态，避免把“代码存在”误写成“正式支持”：

- **Confirmed**：当前代码能够证明，且项目负责人已确认属于正式基线。
- **Deprecated**：历史代码仍然存在，但已被负责人确认废弃；保留但冻结，禁止作为新设计基础。
- **Planned**：已确认属于后续阶段，但当前不属于已实现业务。
- **Unknown**：当前代码和已确认基线均无法给出确定结论，必须标记“未知，需要确认”。

## 1. 项目简介

### 1.1 项目名称

- 英文名称：**Warehouse Forklift Multi-Vehicle Coordination Planning System**
- 中文名称：**仓储叉车多车协同规划系统**

该名称用于项目级文档和项目引用，不替代 ROS package 名称。ROS package 继续使用 `forklift_map`、`forklift_planner` 和 `sandbox_msgs`。

如果后续论文、比赛或实车系统确定正式名称，应统一修改项目名称字段及相关文档引用。

状态：**Confirmed**。

### 1.2 项目目标

本项目是一个面向仓储叉车多车协同规划的 ROS 1 项目。当前核心代码解决以下问题：

- 建立固定仓储场地、货架、B 库位和道路的几何模型；
- 为正式 `B -> A1 -> B` 业务流程生成路径并检查路径合法性；
- 为多辆车分配任务并维护车辆任务状态；
- 根据车辆固定路径之间的几何关系进行跟车和冲突协调；
- 在仿真模式中更新车辆状态，并向 RViz 和日志输出运行信息；
- 为实车适配实验链提供轨迹、协调速度和协调状态输出。

状态：**Confirmed**。

源码依据：

- `forklift_map/src/forklift_map.cpp`：`ForkliftMap::build()`、`build_shelves()`、`build_slots()`、`build_roads()`。
- `forklift_planner/include/forklift_planner/path_generator.h`：`PathGenerator`。
- `forklift_planner/src/multi_vehicle/task_allocator.cpp`：`TaskAllocator` 的任务缓存、路径验证和任务分配函数。
- `forklift_planner/src/multi_vehicle/rule_engine.cpp`：`RuleEngine::decide()`。
- `forklift_planner/src/multi_vehicle_patrol_node.cpp`：`MultiVehiclePatrolNode`。

### 1.3 应用场景

当前代码中的地图、B 库位、A1 业务站点、叉车尺寸参数、多车任务状态和实车适配接口共同构成仓储叉车规划场景。

本文不据此推断项目已支持任意仓库、动态地图、通用导航栈、动态障碍物规划或其他物流业务。代码未证明的场景均不属于本文确认范围。

状态：**Confirmed**。

### 1.4 当前阶段

当前阶段已经形成以 `forklift_map + forklift_planner` 为主体的仿真规划主链，并保留实车适配实验代码。正式业务流程已经确定为 `B -> A1 -> B`。A2 仍处于下一阶段业务规划，自动化测试体系尚不完整。

这里的“已经形成”指仓库中存在对应源码、构建目标和 launch 入口，不代表所有功能已经在当前虚拟机或实车环境完成运行验收。

状态：**Confirmed**。

## 2. 项目边界

### 2.1 当前核心模块

#### `forklift_map`

核心地图模块，负责：

- 场地尺寸和分区参数；
- 货架、B 库位和道路几何构造；
- 地图数据的只读访问和库位占用字段；
- 地图及路径目录调试的 RViz 标记输出；
- 供地图与规划器共用的 Clothoid 几何实现。

主要源码：

- `forklift_map/include/forklift_map/forklift_map.h`：`ForkliftMap`。
- `forklift_map/include/forklift_map/map_param.h`：`MapParam`。
- `forklift_map/include/forklift_map/map_types.h`：`Slot`、`ShelfBlock`、`RoadSegment`。
- `forklift_map/src/map_node.cpp`：`MapNode`。
- `forklift_map/src/clothoid.cpp`。

状态：**Confirmed**。

#### `forklift_planner`

核心规划模块，负责：

- 路径生成和路径调试信息；
- 路径合法性过滤；
- `B -> A1 -> B` 任务分配和物流状态推进；
- 车辆路径弧长状态管理；
- 多车跟车、冲突检测、优先级和停车/限速动作决策；
- 仿真车辆状态推进；
- 规划路径、车辆和冲突的 RViz 可视化；
- 实车适配实验所需的轨迹和协调状态输出。

主要源码：

- `forklift_planner/src/path_generator.cpp` 及 `src/path_generator_routes/`。
- `forklift_planner/src/multi_vehicle/task_allocator.cpp`。
- `forklift_planner/src/multi_vehicle/rule_engine.cpp`。
- `forklift_planner/src/multi_vehicle/path_track.cpp`。
- `forklift_planner/src/multi_vehicle_patrol_node.cpp`。

状态：**Confirmed**。

#### `experiment/sandbox_msgs`

`sandbox_msgs` 虽位于 `experiment` 目录，但它是核心系统使用的公共 ROS 消息接口包，是 `experiment` 定位规则的明确例外。

当前消息定义包括：

- `AprilObject.msg`：带类型、ID、位置、航向和多边形字段的对象消息；
- `Trajectory.msg`：指定目标车辆的轨迹点序列；
- `TrajectoryPoint.msg`：时间、位置、航向、速度和加速度；
- `ChassisCommand.msg`：目标车辆、油门和转向命令。

`forklift_planner/package.xml` 和 `forklift_planner/CMakeLists.txt` 均声明了对 `sandbox_msgs` 的依赖。

状态：**Confirmed**。

### 2.2 实车适配实验模块

`experiment` 中除 `sandbox_msgs` 外的代码定位为实车适配实验模块，主要包含：

- `vrpn_client_ros`：接收 VRPN 跟踪数据并发布 pose/twist/accel；
- `nokov_localization`：发现动捕 topic，整理车辆和障碍物信息并发布 `/object` 等消息；
- `pure_pursuit`：实验性路径跟踪控制器；
- `lqr_controller`：实验性 LQR 控制器；
- `chassis`：通过串口发送底盘命令并接收速度信息。

当前虚拟机缺少实车实验相关依赖，因此 `chassis` 和 `vrpn_client_ros` 使用 `CATKIN_IGNORE`，不属于当前环境的默认编译链。不得把 `experiment` 描述为当前仿真主链，也不得因当前虚拟机编译需求擅自删除这些 `CATKIN_IGNORE`。

实车链在当前目标环境是否可以完整构建、启动和安全运行：**Unknown——未知，需要确认**。

### 2.3 不属于当前已确认范围的能力

当前代码不能确认以下能力已属于正式系统：

- A2 业务状态机和正式任务分配；
- 动态障碍物参与核心路径生成或多车协调；
- 面向任意地图的通用规划；
- 完整自动化测试和持续集成；
- 实车安全认证或生产部署能力。

状态：**Unknown** 或 **Planned**，具体见第 6、9、10 节。

## 3. 系统总体架构

### 3.1 分层结构

```text
配置与启动层
  map_param.yaml / planner_param.yaml / launch
                    |
                    v
地图层：forklift_map
  MapParam -> ForkliftMap -> Slot / ShelfBlock / RoadSegment
                    |
                    v
规划与任务层：forklift_planner
  PathGenerator <-> TaskAllocator -> VehicleAgent / PathTrack
                    |
                    v
多车协调层：forklift_planner
  RuleEngine -> VehicleAction / blocker / reason
                    |
                    v
状态执行层
  仿真：advanceVehicles()
  实车实验：realAdvance() + 轨迹/协调输出
                    |
                    v
输出层
  RViz MarkerArray / ROS 日志 / 协调日志 / 实车实验 topics
```

状态：**Confirmed**。

### 3.2 核心对象关系

`MultiVehiclePatrolNode` 是多车主流程的组织者，其构造过程实际创建并连接以下对象：

```text
MultiVehiclePatrolNode
├── ForkliftMap
├── TrafficResourceMap
├── PathGenerator
├── TaskAllocator
├── RuleEngine
├── MarkerPublisher
└── vector<VehicleAgent>
      └── PathTrack
```

主要依据：`forklift_planner/src/multi_vehicle_patrol_node.cpp` 中 `MultiVehiclePatrolNode::MultiVehiclePatrolNode()`。

需要注意：`TrafficResourceMap` 当前仍被构造并用于路径资源跨度和诊断，但粗粒度资源仲裁不属于正式协调方案，见第 5.4 和第 6 节。

### 3.3 主要输入

核心仿真主链的输入包括：

- `forklift_map/config/map_param.yaml`；
- `forklift_planner/config/planner_param.yaml`；
- launch 参数，例如车辆数、起始库位、目标库位、是否一次运行和 batch 时长；
- 固定随机种子及任务分配随机性配置；
- 地图构造后产生的 `Slot`、`ShelfBlock` 和 `RoadSegment`。

实车适配实验模式还可输入：

- `/object`：车辆真实位姿消息；
- `/rb_start`：实车实验启动信号；
- `/estop`：实车实验急停信号。

状态：**Confirmed**。

### 3.4 主要输出

核心系统输出包括：

- `/forklift_map/markers`：地图和路径目录调试标记；
- `/forklift_planner/markers`：车辆、路径、预测和冲突标记；
- ROS 日志：任务、路径拒绝、冲突、等待、死锁诊断和 batch 汇总；
- 协调日志文件：由 `coord_log_file` 指定。

实车适配实验模式还会发布：

- `/traj_<id>`；
- `/coord_speed_<id>`；
- `/coord_state_<id>`。

状态：**Confirmed**。

## 4. 正式业务流程

### 4.1 正式流程：`B -> A1 -> B`

正式任务循环为：

```text
B 库位
  |
  | 空载，分配 B -> A1 取货航段
  v
A1 业务站点
  |
  | PICKUP_DWELL：取货等待
  | 选择并准备 A1 -> B 目标航段
  v
目标 B 库位
  |
  | UNLOAD_DWELL：卸货等待
  | 完成一次运输任务
  v
再次分配 B -> A1 航段
```

在代码中，A1 是合成业务站点，不是 `ForkliftMap::slots()` 中的普通 B 库位。车辆到达 A1 后，`current_slot` 保留上一个物理 B 库位，物流阶段进入 `PICKUP_DWELL`。

对应状态转换：

```text
TO_A1
  -> PICKUP_DWELL
  -> WAIT_DROPOFF_TASK（仅在暂时无法分配目标 B 时）
  -> TO_B
  -> UNLOAD_DWELL
  -> TO_A1
```

源码依据：

- `forklift_planner/include/forklift_planner/multi_vehicle/vehicle_agent.h`：`MissionPhase`、`LegTargetKind`。
- `forklift_planner/src/multi_vehicle/task_allocator.cpp`：`assignPickupLeg()`、`prepareDropoffLeg()`、`activatePreparedDropoffLeg()`、`assignDropoffLeg()`。
- `forklift_planner/src/multi_vehicle_patrol_node.cpp`：`updateDwellAndTasks()`、`handleLegArrival()`。

状态：**Confirmed / Implemented，正式业务流程**。

### 4.2 Deprecated 流程

以下逻辑仍存在于代码中，但不属于正式业务流程：

- 普通 `B -> B`；
- `simple_forward_demo`。

它们执行“保留但冻结”规则：当前阶段不删除，但禁止新增功能、禁止作为新设计基础、禁止让正式 `B -> A1 -> B` 新增对其依赖，也禁止通过配置或隐式回退重新成为默认正式流程。

状态：**Deprecated**。

### 4.3 A2

仓库中存在 `A2_TO_B`、`B_TO_A2` 路径模式和相应路线源码，但正式多车物流状态机没有 A2 阶段。

A2 已被负责人确定为下一阶段业务站点，当前不得描述为已实现业务流程，也不得在未确认任务语义、状态机、接口和验收标准前接入正式主链。

状态：**Planned**。

## 5. 核心功能模块

### 5.1 Map 模块

Map 模块通过 `MapParam` 和 `ForkliftMap` 构造静态场地模型。

已确认职责：

- 按参数构造场地边界；
- 构造货架矩形块；
- 构造 B 库位及其停靠位姿和预停靠点；
- 构造道路中心线段；
- 提供库位、货架和道路的只读访问；
- 通过 `MapNode` 发布 RViz MarkerArray。

状态：**Confirmed**。

### 5.2 Path Planning 模块

路径规划模块以 `PathGenerator` 为入口，输出由稠密 `RoughWp` 组成的 `RoughPath`。每个路径点包含位置、航向和 `FORWARD/REVERSE` 类型。

已确认能力：

- 按路线模式分派 `B -> A1`、`A1 -> B`、`B -> A2`、`A2 -> B` 或通用路线生成代码；
- 构造通道骨架路径；
- 对转角使用 Clothoid 曲率连续转弯；
- 在局部 Clothoid 不可行时保留 arc fallback 路径信息；
- 支持终端正向或倒车 docking；
- 通过 `PathGenerationInfo` 记录 fallback 和调试层；
- 通过 `TaskAllocator::validatePath()` 检查空路径、fallback 策略、非法尖点、边界以及按配置启用的货架碰撞。

`arc fallback` 的存在不等于 Clothoid 生成成功；路径是否允许进入任务集合还取决于配置和 `TaskAllocator` 的验证结果。

状态：**Confirmed**。

### 5.3 Task Allocation 模块

`TaskAllocator` 负责把路径生成结果转换为可执行任务，并维护任务选择所需的缓存和历史。

正式流程中的已确认职责：

- 构造或读取 A1 航段缓存；
- 验证 B 库位是否存在有效 `B -> A1` 航段；
- 为车辆分配空载 `B -> A1` 航段；
- 在 A1 取货阶段选择并预留目标 B；
- 激活满载 `A1 -> B` 航段；
- 避免把已被其他车辆占用、作为当前目标或预留目标的库位重复分配；
- 记录近期目标、近期排和访问计数；
- 使用已确认的确定性真伪随机逻辑，使相同配置和随机种子下的任务选择可复现。

`TaskAllocator::replanFromPose()` 中“从当前位置重新规划到其他库位”的方法属于 deprecated，不得作为新死锁设计继续使用。

状态：正式 A1 任务分配为 **Confirmed**；当前位置换库位策略为 **Deprecated**。

### 5.4 Multi-Vehicle Coordination 模块

当前正式协调入口是 `RuleEngine::decide()`。按照当前函数实际调用顺序，协调过程包括：

```text
等待关系/死锁状态更新
  -> 刷新路径资源跨度
  -> 重置车辆动作请求
  -> 跟车处理
  -> 精确两车几何冲突处理
  -> 目标库位占用处理
  -> 前向净空保护
  -> 动作保持与动作应用
  -> 更新等待时间
```

已确认职责：

- 同向路径跟车；
- 基于车辆路径和 OBB 足迹的两车冲突区检测；
- 对冲突车辆选择确定性优先方；
- 防止车辆驶入仍被占用的目标库位；
- 对当前车身前方净空执行保护；
- 输出 `STOP`、`CREEP`、`YIELD`、`NOMINAL` 或 `BOOST` 动作；
- 通过动作保持时间减少 STOP/放行抖动；
- 记录 blocker、reason、wait time 和冲突标记。

优先级代码考虑 deadlock breaker 标志、目标库位占用者清出、饥饿等待、载货状态、完成任务数和车辆 ID。该描述只说明当前代码行为，不代表所有组合场景已通过运行验证。

粗粒度资源仲裁已经从正式决策顺序中停用；相关资源数据结构和诊断代码仍然存在，但禁止作为新协调方案重新扩展。

旧 deadlock reverse、旧 cycle break、旧 stall release，以及当前位置重规划换库位策略均不属于正式新设计。

状态：当前精确冲突协调主链为 **Confirmed**；上述旧方案为 **Deprecated**。

### 5.5 State Management 模块

`VehicleAgent` 同时保存：

- 车辆 ID、当前 B 库位、目标 B 库位；
- `VehicleMode` 和 `MissionPhase`；
- 是否载货；
- 当前动作、请求动作和 blocker；
- 路径、`path_s`、速度和轨迹版本；
- 取货/卸货等待时间；
- 任务计数、等待时间和近期任务历史；
- A1 离场准备状态；
- 实车实验位姿和资源跨度缓存。

`MultiVehiclePatrolNode` 负责在定时周期内调用任务更新、协调决策、仿真或实车状态更新以及可视化输出。

状态：**Confirmed**。

### 5.6 Visualization and Diagnostics 模块

已确认输出包括：

- 地图、货架、道路、库位和方向；
- 车辆车身、ID、朝向和路径；
- 预测轨迹和冲突位置；
- A1 局部区域调试；
- 路径生成层、fallback 和拒绝原因；
- 多车状态、等待、碰撞、死锁簇和 batch 汇总日志。

可视化和诊断用于观察与回归，不等价于自动判定所有测试通过。

状态：**Confirmed**。

## 6. 当前实现状态

| 功能 | 状态 | 说明 | 主要依据 |
|---|---|---|---|
| 静态场地、货架、B 库位和道路建模 | Confirmed / Implemented | 当前核心地图模型 | `ForkliftMap::build*()` |
| 地图与规划 RViz 可视化 | Confirmed / Implemented | 发布 MarkerArray | `MapNode`、`MarkerPublisher` |
| `B -> A1 -> B` | Confirmed / Implemented | 当前正式业务流程 | `MissionPhase`、`TaskAllocator`、`updateDwellAndTasks()` |
| A1 取货和 B 库位卸货等待 | Confirmed / Implemented | 使用独立 pickup/unload dwell 状态和参数 | `handleLegArrival()`、`updateDwellAndTasks()` |
| Clothoid 路径 | Confirmed / Implemented | 当前路径生成能力 | `clothoid.cpp`、路线生成源码 |
| Arc fallback | Confirmed / Implemented | 作为局部退化路径并被显式标记 | `PathGenerationInfo::used_arc_fallback` |
| 正向/倒车 docking | Confirmed / Implemented | 路径点带 `FORWARD/REVERSE` 类型 | `PathGenerator`、`WpType` |
| 路径合法性过滤 | Confirmed / Implemented | 空路径、fallback、kink、边界及可配置货架碰撞 | `TaskAllocator::validatePath()` |
| 确定性任务分配 | Confirmed / Implemented | 正式基线要求相同种子可复现 | `TaskAllocator`、`MultiVehicleConfig` |
| 跟车与精确两车几何冲突协调 | Confirmed / Implemented | 当前正式协调主链 | `RuleEngine::resolveFollowing()`、`resolvePairwiseConflicts()` |
| 目标库位占用与前向净空 | Confirmed / Implemented | 当前正式协调保护 | `resolveTargetSlotOccupancy()`、`enforceForwardClearance()` |
| 普通 `B -> B` | Deprecated | 历史兼容逻辑，保留但禁止扩展 | `DIRECT_TO_B` 及普通任务分配分支 |
| `simple_forward_demo` | Deprecated | 历史测试/演示逻辑，禁止作为正式方案 | `MultiVehicleConfig`、`TaskAllocator::assignNextTask()` |
| A2 业务站点 | Planned | 下一阶段业务；当前只有路径代码，没有正式业务状态机 | A2 route mode 与路线源码 |
| 粗粒度资源仲裁 | Deprecated | 正式 `RuleEngine::decide()` 已停用该调用 | `RuleEngine::decide()` |
| 旧 stall release / deadlock reverse / cycle break | Deprecated | 旧策略，禁止作为新设计基础 | 配置开关与 `RuleEngine` 旧逻辑 |
| 当前位置重规划到其他库位 | Deprecated | 禁止作为新的死锁解决方案 | `TaskAllocator::replanFromPose()` |
| 实车适配实验链 | Confirmed / Experimental | 源码和 launch 存在；不属于当前默认虚拟机编译链 | `experiment`、`realbridge_exp.launch` |
| 实车全链运行验收 | Unknown | 当前虚拟机缺少相关依赖 | 未知，需要确认 |
| 自动化单元/集成测试体系 | Incomplete | 核心包未发现有效 gtest/rostest 接入 | CMake 和现有诊断程序 |
| 动态障碍物参与核心规划 | Unknown | 实验定位代码发布 `/obstacles`，核心规划节点未发现订阅 | 未知，需要确认 |

## 7. 软件运行流程

### 7.1 多车仿真启动

以 `forklift_planner/launch/multi_vehicle_patrol.launch` 或 `single_vehicle_patrol.launch` 为例，launch 层执行：

```text
加载 map_param.yaml
  -> 加载 planner_param.yaml
  -> 设置特定模式或任务参数
  -> 启动 forklift_map/map_node
  -> 启动 forklift_planner/multi_vehicle_patrol_node
  -> 启动 RViz（可选，不作为核心节点退出条件）
```

状态：**Confirmed**。

### 7.2 多车节点初始化

`MultiVehiclePatrolNode` 构造过程按照代码执行：

```text
读取 MapParam / PlannerParam / MultiVehicleConfig
  -> 创建 ForkliftMap
  -> 创建 TrafficResourceMap
  -> 创建 PathGenerator
  -> 创建 TaskAllocator
  -> 创建 RuleEngine 并接入资源地图
  -> 按配置构建任务路径缓存
  -> 创建 MarkerPublisher
  -> 初始化 VehicleAgent
  -> 为启用车辆分配初始任务
  -> 创建定时器，或进入无头 batch 模式
```

正式 `B -> A1 -> B` 模式下，初始任务是每辆启用车辆的 `B -> A1` 航段。

状态：**Confirmed**。

### 7.3 仿真主循环

当前普通多车仿真定时流程不是简单地每拍直接重新规划。代码包含滚动时域推演和冻结计划执行，其主要顺序为：

```text
updateDwellAndTasks(dt)
  -> 判断滚动计划是否需要刷新
  -> publishHorizon() 构造前瞻计划（需要时）
  -> executeSimulationPlanSample() 执行冻结计划的一拍
  -> 无可执行帧时调用 RuleEngine::decide() 作为安全回退
  -> advanceVehicles(dt)
  -> A1 离场侵入诊断
  -> 周期性 deadlock 诊断/旧恢复入口
  -> 状态日志和卡滞诊断
  -> MarkerPublisher::publish()
```

其中旧 deadlock 恢复入口及其当前位置换库位逻辑属于 deprecated。其代码存在不改变其设计状态。

状态：**Confirmed**。

### 7.4 无头 batch

`multi_vehicle_batch.launch` 启动 `multi_vehicle_patrol_node` 的无头快速运行模式。节点不创建实时时钟 timer，而由 `runBatch()` 按固定 `dt` 快速推进指定仿真时长，并输出碰撞、等待、死锁和任务统计。

batch 是当前回归手段之一，但不是完整自动化测试体系。

状态：**Confirmed**。

### 7.5 实车适配实验流程

`realbridge_exp.launch` 描述的实验链为：

```text
VRPN / Nokov 位姿
  -> nokov_localization
  -> /object
  -> multi_vehicle_patrol_node(real_mode=true)
  -> /traj_<id> + /coord_speed_<id> + /coord_state_<id>
  -> Pure Pursuit 或 LQR 实验控制器
  -> /chassis
  -> chassis_node
  -> 串口底盘
```

实车 launch 以逗号分隔的 `vehicle_ids` 作为参与车辆的唯一选择参数。
规划器只创建这些真实 ID 的 `VehicleAgent`，控制器启动器也从同一 ROS 参数
启动对应的 `controller_<id>`；`vehicle_count` 仅保留给连续编号的仿真模式。

该流程的源码和 launch 存在，但当前虚拟机不具备完整实车依赖，因此不能据此确认实车全链已在当前基线完成运行验收。

状态：**Confirmed / Experimental**；运行验收为 **Unknown——未知，需要确认**。

## 8. 当前开发原则

所有开发和 AI 协作必须遵守仓库根目录 `AGENTS.md`。本节仅摘要，不替代原文件。

### 8.1 代码与证据

- 所有结论必须来自代码、配置、构建文件、launch、接口定义或运行证据；
- 无法确认时必须写“未知，需要确认”；
- 重要结论应注明文件、类和函数；
- 必须区分代码存在、默认启用、正式支持和运行验证。

### 8.2 正式流程与 deprecated 管理

- 新功能围绕正式 `B -> A1 -> B` 流程设计；
- 普通 `B -> B`、`simple_forward_demo` 和旧协调/死锁逻辑保留但冻结；
- 禁止重新启用、扩展或让正式流程新增对 deprecated 逻辑的依赖；
- deprecated 代码如必须修复，只允许经负责人确认的最小修改。

### 8.3 安全规则

- 未经负责人批准，禁止关闭或削弱碰撞硬保护、实车急停、动捕超时停车、实车足迹紧急保护、路径边界检查和正式流程使用的路径尖点检查；
- 安全阈值修改必须有负责人批准和实验记录；
- 不得以吞吐量、临时调试或日志简化为理由绕过安全规则；
- 代码中存在安全判断不等于已完成实车安全认证。

### 8.4 回归验证

- 路径核心修改必须执行路径目录诊断，覆盖 `B -> A1` 与 `A1 -> B`；
- 多车协调核心修改必须执行固定种子的确定性 batch 回归；
- 任务随机性修改必须验证相同输入和种子产生可复现任务序列；
- 未执行验证时必须报告原因，禁止把“编译通过”描述为“回归通过”。

### 8.5 实验记录

核心逻辑或参数实验必须记录实验编号、Git commit、工作区状态、修改点、参数、运行命令、随机种子、日志、结果、结论和是否保留。

## 9. 已知限制

### 9.1 A2 尚未进入正式业务

A2 只有路径生成和调试相关代码，尚未进入 `MissionPhase`、正式任务分配和正式多车业务流程。

状态：**Planned**。

### 9.2 实车依赖不属于当前虚拟机默认编译链

当前虚拟机缺少实车实验依赖，相关包通过 `CATKIN_IGNORE` 隔离。当前环境不能用于证明完整实车适配链已经构建或运行通过。

状态：**Confirmed limitation**。

### 9.3 自动化测试不足

核心包 CMake 中未发现已接入的 gtest/rostest。当前主要依赖路径调试可执行程序、RViz 检查和多车 batch 日志回归。这些手段不能替代完整自动化测试体系。

状态：**Confirmed limitation**。

### 9.4 路径验证由配置控制

`TaskAllocator::validatePath()` 支持边界、货架碰撞、fallback 和 kink 等检查，但部分检查是否启用取决于配置。不得把“存在检查代码”描述为“所有检查始终开启”。

状态：**Confirmed limitation**。

### 9.5 配置不等于生效

当前仓库存在 YAML 或 launch 声明与节点读取点需要分别核对的情况。任何参数说明都必须确认参数被正确命名空间加载、被代码读取并实际参与逻辑。

状态：**Confirmed limitation**。

### 9.6 核心地图包存在调试层反向源码依赖

`forklift_map/CMakeLists.txt` 为 `path_catalog_debug_node` 直接编译 `forklift_planner` 的路径生成源码，并引用相邻 planner include 目录。这是当前构建事实，但不代表它是长期认可的模块边界。

其后续是否重构：**Unknown——未知，需要确认**。

### 9.7 历史逻辑仍保留在主源码中

普通 `B -> B`、演示任务和旧死锁处理仍与正式逻辑共存。开发者必须依靠状态标记和 `AGENTS.md` 约束避免误用；当前阶段不得直接删除。

状态：**Confirmed limitation / Deprecated code retained**。

## 10. 未确认事项

以下内容无法从当前代码和已确认基线得到完整结论，不得自行补全：

1. **Unknown：实车全链验收状态。** 当前虚拟机未编译实车依赖；是否已在目标实车环境完整通过，未知，需要确认。
2. **Unknown：实车安全认证或生产部署状态。** 代码存在急停、超时和足迹保护，但认证与现场验收证据未知，需要确认。
3. **Unknown：`world` 与 `map` 的正式坐标变换。** 当前代码中可见两个 frame 名称，但完整标定契约未知，需要确认。
4. **Unknown：动态障碍物是否属于正式规划范围。** 实验定位模块发布障碍物信息，核心规划节点未发现对应订阅，未知，需要确认。
5. **Unknown：A2 的详细业务定义。** 仅确认其为下一阶段站点；任务语义、状态机、接口和验收标准未知，需要确认。
6. **Unknown：当前代码基线最近一次完整构建与 batch 回归结果。** 本文基于源码审计和负责人确认，不包含本轮运行验证。
7. **Unknown：长期支持的 ROS、VRPN、控制器、串口库及实车硬件版本矩阵。** 当前仓库没有统一环境基线文档。
8. **Unknown：`TrafficResourceMap` 的长期保留范围。** 当前仍用于跨度缓存和诊断，粗粒度仲裁已 deprecated；后续是否仅保留诊断用途，未知，需要确认。
9. **Unknown：核心自动化测试和 CI 的目标门槛。** 当前已确认必须回归，但具体覆盖率、运行时长和合格阈值尚未形成代码内可验证标准。

## 11. 新开发者与 AI Agent 的使用原则

开始工作前，应按以下顺序理解项目：

1. 阅读根目录 `AGENTS.md`；
2. 把 `forklift_map + forklift_planner` 视为当前核心规划系统；
3. 把 `sandbox_msgs` 视为公共接口包；
4. 把其他 `experiment` 包视为实车适配实验模块；
5. 以 `B -> A1 -> B` 为唯一正式业务流程；
6. 不扩展普通 `B -> B`、`simple_forward_demo` 或旧协调/死锁方案；
7. 不把 A2 描述为当前已实现业务；
8. 修改路径或协调核心前先建立代码证据和回归计划；
9. 遇到无法确认的信息，明确写“未知，需要确认”并询问项目负责人。

本文用于回答三个基础问题：

- **这个项目是什么：**仓储叉车多车协同规划 ROS 系统。
- **当前解决什么问题：**围绕正式 `B -> A1 -> B` 流程完成地图、路径、任务、协调、状态推进和可视化。
- **哪些东西不能再使用：**普通 `B -> B`、`simple_forward_demo`、粗粒度资源仲裁、旧死锁策略和当前位置重规划换库位策略不得作为新设计基础。
