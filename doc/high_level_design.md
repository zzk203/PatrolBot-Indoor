# 室内巡逻机器人系统概要设计文档

## 目录

- [1. 概述](#1-概述)
  - [1.1 项目背景](#11-项目背景)
  - [1.2 设计范围](#12-设计范围)
  - [1.3 术语](#13-术语)
- [2. 总体架构](#2-总体架构)
  - [2.1 系统拓扑](#21-系统拓扑)
  - [2.2 架构视图](#22-架构视图)
  - [2.3 技术选型](#23-技术选型)
- [3. 行为树设计](#3-行为树设计)
  - [3.1 设计原则](#31-设计原则)
  - [3.2 行为树拓扑](#32-行为树拓扑)
  - [3.3 叶子节点清单](#33-叶子节点清单)
- [4. 状态机设计](#4-状态机设计)
  - [4.1 状态定义](#41-状态定义)
  - [4.2 状态转移](#42-状态转移)
- [5. 接口设计](#5-接口设计)
  - [5.1 Service 接口](#51-service-接口)
  - [5.2 Topic 接口](#52-topic-接口)
  - [5.3 Action Client 接口](#53-action-client-接口)
  - [5.4 自定义消息定义](#54-自定义消息定义)
- [6. 数据设计](#6-数据设计)
  - [6.1 黑板变量](#61-黑板变量)
  - [6.2 YAML 配置文件](#62-yaml-配置文件)
  - [6.3 日志文件格式](#63-日志文件格式)
  - [6.4 照片文件命名](#64-照片文件命名)
- [7. 模块设计](#7-模块设计)
  - [7.1 系统节点 (patrol_bot_node)](#71-系统节点-patrol_bot_node)
  - [7.2 配置加载器 (ConfigLoader)](#72-配置加载器-configloader)
  - [7.3 电池模型 (BatteryModel)](#73-电池模型-batterymodel)
  - [7.4 报警管理器 (AlarmManager)](#74-报警管理器-alarmmanager)
  - [7.5 巡逻日志 (PatrolLogger)](#75-巡逻日志-patrollogger)
  - [7.6 导航客户端 (Nav2ActionClient)](#76-导航客户端-nav2actionclient)
- [8. 关键场景流程](#8-关键场景流程)
  - [8.1 正常巡逻循环](#81-正常巡逻循环)
  - [8.2 低电回充与恢复](#82-低电回充与恢复)
  - [8.3 Critical 异常中断](#83-critical-异常中断)
  - [8.4 导航超时跳过](#84-导航超时跳过)
  - [8.5 充停控制指令](#85-充停控制指令)
- [9. 目录结构](#9-目录结构)
- [10. 依赖分析](#10-依赖分析)

---

## 1. 概述

### 1.1 项目背景

开发一个在 Gazebo Fortress 仿真环境中运行的室内巡逻机器人系统。机器人按照预设固定路线自动巡逻，具备低电回充、异常检测报警、日志记录等功能。

### 1.2 设计范围

本文档为概要设计文档，描述系统的模块划分、行为树结构、状态机、接口定义和数据设计。不涉及具体代码实现细节。

### 1.3 术语

| 术语       | 说明                                 |
| ---------- | ------------------------------------ |
| 巡逻路线   | 由多个有序巡逻点组成的巡检路径       |
| 巡逻点     | 路线上机器人的目标停留坐标           |
| 充电桩     | 机器人自动回充的目标位置             |
| 行为树(BT) | BehaviorTree.CPP v4，用于顶层任务编排 |
| 黑板       | BT Blackboard，行为树节点间的共享数据容器 |
| 抢占       | 高优先级条件触发时中断正在执行的子节点并切换行为 |

---

## 2. 总体架构

### 2.1 系统拓扑

采用**单节点架构**——所有巡逻业务逻辑运行在同一个 ROS2 节点 (`patrol_bot_node`) 内，BT 引擎为核心驱动。

```
┌────────────────────────────────────────────────────┐
│                  patrol_bot_node                   │
│                                                    │
│  ┌──────────────┐   ┌───────────────────────────┐  │
│  │  BT Engine   │   │    辅助模块               │  │
│  │  (Behavior   │   │  ┌─────────────────────┐  │  │
│  │   Tree.CPP)  │   │  │  ConfigLoader       │  │  │
│  │              │   │  ├─────────────────────┤  │  │
│  │  ┌────────┐  │◄──┤  │  BatteryModel       │  │  │
│  │  │ 行为树  │──┼──►│  ├─────────────────────┤  │  │
│  │  │ (XML)   │  │   │  │  AlarmManager       │  │  │
│  │  └────────┘  │   │  ├─────────────────────┤  │  │
│  │       ▼      │   │  │  PatrolLogger        │  │  │
│  │  ┌────────┐  │   │  └─────────────────────┘  │  │
│  │  │ 黑板    │  │   │                           │  │
│  │  │(BB)     │──┼───┤                           │  │
│  │  └────────┘  │   │  ┌─────────────────────┐  │  │
│  │              │   │  │  Nav2ActionClient    │  │  │
│  └──────────────┘   │  └─────────────────────┘  │  │
│                      └───────────────────────────┘  │
│                                                    │
│  ┌──────────────────────────────────────────────┐  │
│  │          ROS2 接口层                          │  │
│  │  Services: start/pause/resume/stop            │  │
│  │  Topics:   /patrol/status, /patrol/alarm       │  │
│  └──────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────┘
         │                    │                │
         ▼                    ▼                ▼
┌─────────────┐    ┌──────────────┐    ┌──────────────┐
│   Nav2      │    │  传感器驱动   │    │  Gazebo      │
│   (Action)  │    │  (Camera等)   │    │  Fortress    │
└─────────────┘    └──────────────┘    └──────────────┘
```

**设计理由**：
- 当前系统复杂度不需要进程级隔离（所有模块运行在同一线程空间，通过黑板共享数据）
- BT 本身已提供充分的模块化能力（子树、叶子节点）
- 减少 ROS2 通信开销，简化调试和部署

### 2.2 架构视图

```
         ┌──────────────────────────┐
         │      YAML 配置文件        │
         │  (路线/巡逻点/充电桩/参数) │
         └────────────┬─────────────┘
                      │ 加载
         ┌────────────▼─────────────┐
         │       ConfigLoader       │
         └────────────┬─────────────┘
                      │ 填充
         ┌────────────▼─────────────┐
         │      BT Blackboard       │◄────────── BatteryModel (每秒更新电量)
         │  - patrol_routes         │
         │  - battery_level         │◄────────── ConfigLoader (启动时加载)
         │  - patrol_state          │
         │  - current_route_index   │
         │  - current_waypoint_index  │
         │  - alarm_list            │
         │  - ...                   │
         └────────────┬─────────────┘
                      │ Tick
         ┌────────────▼─────────────┐
         │      BT Engine           │
         │  ┌─────────────────────┐ │
         │  │  行为树 (XML)        │ │
         │  │  - Condition 节点   │ │──► 读黑板: 判断状态
         │  │  - Action 节点      │ │──► 写黑板: 更新状态
         │  └─────────────────────┘ │
         └───────────┬──────┬──────┘
                     │      │
          ┌──────────┘      └──────────┐
          ▼                            ▼
┌──────────────────┐       ┌──────────────────┐
│  Nav2ActionClient│       │  AlarmManager    │
│  - NavigateToPose│       │  - 分级报警       │
│  - 超时监测      │       │  - Topic 发布     │
└──────────────────┘       └──────────────────┘
          │                            │
          ▼                            ▼
   ┌────────────┐             ┌──────────────┐
   │  Nav2 节点  │             │ /patrol/alarm │
   └────────────┘             │ ROS2 Topic    │
                              └──────────────┘
```

### 2.3 技术选型

| 组件       | 版本/选型                 |
| ---------- | ------------------------- |
| ROS2       | Humble                    |
| 行为树     | BehaviorTree.CPP v4       |
| 仿真引擎   | Gazebo Fortress (Ignition)|
| 导航框架   | Nav2 (Navigation2)        |
| 机器人模型 | TurtleBot3 (差分驱动)     |
| 容器化     | Docker                    |

---

## 3. 行为树设计

### 3.1 设计原则

- **抢占式响应**：使用 `ReactiveSequence` / `ReactiveFallback` 实现高优先级事件（低电、stop 指令、critical 异常）对正在运行动作的即时中断。
- **单一职责**：每个 BT 叶子节点只做一件事（导航、等待、拍照、报警等），组合由 XML 定义。
- **数据驱动**：所有配置和运行时状态存储在黑板中，节点通过黑板读取/写入。

### 3.2 行为树拓扑

```
顶层: ReactiveSequence                         // 抢占式：条件变化时中断子节点
├── [Condition]  patrol_state != STOPPED       // 已停止 → FAILURE 阻停整个树
├── [Condition]  patrol_state != PAUSED        // 已暂停 → FAILURE 挂起所有动作
├── [ReactiveFallback]                          // 抢占式回退：高优先级行为优先
│   ├── [Sequence]  充电子树                    // 优先级最高
│   │   ├── [Condition]  IsBatteryLow (电量 < 阈值)
│   │   ├── [Action]     SavePatrolContext (保存中断上下文)
│   │   ├── [Action]     NavigateToCharger (导航至充电桩)
│   │   └── [Action]     SimulateCharging (充电至恢复阈值)
│   │
│   └── [ReactiveSequence]  巡逻子流程          // 正常巡逻
│       ├── [Action]    LoadRoutes (从黑板加载路线数据)
│       ├── [Loop]      循环逐条路线
│       │   └── [Sequence]  执行单条路线
│       │       ├── [Action]  SetRouteContext (设置当前路线上下文)
│       │       └── [Loop]   循环逐巡逻点
│       │           └── [Sequence]  访问单个巡逻点
│       │               ├── [Action]     NavigateToWaypoint (导航，含超时)
│       │               ├── [Action]     WaitAtWaypoint (原地停留)
│       │               ├── [Action]     CaptureImage (拍照存档)
│       │               └── [ReactiveFallback]  异常处理
│       │                   ├── [Sequence]  Critical 分支
│       │                   │   ├── [Condition]  HasAlarm (当前点有异常)
│       │                   │   ├── [Condition]  IsAlarmCritical (严重性 == critical)
│       │                   │   └── [Action]     HandleAlarm (报警 + 写黑板暂停)
│       │                   └── [Sequence]  Warning 分支
│       │                       ├── [Condition]  HasAlarm
│       │                       └── [Action]     HandleAlarm (报警 + 继续)
```

**抢占机制详解**：

| 抢占场景 | 触发条件 | 机制 |
|---------|---------|------|
| 巡逻中低电 | `IsBatteryLow` 从 FAILURE → SUCCESS | `ReactiveFallback` 回退到充电子树，halt 巡逻中正在运行的 Action |
| 巡逻中收到 stop 指令 | `patrol_state` 改为 STOPPED | 顶层 `ReactiveSequence` 第一个 Condition 变 FAILURE → halt 整个树，不再 tick |
| 巡逻中收到 pause 指令 | `patrol_state` 改为 PAUSED | 顶层第二个 Condition 变 FAILURE → halt 所有运行中 Action，不退出树 |
| 巡逻点 critical 异常 | `IsAlarmCritical` → SUCCESS | 内层 `ReactiveFallback` 走 critical 分支，`patrol_state` 改为 PAUSED |

### 3.3 叶子节点清单

#### Condition 节点

| 节点类名 | 父类 | 职责 | 读取黑板 | 返回逻辑 |
|---------|------|------|---------|---------|
| `IsBatteryLow` | `ConditionNode` | 判断电量是否低于低电阈值 | `battery_level`, `low_threshold` | `battery_level < low_threshold` → SUCCESS |
| `IsChargingComplete` | `ConditionNode` | 判断充电是否达到恢复阈值 | `battery_level`, `recovery_threshold` | `battery_level >= recovery_threshold` → SUCCESS |
| `HasAlarm` | `ConditionNode` | 判断当前巡逻点是否存在异常 | `current_waypoint.alarm_simulate` | 存在异常字段 → SUCCESS |
| `IsAlarmCritical` | `ConditionNode` | 判断异常严重性是否为 critical | `current_waypoint.alarm_simulate.severity` | == `critical` → SUCCESS |

#### Action 节点

| 节点类名 | 父类 | 职责 | 对应需求 | 超时/Halt 行为 |
|---------|------|------|---------|---------------|
| `NavigateToWaypoint` | `StatefulActionNode` | 调用 Nav2 Action 导航至巡逻点 | FR-02, FR-15 | 超时返回 FAILURE；halt 时取消 Nav2 goal |
| `WaitAtWaypoint` | `StatefulActionNode` | 按配置时长原地停留 | FR-02 | halt 时立即结束 |
| `CaptureImage` | `SyncActionNode` | 调用摄像头拍照，按命名规则存档 | FR-16 | 无 |
| `NavigateToCharger` | `StatefulActionNode` | 调用 Nav2 Action 导航至充电桩 | FR-06 | halt 时取消 Nav2 goal |
| `SimulateCharging` | `StatefulActionNode` | 充电过程中逐渐恢复电量 | FR-05, FR-07 | halt 时立即结束 |
| `HandleAlarm` | `SyncActionNode` | 根据严重性分级处理：warning 写日志；critical 写日志+设 patrol_state=PAUSED | FR-10 | 无 |
| `SavePatrolContext` | `SyncActionNode` | 将当前路线编号和巡逻点编号保存至 `saved_route_index` / `saved_waypoint_idx` | FR-06 | 无 |
| `LoadRoutes` | `SyncActionNode` | 将 ConfigLoader 解析好的路线数据导入黑板 | FR-01 | 无 |
| `SetRouteContext` | `SyncActionNode` | 设置 `current_route` 和 `current_waypoint_index` | — | 无 |

**不需要单独节点的功能**：
- **始停控制**——通过 ROS2 Service 修改黑板变量 `patrol_state`，配合顶层 Condition 节点实现，不依赖 BT 内部节点。

---

## 4. 状态机设计

### 4.1 状态定义

巡逻系统的运行状态通过黑板变量 `patrol_state` 维护，共 6 个状态：

| 枚举值 | 状态 | 含义 |
|-------|------|------|
| 0 | IDLE | 系统就绪，尚未启动巡逻 |
| 1 | PATROLLING | 正在按路线执行巡逻任务 |
| 2 | PAUSED | 暂停在原地，等待恢复指令 |
| 3 | CHARGING | 导航至充电桩并充电中 |
| 4 | STOPPED | 任务已终止，不再 tick 行为树 |
| 5 | — | (预留) |

### 4.2 状态转移

```
                      start
        ┌──────────────────────────────────┐
        │                                  │
        ▼                                  │
┌───────────┐    start    ┌───────────────┴──┐
│           │────────────>│                  │
│   IDLE    │             │   PATROLLING     │
│           │<────────────│                  │
└───────────┘    stop     └──┬──────┬────┬──┘
      ▲                      │      │    │
      │   stop               │      │    │ low_battery
      │                      │      │    │
      │          pause       │      │   ┌▼──────────┐
      │          /critical   │      │   │           │
      │  ┌───────────┐<──────┘      │   │ CHARGING  │
      │  │           │              │   │           │
      │  │  PAUSED   │              │   └─────┬─────┘
      │  │           │              │         │
      │  └─────┬─────┘              │         │ stop
      │        │ resume             │         │
      │        └────────────────────┘         │
      │                                       │
      │          ┌───────────┐                │
      └──────────│  STOPPED  │<───────────────┘
                 │           │
                 └───────────┘
                    (终态)
```

**转移规则表**：

| 源状态 | 事件 | 目标状态 | 说明 |
|-------|------|---------|------|
| IDLE | ROS2 Service `start` | PATROLLING | 启动巡逻 |
| PATROLLING | ROS2 Service `pause` | PAUSED | 暂停在原地 |
| PATROLLING | critical 异常 | PAUSED | 报警后暂停等待指令 |
| PATROLLING | 电量 < low_threshold | CHARGING | BT 抢占切换到充电子树 |
| PATROLLING | ROS2 Service `stop` | STOPPED | 终止任务 |
| PAUSED | ROS2 Service `resume` | PATROLLING | 恢复巡逻 |
| CHARGING | 电量 >= recovery_threshold | PATROLLING | 充电完成，从断点恢复巡逻 |
| CHARGING | ROS2 Service `stop` | STOPPED | 终止任务（充电中强制中止） |
| 任意 | ROS2 Service `stop` | STOPPED | 从任意状态强制终止 |
| STOPPED | ROS2 Service `start` | IDLE | 重新就绪 |

---

## 5. 接口设计

### 5.1 Service 接口

所有 Service 使用 `std_srvs/srv/Trigger` 标准类型，无需自定义 srv。

| Service 名称 | 类型 | 请求 | 响应 |
|-------------|------|------|------|
| `/patrol/start_patrol` | `std_srvs/srv/Trigger` | — | `bool success`, `string message` |
| `/patrol/pause_patrol` | `std_srvs/srv/Trigger` | — | `bool success`, `string message` |
| `/patrol/resume_patrol` | `std_srvs/srv/Trigger` | — | `bool success`, `string message` |
| `/patrol/stop_patrol` | `std_srvs/srv/Trigger` | — | `bool success`, `string message` |

**状态约束**：

| Service | 合法源状态 | 非法源状态时的响应 |
|--------|----------|------------------|
| start | IDLE | `success=false, message="已在巡逻中"` |
| pause | PATROLLING | `success=false, message="当前不在巡逻中"` |
| resume | PAUSED | `success=false, message="当前未暂停"` |
| stop | 任意非 STOPPED 状态 | `success=false, message="已停止"` |

**实现方式**：Service 回调直接修改 BT 黑板中的 `patrol_state` 变量，BT 引擎在下一次 tick 时感知状态变化并通过抢占机制响应。

### 5.2 Topic 接口

| Topic 名称 | 消息类型 | 发布频率 | 用途 |
|-----------|---------|---------|------|
| `/patrol/status` | `PatrolStatus` | 1 Hz | 心跳 + 状态监控 |
| `/patrol/alarm` | `PatrolAlarm` | 事件触发 | 报警通知 |

**设计理由**：
- `/patrol/status` 采用 1Hz 周期发布：提供心跳机制（外部监控可通过断流检测节点异常），便于调试和监控面板实时刷新。消息体约 20 字节，开销可忽略。
- `/patrol/alarm` 仅事件触发：报警信息不容延迟，一次发布即可，无需周期重发。

### 5.3 Action Client 接口

巡逻节点作为客户端调用 Nav2 提供的 Action：

| Action 名称 | 类型 | 用途 |
|------------|------|------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 导航至指定坐标 |

### 5.4 自定义消息定义

```python
# 文件: msg/PatrolStatus.msg
uint8 IDLE=0
uint8 PATROLLING=1
uint8 PAUSED=2
uint8 CHARGING=3
uint8 STOPPED=4
uint8 state
int32 current_route_index       # 当前路线编号 (从 0 开始), -1 表示无
int32 current_waypoint_index   # 当前巡逻点编号 (从 0 开始), -1 表示无
float32 battery_level          # 电量百分比 (0.0-100.0)

# 文件: msg/PatrolAlarm.msg
uint8 WARNING=0
uint8 CRITICAL=1
uint8 severity
string alarm_type               # alarm type, e.g. "smoke_detected", "temperature_high"
float32 x                       # 触发位置 x 坐标
float32 y                       # 触发位置 y 坐标
builtin_interfaces/msg/Time timestamp   # 触发时间
```

---

## 6. 数据设计

### 6.1 黑板变量

BT 黑板是行为树节点间的中心数据总线，完全由 ConfigLoader 初始化，运行时由各节点读写。

| 变量名 | 类型 | 来源 | 说明 |
|-------|------|------|------|
| `patrol_state` | `int` | Service 回调 / BT Action | 当前系统状态 (0-4) |
| `battery_level` | `double` | BatteryModel 定时更新 | 当前电量百分比 |
| `low_threshold` | `double` | ConfigLoader 加载 | 低电阈值 |
| `recovery_threshold` | `double` | ConfigLoader 加载 | 充电恢复阈值 |
| `patrol_routes` | `vector<Route>` | ConfigLoader 加载 | 所有路线数据 |
| `current_route_index` | `int` | BT Action 节点更新 | 当前执行的路线编号 |
| `current_waypoint_index` | `int` | BT Action 节点更新 | 当前执行的巡逻点编号 |
| `current_waypoint` | `Waypoint` | BT Action 节点更新 | 当前巡逻点数据 (x, y, yaw, wait_seconds) |
| `charging_station` | `Pose2D` | ConfigLoader 加载 | 充电桩坐标 |
| `nav_timeout` | `double` | ConfigLoader 加载 | 导航超时秒数 |
| `saved_route_index` | `int` | SavePatrolContext Action | 中断时保存的路线编号 |
| `saved_waypoint_idx` | `int` | SavePatrolContext Action | 中断时保存的巡逻点编号 |

**内部数据结构**（非 ROS 消息，仅 C++ 内部使用）：

```cpp
// 坐标 (内部使用，非 ROS 消息)
struct Pose2D {
    double x, y, yaw;
};

// 巡逻点
struct Waypoint {
    Pose2D pose;
    double wait_seconds;
    // alarm_simulate 为可选字段
    bool has_alarm;
    std::string alarm_type;
    std::string alarm_severity;  // "warning" | "critical"
};

// 路线
struct Route {
    std::string name;
    int priority;
    std::vector<Waypoint> waypoints;
};
```

### 6.2 YAML 配置文件

文件路径：`config/patrol_config.yaml`

```yaml
charging_station:
  x: 1.5
  y: 0.8
  yaw: 0.0

battery:
  initial_level: 100.0
  low_threshold: 20.0
  recovery_threshold: 95.0
  discharge_rate:
    moving: 0.5
    idle: 0.05
  charge_rate: 2.0

patrol_routes:
  - name: "主通道巡检"
    priority: 1
    waypoints:
      - x: 2.0
        y: 1.0
        yaw: 0.0
        wait_seconds: 5
      - x: 5.0
        y: 3.0
        yaw: 1.57
        wait_seconds: 3
        alarm_simulate:
          type: smoke_detected
          severity: warning

  - name: "办公区巡检"
    priority: 2
    waypoints:
      - x: -2.0
        y: 4.0
        yaw: 3.14
        wait_seconds: 8
        alarm_simulate:
          type: temperature_high
          severity: critical
      - x: 0.0
        y: 6.0
        yaw: -1.57
        wait_seconds: 4

nav2:
  waypoint_timeout: 120.0
```

**字段说明**：

| 配置段 | 字段 | 类型 | 必填 | 说明 |
|-------|------|------|------|------|
| `charging_station` | `x`, `y`, `yaw` | float | 是 | 充电桩在 map 坐标系下的位姿 |
| `battery` | `initial_level` | float | 是 | 初始电量 % |
| `battery` | `low_threshold` | float | 是 | 触发电量 % |
| `battery` | `recovery_threshold` | float | 是 | 恢复离开充电桩电量 % |
| `battery` | `discharge_rate.moving` | float | 是 | 行进中每秒消耗 % |
| `battery` | `discharge_rate.idle` | float | 是 | 停留中每秒消耗 % |
| `battery` | `charge_rate` | float | 是 | 充电时每秒恢复 % |
| `patrol_routes` | `name` | string | 是 | 路线名称 |
| `patrol_routes` | `priority` | int | 是 | 优先级 (越小越优先) |
| `patrol_routes[].waypoints` | `x`, `y`, `yaw` | float | 是 | 巡逻点位姿 |
| `patrol_routes[].waypoints` | `wait_seconds` | float | 是 | 停留时长 |
| `patrol_routes[].waypoints` | `alarm_simulate` | dict | 否 | 模拟异常配置 |
| `nav2` | `waypoint_timeout` | float | 是 | 导航超时秒数 |

### 6.3 日志文件格式

日志文件路径：`logs/patrol_YYYY-MM-DD_HH-MM-SS.log`

采用结构化文本格式，每行一条日志：

```
[2026-05-25 14:30:01] [INFO] [STATE] Patrol started. State: IDLE -> PATROLLING
[2026-05-25 14:30:15] [INFO] [ROUTE] Starting route: "主通道巡检" (index: 0)
[2026-05-25 14:30:45] [INFO] [WAYPOINT] Arrived at waypoint 0 of route 0. Position: (2.0, 1.0). Waiting 5.0s.
[2026-05-25 14:30:50] [INFO] [WAYPOINT] Leaving waypoint 0 of route 0.
[2026-05-25 14:32:10] [WARN] [ALARM] Anomaly detected at waypoint 1, route 0: smoke_detected (warning). Position: (5.0, 3.0)
[2026-05-25 14:35:00] [INFO] [BATTERY] Battery low (19.5%), switching to charging.
[2026-05-25 14:35:00] [INFO] [CHARGING] Context saved: route=0, waypoint=2
[2026-05-25 14:36:30] [INFO] [CHARGING] Navigating to charging station (1.5, 0.8)
[2026-05-25 14:38:00] [INFO] [CHARGING] Charging started at 18.2%
[2026-05-25 14:40:00] [INFO] [CHARGING] Charging complete at 95.0%. Resuming patrol.
[2026-05-25 14:40:15] [INFO] [ROUTE] Resuming route 0 from waypoint 2.
```

**日志级别映射**：

| BT 事件 | 日志级别 | 控制台输出 | 文件写入 |
|--------|---------|-----------|---------|
| 巡逻开始/结束、充电开始/结束 | INFO | ✓ | ✓ |
| 巡逻点到达/离开 | INFO | ✓ | ✓ |
| warning 异常报警 | WARN | ✓ | ✓ |
| critical 异常报警 | ERROR | ✓ | ✓ |
| 导航超时跳过 | WARN | ✓ | ✓ |
| 电量低于阈值 | WARN | ✓ | ✓ |

### 6.4 照片文件命名

格式：`images/route{R}_wp{W}_{YYYYMMDD_HHMMSS}.png`

示例：`images/route0_wp2_20260525_143045.png`

- `{R}`：路线编号（从 0 开始）
- `{W}`：巡逻点编号（从 0 开始）
- `{YYYYMMDD_HHMMSS}`：拍照时间戳

---

## 7. 模块设计

### 7.1 系统节点 (patrol_bot_node)

**文件**：`src/patrol_bot_node.cpp`

**职责**：ROS2 节点入口，负责初始化、协调各模块，驱动 BT 引擎主循环。

**关键行为**：
1. 初始化 ROS2 节点
2. 创建 ConfigLoader 实例，加载 YAML 配置
3. 创建 BatteryModel、AlarmManager、PatrolLogger 实例
4. 初始化 BT 工厂，注册所有自定义 Condition/Action 节点
5. 加载 BT XML 文件，创建行为树实例
6. 将配置数据写入黑板
7. 创建 Service Server（start/pause/resume/stop）
8. 创建 Topic Publisher（/patrol/status, /patrol/alarm）
9. 启动主循环：`ros2::spin()` + 定时 tick BT 引擎
10. 注册 1Hz 定时器发布 `/patrol/status`
11. 注册 BatteryModel 定时器（每秒更新电量）

**主循环伪代码**：
```cpp
// 定时器: 20ms tick BT
auto bt_timer = create_wall_timer(50ms, [this]() {
    if (patrol_state_ == STOPPED) return;
    bt_tree_->tickRoot();
});
```

### 7.2 配置加载器 (ConfigLoader)

**文件**：`src/config_loader.cpp`, `include/patrol_bot/config_loader.hpp`

**职责**：从 YAML 文件解析配置，返回结构化数据。

**接口**：
```cpp
class ConfigLoader {
public:
    struct Config {
        Pose2D charging_station;
        BatteryConfig battery;
        std::vector<Route> routes;
        double waypoint_timeout;
    };

    static Config load(const std::string& yaml_path);

private:
    static Route parse_route(const YAML::Node& node);
    static Waypoint parse_waypoint(const YAML::Node& node);
};
```

**依赖**：`yaml-cpp` 库。

### 7.3 电池模型 (BatteryModel)

**文件**：`src/battery_model.cpp`, `include/patrol_bot/battery_model.hpp`

**职责**：模拟电池电量消耗与恢复，不依赖 ROS2 通信（不发布独立 Topic），仅更新黑板变量。

**状态机**：

```
         patrol_state == PATROLLING
         ┌──────────────────┐
         ▼                  │
    ┌─────────┐     ┌───────┴──────┐
    │ MOVING  │     │    IDLE      │
    │(行进消耗)│     │  (停留消耗)   │
    └─────────┘     └──────────────┘
                            │
         patrol_state == CHARGING
         ┌──────────────────┘
         ▼
    ┌─────────┐
    │CHARGING │
    │(恢复)    │
    └─────────┘
```

**接口**：
```cpp
class BatteryModel {
public:
    BatteryModel(double initial_level,
                 double low_threshold,
                 double recovery_threshold,
                 double moving_rate,
                 double idle_rate,
                 double charge_rate);

    // 每秒调用一次，根据 patrol_state 更新电量
    void update(int patrol_state, BT::Blackboard::Ptr blackboard);

    double level() const;

private:
    double level_;
    double low_threshold_;
    double recovery_threshold_;
    double moving_rate_;
    double idle_rate_;
    double charge_rate_;
};
```

**电量计算规则**：
- `patrol_state == PATROLLING` 且导航中（is_navigating flag）：每秒 `level -= moving_rate`
- `patrol_state == PATROLLING` 且停留中：每秒 `level -= idle_rate`
- `patrol_state == CHARGING`：每秒 `level += charge_rate`，最大 100.0
- 其他状态：电量不变

### 7.4 报警管理器 (AlarmManager)

**文件**：`src/alarm_manager.cpp`, `include/patrol_bot/alarm_manager.hpp`

**职责**：处理报警分级逻辑，发布 `/patrol/alarm` Topic，写入日志。

**接口**：
```cpp
class AlarmManager {
public:
    AlarmManager(rclcpp::Publisher<PatrolAlarm>::SharedPtr publisher,
                 PatrolLogger& logger);

    // 处理单次报警
    void raise_alarm(const std::string& type,
                     const std::string& severity,
                     double x, double y);

private:
    rclcpp::Publisher<PatrolAlarm>::SharedPtr alarm_pub_;
    PatrolLogger& logger_;
};
```

**处理流程**：
```
raise_alarm(type, severity, x, y)
    │
    ├── 构造 PatrolAlarm 消息
    ├── alarm_pub_->publish()    // 发布到 Topic
    ├── logger.log(severity)    // 写入日志
    │
    └── 若 severity == "critical"
        └── 设置 blackboard: patrol_state = PAUSED
```

### 7.5 巡逻日志 (PatrolLogger)

**文件**：`src/patrol_logger.cpp`, `include/patrol_bot/patrol_logger.hpp`

**职责**：统一日志接口，同时输出到控制台和文件。

**接口**：
```cpp
class PatrolLogger {
public:
    explicit PatrolLogger(const std::string& log_dir);

    void info(const std::string& tag, const std::string& message);
    void warn(const std::string& tag, const std::string& message);
    void error(const std::string& tag, const std::string& message);

private:
    std::ofstream file_;
    std::mutex mutex_;  // 线程安全写入
    std::string format(const std::string& level,
                       const std::string& tag,
                       const std::string& message);
};
```

**实现细节**：
- 使用 `std::mutex` 保护文件写入（BT 节点可能在多线程环境下运行）
- 日志文件按任务轮次命名：`logs/patrol_YYYY-MM-DD_HH-MM-SS.log`
- 每次写操作立即 `flush()`，确保崩溃时日志不丢失

### 7.6 导航客户端 (Nav2ActionClient)

**文件**：`src/nav2_action_client.cpp`, `include/patrol_bot/nav2_action_client.hpp`

**职责**：封装 Nav2 `NavigateToPose` Action 调用，提供超时和取消能力。

**接口**：
```cpp
class Nav2ActionClient {
public:
    Nav2ActionClient(rclcpp::Node::SharedPtr node, double timeout_sec);

    // 阻塞等待导航结果，超时返回 FAILURE
    BT::NodeStatus navigate_to_pose(double x, double y, double yaw);

    // 取消当前导航
    void cancel();

    bool is_navigating() const;

private:
    rclcpp::Client<NavigateToPose>::SharedPtr client_;
    double timeout_sec_;
    bool is_navigating_;
};
```

**行为**：
- `navigate_to_pose()` 发送 goal 后阻塞等待，每 100ms 轮询一次结果
- 超时后自动取消 goal，返回 FAILURE
- `cancel()` 调用 Nav2 `cancel_goal`，在 BT halt 时被调用
- 导航成功时返回 SUCCESS，导航失败（Nav2 返回 ABORTED/FAILED）返回 FAILURE

---

## 8. 关键场景流程

### 8.1 正常巡逻循环

```
用户: ros2 service call /patrol/start_patrol std_srvs/srv/Trigger

1. Service 回调: patrol_state = PATROLLING
2. BT Tick: ReactiveSequence 顶层两个 Condition 均为 SUCCESS
3. ReactiveFallback: 充电分支 IsBatteryLow = FAILURE → 走巡逻分支
4. LoadRoutes: 从黑板加载路线数据
5. Loop 路线 0:
   ├── SetRouteContext: current_route = routes[0]
   ├── Loop 巡逻点 0:
   │   ├── NavigateToWaypoint → Nav2 /navigate_to_pose → RUNNING → SUCCESS
   │   ├── WaitAtWaypoint → RUNNING(等待5s) → SUCCESS
   │   ├── CaptureImage → 拍照保存
   │   └── HasAlarm: 无异常 → 跳过 → SUCCESS
   ├── NavigateToWaypoint → Nav2 → SUCCESS
   ├── ...（巡逻点1有 smoke_detected warning）
   │   └── ReactiveFallback:
   │       ├── HasAlarm=SUCCESS, IsAlarmCritical=FAILURE → 不走 critical
   │       └── HasAlarm=SUCCESS, HandleAlarm(warning) → 写日志 → SUCCESS
   └── 路线完成
6. Loop 路线 1: ...（同路线 0）
7. 全部路线完成 → Loop 回到路线 0 → 无限循环
```

### 8.2 低电回充与恢复

```
前置: 执行路线 0, 巡逻点 2 导航中
触发: 电量降到 19.5%（< 20% threshold）

1. BT Tick：
   ├── 顶层 ReactiveSequence: 前两个 Condition 仍 SUCCESS
   └── ReactiveFallback:
       └── 检查充电子树第一个 Condition IsBatteryLow → SUCCESS
           ReactiveFallback halt 巡逻分支中正在 RUNNING 的 NavigateToWaypoint
           (Nav2 goal 被 cancel)

2. 充电子树 Sequence 执行:
   ├── SavePatrolContext: saved_route=0, saved_waypoint=2
   ├── NavigateToCharger: Nav2 导航至 charging_station → SUCCESS
   └── SimulateCharging:
       ├── patrol_state = CHARGING
       ├── 每秒 battery_level += 2.0%
       └── 当 battery_level >= 95.0% → SUCCESS

3. 充电完成:
   ├── ReactiveFallback: IsBatteryLow=FAILURE → 走巡逻分支
   ├── 巡逻分支:
   │   └── LoadRoutes: 根据 saved_route 和 saved_waypoint 恢复上下文
   │       current_route_index = 0, current_waypoint_index = 2
   └── 从路线 0 巡逻点 2 继续巡逻
```

### 8.3 Critical 异常中断

```
前置: 执行路线 1, 巡逻点 0, 配置了 temperature_high (critical)
触发: 巡逻点停留结束后

1. ReactiveFallback (异常处理子树):
   ├── HasAlarm → SUCCESS
   ├── IsAlarmCritical → SUCCESS
   └── HandleAlarm(critical):
       ├── alarm_pub_->publish()  # 发布 PatrolAlarm CRITICAL
       ├── logger.error()         # 写入日志
       └── blackboard: patrol_state = PAUSED

2. 下一次 BT Tick:
   └── 顶层 ReactiveSequence: 第二个 Condition (patrol_state != PAUSED) → FAILURE
       └── halt 所有运行中子节点
       └── 树不继续执行，等待 resume 指令

3. 操作者: ros2 service call /patrol/resume_patrol
   ├── Service 回调: patrol_state = PATROLLING
   └── BT Tick: 第二个 Condition → SUCCESS, 从下一个巡逻点继续
```

### 8.4 导航超时跳过

```
前置: 巡逻点导航中，被障碍物阻挡已超过 waypoint_timeout (120s)

1. NavigateToWaypoint Action 内部逻辑:
   ├── 发送 Nav2 goal
   ├── 每 100ms 检查结果
   ├── 120s 后仍未完成:
   │   ├── cancel Nav2 goal
   │   ├── logger.warn("Navigate timeout at route X waypoint Y")
   │   └── 返回 FAILURE

2. BT 中:
   └── NavigateToWaypoint FAILURE → 上层 Sequence 变为 FAILURE
       → Loop 进入下一轮 → 执行下一个巡逻点
```

### 8.5 充停控制指令

**start_patrol**:
```
Service 回调:
  if patrol_state == IDLE: patrol_state = PATROLLING
  else: 返回 success=false
```

**pause_patrol**:
```
Service 回调:
  if patrol_state == PATROLLING: patrol_state = PAUSED
  else: 返回 success=false

BT 响应: 顶层第二个 Condition 变 FAILURE → halt 所有动作 → 停在原地
```

**resume_patrol**:
```
Service 回调:
  if patrol_state == PAUSED: patrol_state = PATROLLING
  else: 返回 success=false

BT 响应: 顶层第二个 Condition 变 SUCCESS → 继续执行
```

**stop_patrol**:
```
Service 回调:
  if patrol_state != STOPPED: patrol_state = STOPPED
  else: 返回 success=false

BT 响应: 顶层第一个 Condition 变 FAILURE → halt 整个树 → 不再 tick
```

---

## 9. 目录结构

```
PatrolBot-Indoor/
├── config/
│   ├── patrol_config.yaml          # 巡逻配置文件
│   └── nav2_params.yaml            # Nav2 参数（地图加载、规划器等）
├── bt_xml/
│   └── patrol_tree.xml             # 行为树 XML 定义
├── include/
│   └── patrol_bot/
│       ├── config_loader.hpp       # ConfigLoader 头文件
│       ├── battery_model.hpp       # BatteryModel 头文件
│       ├── alarm_manager.hpp       # AlarmManager 头文件
│       ├── patrol_logger.hpp       # PatrolLogger 头文件
│       ├── nav2_action_client.hpp  # Nav2ActionClient 头文件
│       └── patrol_types.hpp        # 内部数据结构定义 (Pose2D, Waypoint, Route)
├── msg/
│   ├── PatrolStatus.msg            # 自定义消息: 状态播报
│   └── PatrolAlarm.msg             # 自定义消息: 报警通知
├── src/
│   ├── patrol_bot_node.cpp         # 主节点入口
│   ├── config_loader.cpp           # YAML 配置加载
│   ├── battery_model.cpp           # 电池电量模拟
│   ├── alarm_manager.cpp           # 报警管理器
│   ├── patrol_logger.cpp           # 日志系统
│   ├── nav2_action_client.cpp      # Nav2 Action 封装
│   ├── bt_nodes/
│   │   ├── conditions/
│   │   │   ├── is_battery_low.cpp
│   │   │   ├── is_charging_complete.cpp
│   │   │   ├── has_alarm.cpp
│   │   │   └── is_alarm_critical.cpp
│   │   └── actions/
│   │       ├── navigate_to_waypoint.cpp
│   │       ├── wait_at_waypoint.cpp
│   │       ├── capture_image.cpp
│   │       ├── navigate_to_charger.cpp
│   │       ├── simulate_charging.cpp
│   │       ├── handle_alarm.cpp
│   │       ├── save_patrol_context.cpp
│   │       ├── load_routes.cpp
│   │       └── set_route_context.cpp
├── launch/
│   ├── patrol_simulation.launch.py  # Gazebo + Nav2 + 巡逻节点 启动脚本
│   └── patrol_bot.launch.py         # 仅巡逻节点启动脚本
├── scripts/
│   ├── start_patrol.sh              # 终端快捷控制脚本
│   ├── pause_patrol.sh
│   ├── resume_patrol.sh
│   └── stop_patrol.sh
├── test/
│   └── ...                          # 单元测试和集成测试
├── logs/                            # 日志输出目录（运行时创建）
├── images/                          # 照片存档目录（运行时创建）
├── doc/
│   ├── proposal.md                  # 需求文档
│   └── high-level-design.md         # 本概要设计文档
├── CMakeLists.txt
├── package.xml
└── Dockerfile
```

---

## 10. 依赖分析

### 10.1 模块间依赖

```
patrol_bot_node
├── ConfigLoader       (启动时加载 YAML, 返回结构化配置)
├── BatteryModel       (运行时每秒更新电量, 写入黑板)
├── AlarmManager       (BT HandleAlarm 节点调用, 发布 Topic + 写日志)
├── PatrolLogger       (各模块统一日志, 控制台 + 文件双输出)
├── Nav2ActionClient   (NavigateToWaypoint / NavigateToCharger 封装)
├── BT Engine          (BehaviorTree.CPP 运行时)
│   ├── BT XML         (行为树定义文件)
│   └── BT 叶子节点
│       ├── Condition 节点 (读黑板)
│       └── Action 节点   (读/写黑板, 调用 AlarmManager/Nav2ActionClient)
└── ROS2 接口层
    ├── Service Server (修改黑板 patrol_state)
    └── Topic Publisher (/patrol/status, /patrol/alarm)
```

### 10.2 关键时序依赖

| 阶段 | 依赖项 | 就绪条件 |
|------|-------|---------|
| 系统启动 | Nav2 节点 | Nav2 Action Server `/navigate_to_pose` 就绪 |
| 系统启动 | Gazebo | 机器人模型、传感器已 spawn |
| 系统启动 | 地图 | occupancy grid map 已加载 |
| 巡逻开始 | 配置加载 | YAML 文件存在且格式正确 |
| 导航请求 | Nav2 | Action Server 可达，地图已匹配当前机器人位置 |
| 拍照 | 摄像头 | 传感器 Topic 正常发布 |

### 10.3 外部依赖库

| 库 | 版本 | 用途 |
|---|------|------|
| BehaviorTree.CPP | v4.x | 行为树引擎 |
| yaml-cpp | 0.x | YAML 配置文件解析 |
| nav2_msgs | Humble | Nav2 Action 接口消息类型 |
| rclcpp | Humble | ROS2 C++ 客户端库 |
| std_msgs / std_srvs | Humble | 标准消息和 Service 类型 |
| OpenCV | 4.x | 摄像头图像采集和保存 |
| cv_bridge | Humble | ROS2 Image 消息 ↔ OpenCV 转换 |
