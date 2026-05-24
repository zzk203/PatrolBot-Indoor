# M7: 异常处理与日志模块

> 目标：完善异常处理策略，添加日志与报警发布，确保系统在异常时仍能继续运行（fail-safe）。

## 概述

M7 模块为 PatrolBot 巡逻系统增加了全面的异常处理与日志告警能力。通过统一的 `/patrol_alerts` 话题，所有异常事件被集中记录和发布，便于运维监控和事后分析。

## 新增组件

### M7.1 LogAlertNode

**文件**: `include/nav2_demo/bt_nodes/log_alert_node.hpp`, `src/bt_nodes/log_alert_node.cpp`

行为树同步动作节点，负责：
- 接受 `alert_msg`（string）和 `severity`（string: ERROR/WARN/INFO）输入
- 根据 severity 选择 RCLCPP_ERROR/WARN/INFO 输出日志
- 发布消息到 `/patrol_alerts` 话题（std_msgs::msg::String）
- **始终返回 SUCCESS**（fail-safe 设计）

**黑板端口**:
| 端口 | 方向 | 类型 | 默认值 | 说明 |
|------|------|------|--------|------|
| alert_msg | input | string | - | 告警消息内容 |
| severity | input | string | ERROR | 日志级别 (ERROR/WARN/INFO) |

### M7.5 AmclPoseMonitorNode

**文件**: `include/nav2_demo/bt_nodes/amcl_pose_monitor_node.hpp`, `src/bt_nodes/amcl_pose_monitor_node.cpp`

行为树条件节点，监控 AMCL 定位质量：
- 订阅 `/amcl_pose` 话题
- 提取协方差矩阵对角元素（x, y, yaw 的方差）
- 使用指数平滑防止瞬态抖动误报
- 协方差超阈值时发布告警并返回 FAILURE

**黑板端口**:
| 端口 | 方向 | 类型 | 默认值 | 说明 |
|------|------|------|--------|------|
| covariance_threshold | input | double | 0.5 | 协方差阈值 |
| localization_valid | output | bool | - | 定位是否有效 |

## 修改的组件

### M7.2 巡逻堵赛日志集成

**文件**: `behavior_trees/patrol_round.xml`

在 `patrol_round.xml` 的 Fallback 备选分支中，串联 LogAlertNode 到 RecordFailureNode 之后：
```
Fallback
├── NavigateToPoseNode (主导航)
└── Sequence
    ├── RecordFailureNode (记录失败)
    └── LogAlertNode (发布 "Waypoint X blocked" 告警)
```

### M7.3 对接失败日志集成

**文件**: `include/nav2_demo/bt_nodes/dock_action_node.hpp`, `src/bt_nodes/dock_action_node.cpp`

在 DockActionNode 中集成失败告警：
- 导航阶段失败时发布 "Docking navigation failed"
- 伺服阶段重试 3 次全部耗尽时发布 "Docking failed after 3 retries"
- 通过内部 `publishAlert()` 辅助函数直接发布到 `/patrol_alerts`

### M7.4 电池数据丢失检测

**文件**: `include/nav2_demo/bt_nodes/battery_monitor_node.hpp`, `src/bt_nodes/battery_monitor_node.cpp`

在 BatteryMonitorNode 中增加话题超时检测：
- 记录最后一条消息的时间戳 `last_msg_time_`
- 在 tick() 中检查是否超过 5 秒未收到新数据
- 超时时：发布 "Battery data lost (topic timeout)" 告警
- 超时时：将电池视为满电（100%），低电标志设为 false
- **返回 SUCCESS** 确保主流程不中断

## Fail-Safe 设计

所有异常处理节点遵循以下原则：
1. **LogAlertNode**：始终返回 SUCCESS
2. **BatteryMonitorNode**：超时时返回 SUCCESS（满电）
3. **AmclPoseMonitorNode**：返回 FAILURE 但不抛出异常
4. **行为树层**：使用 `Fallback{... | AlwaysSuccess}` 确保异常不阻塞流程

## 异常路径汇总

| 异常场景 | 检测节点 | 处理方式 | 告警消息 |
|----------|----------|----------|----------|
| 航点导航超时 | NavigateToPoseNode → RecordFailureNode → LogAlertNode | 跳过该点继续巡逻 | "Waypoint X blocked" |
| 对接导航失败 | DockActionNode | 进入 FAILED 状态 | "Docking navigation failed" |
| 伺服重试耗尽 | DockActionNode (3次重试后) | 进入 FAILED 状态 | "Docking failed after 3 retries" |
| 电池话题超时 | BatteryMonitorNode (>5s 无数据) | 视为满电继续 | "Battery data lost (topic timeout)" |
| 定位协方差过大 | AmclPoseMonitorNode | 标记定位无效 | "Localization quality degraded..." |

## 测试覆盖

| 测试 | 文件 | 覆盖数量 |
|------|------|----------|
| LogAlertNode 话题输出验证 | test_m7_exceptions.cpp | 5 |
| 堵塞日志集成 | test_m7_exceptions.cpp | 2 |
| 对接失败告警 | test_m7_exceptions.cpp | 1 |
| 电池超时处理 | test_m7_exceptions.cpp | 3 |
| 定位丢失检测 | test_m7_exceptions.cpp | 3 |
| Fail-safe 不破坏主流程 | test_m7_exceptions.cpp | 2 |

运行测试：
```bash
cd patrol_robot_ws
colcon build --packages-select nav2_demo --cmake-force-configure
colcon test --packages-select nav2_demo --ctest-args -R test_m7_exceptions
```
