# M5: 视觉分段对接

## 概述

M5 模块实现了机器人从低电触发到充电桩自动对接充电的完整流程。采用"分段对接"策略：先使用 Nav2 导航到充电桩前方的预备位姿，然后通过模拟视觉伺服 P 控制器进行精细对准。

## 架构设计

```
┌──────────────────────────────────────────────────────┐
│                  ChargeRecovery 子树                    │
│  ┌──────────────────────────────────────────────────┐ │
│  │  Sequence                                        │ │
│  │  ├─ Script: 保存断点                             │ │
│  │  ├─ NavigateToPoseNode(dock_prep_pose)           │ │
│  │  ├─ DockActionNode(dock_pose)                    │ │
│  │  │    ├─ Phase 1: Navigate to prep_pose          │ │
│  │  │    ├─ Phase 2: VisualServo (P controller)     │ │
│  │  │    └─ Retry: max 3 attempts on servo phase    │ │
│  │  ├─ SetChargingModeNode(enabled=true)            │ │
│  │  ├─ Wait(10s)  ← 模拟充电                        │ │
│  │  ├─ SetChargingModeNode(enabled=false)           │ │
│  │  ├─ Script: is_low_battery=false, battery=100    │ │
│  │  └─ Script: 恢复断点                             │ │
│  └──────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

## 组件说明

### M5.1 DockActionNode (`BT::StatefulActionNode`)
- **功能**: 高层次的对接动作节点，封装两阶段对接流程
- **阶段 1 (NAVIGATE)**: 通过 Nav2 `navigate_to_pose` 动作客户端导航到预备位姿
- **阶段 2 (SERVO)**: 通过 P 控制器视觉伺服到充电桩位姿
- **重试**: 伺服阶段内部最多重试 3 次
- **端口**:
  - `dock_prep_pose` (input, PoseStamped): 导航预备位姿
  - `dock_pose_x/y/yaw` (input, double): 充电桩位姿
  - `navigation_timeout` (input, double): 导航超时（默认 60s）
  - `servo_timeout` (input, double): 伺服超时（默认 15s）
  - `dock_success` (output, bool): 对接成功标志

### M5.2 VisualServoNode (`BT::StatefulActionNode`)
- **功能**: 模拟视觉伺服，通过 P 控制器计算速度指令发布到 `/cmd_vel`
- **位姿源**: 默认从 `/odom` 获取机器人位姿，充电桩位姿从黑板的端口参数获取
- **可测试性**: `setRobotPoseSource()` 方法注入自定义位姿源回调
- **P 控制器**:
  - `Kp_linear = 0.3`, `Kp_angular = 0.5`
  - 线速度钳制 `±0.2 m/s`, 角速度钳制 `±0.5 rad/s`
- **成功条件**:
  - 角度偏差 `|yaw_error| < 0.05 rad`
  - 横向偏差 `|lateral_error| < 0.02 m`
  - 前进偏差 `|forward_error| < 0.15 m`
- **超时**: 默认 15 秒
- **端口**:
  - `timeout` (input, double): 超时秒数
  - `dock_pose_x/y/yaw` (input, double): 充电桩位姿
  - `dock_success` (output, bool): 对接成功标志

### M5.3 RetryNode (`BT::DecoratorNode`)
- **功能**: 包装子节点，失败时自动重试
- **配置**: `max_attempts` 端口配置最大尝试次数（默认 3）
- **报警**: 全部重试失败时记录 ERROR 级别日志
- **端口**:
  - `max_attempts` (input, int): 最大尝试次数
  - `retry_count` (output, int): 当前尝试计数

### M5.4 模拟充电完成
- 对接成功后，`SetChargingModeNode` 启用充电模式
- `Wait(10s)` 等待 10 秒模拟充电过程
- 充电完成后将 `is_low_battery` 设为 false, `battery_level` 设为 100.0

### M5.5 完整对接流程
- 主树的 `ReactiveSequence(BatteryMonitor + PatrolRound)` 在低电时触发
- `Fallback` 捕获低电后执行 `Sequence(IsBatteryLowCondition + ChargeRecovery)`
- `ChargeRecovery` 子树顺序执行：保存断点 → 导航预备位姿 → 分段对接 → 充电 → 恢复

## 测试策略

| 测试项 | 覆盖范围 |
|--------|---------|
| `VisualServoNodeTest` | 位姿注入、P 控制器、成功条件、超时处理、halt 行为 |
| `RetryNodeTest` | 重试次数、默认参数、一次成功、全部失败 |
| `RetryWithServoTest` | RetryNode + VisualServoNode 集成 |
| `ChargeRecoveryIntegrationTest` | 完整对接 + 充电 + 断点恢复流程 |

所有 VisualServoNode 测试均通过 `setRobotPoseSource()` 注入位姿源，无需 Gazebo 仿真环境。

## 新增/修改文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `include/nav2_demo/bt_nodes/visual_servo_node.hpp` | 新增 | VisualServoNode 头文件 |
| `src/bt_nodes/visual_servo_node.cpp` | 新增 | VisualServoNode 实现 |
| `include/nav2_demo/bt_nodes/retry_node.hpp` | 新增 | RetryNode 头文件 |
| `src/bt_nodes/retry_node.cpp` | 新增 | RetryNode 实现 |
| `include/nav2_demo/bt_nodes/dock_action_node.hpp` | 新增 | DockActionNode 头文件 |
| `src/bt_nodes/dock_action_node.cpp` | 新增 | DockActionNode 实现 |
| `include/nav2_demo/bt_nodes/load_waypoints_node.hpp` | 修改 | 新增 dock_prep_pose/dock_pose 输出 |
| `src/bt_nodes/load_waypoints_node.cpp` | 修改 | 实现新增输出 |
| `behavior_trees/charge_recovery.xml` | 修改 | 集成视觉分段对接 |
| `behavior_trees/patrol_main.xml` | 修改 | 新增 dock 相关黑板变量 |
| `CMakeLists.txt` | 修改 | 新增源文件和测试目标 |
| `tests/test_m5_docking.cpp` | 新增 | 完整测试套件 |
