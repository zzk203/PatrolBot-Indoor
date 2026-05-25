# 08. 行为树节点

> 对应需求: FR-02~FR-04 巡逻执行、FR-06~FR-07 回充、FR-10 异常分级、FR-15 超时跳过、FR-16 拍照
> 参考设计: detailed_design.md §3

## 8.1 目录结构

- [ ] 创建 `src/bt_nodes/` 目录
- [ ] 创建 `src/bt_nodes/conditions/` 子目录
- [ ] 创建 `src/bt_nodes/actions/` 子目录
- [ ] 创建 `bt_xml/` 目录

---

## 8.2 Condition 节点

### 8.2.1 IsBatteryLow

- [ ] 创建 `src/bt_nodes/conditions/is_battery_low.cpp`
- [ ] 实现 `IsBatteryLow` 函数（或注册为 SimpleCondition）
- [ ] 从黑板读取 `battery_level` (double) 和 `low_threshold` (double)
- [ ] 返回 `(battery_level < low_threshold) ? SUCCESS : FAILURE`

### 8.2.2 HasAlarm

- [ ] 创建 `src/bt_nodes/conditions/has_alarm.cpp`
- [ ] 从黑板读取 `current_waypoint` (Waypoint)
- [ ] 返回 `wp.has_alarm ? SUCCESS : FAILURE`

### 8.2.3 IsAlarmCritical

- [ ] 创建 `src/bt_nodes/conditions/is_alarm_critical.cpp`
- [ ] 从黑板读取 `current_waypoint` (Waypoint)
- [ ] 无异常时返回 FAILURE
- [ ] 返回 `(wp.alarm.severity == "critical") ? SUCCESS : FAILURE`

---

## 8.3 SyncAction 节点

### 8.3.1 LoadRoutes

- [ ] 创建 `src/bt_nodes/actions/load_routes.cpp`
- [ ] 类型：`SyncActionNode`
- [ ] 从黑板读取 `config` (Config)
- [ ] 向黑板写入 `patrol_routes` / `charging_station` / `low_threshold` / `recovery_threshold` / `nav_timeout`
- [ ] 调用 `logger_.info("ROUTE", ...)` 记录加载的路线数

### 8.3.2 RestorePatrolContext

- [ ] 创建 `src/bt_nodes/actions/restore_patrol_context.cpp`
- [ ] 类型：`SyncActionNode`
- [ ] 从黑板读取 `saved_route_index` 和 `saved_waypoint_idx`
- [ ] 如果有保存的断点：
  - 设置 `current_route_index = saved_route_index`
  - 设置 `current_waypoint_index = saved_waypoint_idx`
  - 从 `patrol_routes` 获取对应 `current_waypoint`
  - 清除 `saved_route_index` / `saved_waypoint_idx` = -1
- [ ] 无断点时：初始化 `current_route_index = 0`, `current_waypoint_index = 0`

### 8.3.3 SetRouteContext

- [ ] 创建 `src/bt_nodes/actions/set_route_context.cpp`
- [ ] 类型：`SyncActionNode`
- [ ] 从黑板读取 `current_route_index` / `patrol_routes`
- [ ] 向黑板写入 `current_route`
- [ ] 调用 `logger_.info("ROUTE", ...)` 记录当前路线

### 8.3.4 CaptureImage

- [ ] 创建 `src/bt_nodes/actions/capture_image.cpp`
- [ ] 类型：`SyncActionNode`
- [ ] 从黑板读取 `current_route_index` / `current_waypoint_index`
- [ ] 生成文件名：`route{R}_wp{W}_{YYYYMMDD_HHMMSS}`
- [ ] 调用 `camera_buffer_->save_latest(filename)`
- [ ] 无帧时记录 WARN 日志，返回 SUCCESS（降级，不阻塞巡逻）
- [ ] 成功时记录 INFO 日志

### 8.3.5 HandleAlarm

- [ ] 创建 `src/bt_nodes/actions/handle_alarm.cpp`
- [ ] 类型：`SyncActionNode`
- [ ] 从黑板读取 `current_waypoint` (Waypoint)
- [ ] 调用 `alarm_manager_->raise_alarm(type, severity, x, y)`
- [ ] critical 异常：向黑板写入 `patrol_state = STATE_PAUSED` + 记录 ERROR 日志
- [ ] warning 异常：记录 WARN 日志（不修改 patrol_state）

### 8.3.6 SavePatrolContext

- [ ] 创建 `src/bt_nodes/actions/save_patrol_context.cpp`
- [ ] 类型：`SyncActionNode`
- [ ] 从黑板读取 `current_route_index` / `current_waypoint_index`
- [ ] 向黑板写入 `saved_route_index` / `saved_waypoint_idx`
- [ ] 调用 `logger_.info("CHARGING", ...)` 记录断点

---

## 8.4 StatefulAction 节点

### 8.4.1 NavigateToWaypoint

- [ ] 创建 `src/bt_nodes/actions/navigate_to_waypoint.cpp`
- [ ] 类型：`StatefulActionNode`
- [ ] 实现 `onStart()`：
  - 从黑板读取 `current_waypoint.pose` / `nav_timeout`
  - 调用 `nav_client_->send_goal(x, y, yaw)`
  - 记录开始时间
  - 返回 RUNNING
- [ ] 实现 `onRunning()`：
  - 检查 `nav_client_->check_result()`
  - SUCCESS → 返回 SUCCESS
  - FAILURE → 返回 FAILURE
  - 超时 → `cancel_goal()` + WARN 日志 + 返回 FAILURE
  - 否则 → 返回 RUNNING
- [ ] 实现 `onHalted()`：调用 `nav_client_->cancel_goal()`

### 8.4.2 NavigateToCharger

- [ ] 创建 `src/bt_nodes/actions/navigate_to_charger.cpp`
- [ ] 类型：`StatefulActionNode`
- [ ] 实现 `onStart()`：
  - 从黑板读取 `charging_station`
  - 调用 `nav_client_->send_goal(...)`
  - **导航至充电桩不设超时**（必须到达）
- [ ] 实现 `onRunning()` / `onHalted()`：逻辑与 NavigateToWaypoint 相同（除无超时外）

### 8.4.3 WaitAtWaypoint

- [ ] 创建 `src/bt_nodes/actions/wait_at_waypoint.cpp`
- [ ] 类型：`StatefulActionNode`
- [ ] 实现 `onStart()`：
  - 从黑板读取 `current_waypoint.wait_seconds`
  - wait_seconds <= 0 → 直接返回 SUCCESS
  - 记录开始时间 → 返回 RUNNING
- [ ] 实现 `onRunning()`：
  - 已过 >= wait_seconds → 返回 SUCCESS
  - 否则 → 返回 RUNNING
- [ ] 实现 `onHalted()`：无需清理

### 8.4.4 SimulateCharging

- [ ] 创建 `src/bt_nodes/actions/simulate_charging.cpp`
- [ ] 类型：`StatefulActionNode`
- [ ] 实现 `onStart()`：
  - 向黑板写入 `patrol_state = STATE_CHARGING`
  - 从黑板读取 `recovery_threshold`
  - 记录开始时间 → 返回 RUNNING
- [ ] 实现 `onRunning()`：
  - 从黑板读取 `battery_level`
  - battery_level >= recovery_threshold：
    - 向黑板写入 `patrol_state = STATE_PATROLLING`
    - INFO 日志
    - 返回 SUCCESS
  - 否则 → 返回 RUNNING
- [ ] 实现 `onHalted()`：由 stop 指令中断

---

## 8.5 顶层 Condition 节点（内联注册）

- [ ] 实现 `IsNotStopped`：黑板 `patrol_state != STATE_STOPPED` → SUCCESS
- [ ] 实现 `IsNotPaused`：黑板 `patrol_state != STATE_PAUSED` → SUCCESS

---

## 8.6 行为树 XML

- [ ] 创建 `bt_xml/patrol_tree.xml`
- [ ] 顶层 `ReactiveSequence` 节点
- [ ] 添加 `IsNotStopped` Condition
- [ ] 添加 `IsNotPaused` Condition
- [ ] 添加 `ReactiveFallback` 节点
- [ ] 充电子树（Sequence）：IsBatteryLow → SavePatrolContext → NavigateToCharger → SimulateCharging
- [ ] 巡逻子树（ReactiveSequence）：LoadRoutes → RestorePatrolContext → 外层 Loop（路线循环）
- [ ] 路线循环内：SetRouteContext → 内层 Loop（巡逻点循环）
- [ ] 巡逻点循环内：NavigateToWaypoint → WaitAtWaypoint → CaptureImage → 异常处理 ReactiveFallback
- [ ] 异常处理：critical 分支（HasAlarm + IsAlarmCritical + HandleAlarm）| warning 分支（HasAlarm + HandleAlarm）

## 8.7 编译验证

- [ ] 所有 BT 节点编译通过
- [ ] 依赖 BehaviorTree.CPP v4、patrol_bot_core 正确链接
- [ ] XML 文件可被 BT Factory 正确加载
