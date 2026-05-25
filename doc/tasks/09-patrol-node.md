# 09. 系统主节点 (patrol_bot_node)

> 对应需求: FR-04 巡逻执行模式、FR-11 启停控制
> 参考设计: detailed_design.md §4, §8.3, §8.4

## 9.1 目录与入口

- [ ] 创建 `src/patrol_bot_node.cpp`
- [ ] 实现 `main()` 函数：rclcpp::init → 创建节点 → spin → shutdown

## 9.2 参数声明

- [ ] 声明 ROS2 参数 `"config_path"`（默认 `"config/patrol_config.yaml"`）

## 9.3 配置加载 (fail-fast)

- [ ] 调用 `ConfigLoader::load()` 加载配置
- [ ] 加载失败时 `RCLCPP_FATAL` 日志 + 抛异常终止

## 9.4 核心模块初始化

- [ ] 创建 `PatrolLogger` 实例（日志目录 "logs"）
- [ ] 创建 `CameraBuffer` 实例（传入摄像头配置）
- [ ] 创建 `BatteryModel` 实例（传入电池配置）
- [ ] 创建 `AlarmManager` 实例（传入 logger 引用）
- [ ] 创建 `Nav2ActionClient` 实例

## 9.5 等待 Nav2 Action Server

- [ ] 调用 `nav_client_->wait_for_server(10s)` 首次等待
- [ ] 失败时重试 3 次（每次 5s）
- [ ] 全部失败：`RCLCPP_FATAL` + 抛异常

## 9.6 黑板初始化

- [ ] 创建 `BT::Blackboard::create()`
- [ ] 写入 config、battery_level、patrol_state=STATE_IDLE
- [ ] 写入 saved_route_index=-1、saved_waypoint_idx=-1
- [ ] 写入 current_route_index=0、current_waypoint_index=0

## 9.7 BT 工厂注册

- [ ] 创建 `BT::BehaviorTreeFactory`
- [ ] 实现 `register_nodes(factory)` 函数
- [ ] 注册 IsNotStopped / IsNotPaused（内联 lambda）
- [ ] 注册 IsBatteryLow / HasAlarm / IsAlarmCritical
- [ ] 注册 LoadRoutes / RestorePatrolContext / SetRouteContext / CaptureImage / HandleAlarm / SavePatrolContext（SyncAction）
- [ ] 注册 NavigateToWaypoint / NavigateToCharger / WaitAtWaypoint / SimulateCharging（StatefulAction，使用 RegisterBuilder）

## 9.8 BT 实例化

- [ ] 调用 `factory.createTreeFromFile("bt_xml/patrol_tree.xml", blackboard_)` 加载行为树

## 9.9 ROS2 Service Server

- [ ] 创建 Service `/patrol/start_patrol`（std_srvs::srv::Trigger）
- [ ] 创建 Service `/patrol/pause_patrol`（std_srvs::srv::Trigger）
- [ ] 创建 Service `/patrol/resume_patrol`（std_srvs::srv::Trigger）
- [ ] 创建 Service `/patrol/stop_patrol`（std_srvs::srv::Trigger）

### 9.9.1 回调实现

- [ ] 实现 `handle_start()`：IDLE → PATROLLING，非法状态返回 success=false
- [ ] 实现 `handle_pause()`：PATROLLING → PAUSED，非法状态返回 success=false
- [ ] 实现 `handle_resume()`：PAUSED → PATROLLING，非法状态返回 success=false
- [ ] 实现 `handle_stop()`：任意非 STOPPED → STOPPED，非法状态返回 success=false
- [ ] 每次状态变更调用 `logger_->info("STATE", ...)` 记录日志

## 9.10 Topic 发布/订阅

- [ ] 创建 Publisher `<PatrolStatus>` → `/patrol/status`
- [ ] 创建 Subscription `<BatteryState>` → `/patrol/battery`
- [ ] 实现 `battery_callback()`：`msg.percentage * 100.0` → 写入黑板 `battery_level`
- [ ] 实现 `publish_status()`：发布 current state / route_index / waypoint_index / battery_level

## 9.11 定时器

- [ ] 创建 BT tick 定时器：50ms 周期，调用 `tree_->tickRoot()`
- [ ] STOPPED 状态时跳过 tick
- [ ] 创建 Status 发布定时器：1s 周期，调用 `publish_status()`

## 9.12 启动

- [ ] 调用 `battery_model_->start()` 启动电池模型
- [ ] 输出 `RCLCPP_INFO` 节点初始化完成日志

## 9.13 编译验证

- [ ] 编译通过，作为独立可执行文件
- [ ] 依赖 patrol_bot_core + patrol_bt_nodes 正确链接
- [ ] 节点启动后无崩溃，各模块正常初始化
