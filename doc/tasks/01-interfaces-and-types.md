# 01. 消息包与内部数据结构

> 对应需求: FR-08/FR-09/FR-10 报警发布、FR-11 状态发布
> 参考设计: detailed_design.md §2.1, §5

## 1.1 patrol_bot_interfaces 消息包

### 1.1.1 包目录结构
- [ ] 创建 `patrol_bot_interfaces/` 目录
- [ ] 创建 `patrol_bot_interfaces/msg/` 子目录

### 1.1.2 patrol_bot_interfaces/package.xml
- [ ] 编写 package.xml（format=3，成员组 rosidl_interface_packages）
- [ ] 声明依赖：ament_cmake、rosidl_default_generators、rosidl_default_runtime、builtin_interfaces

### 1.1.3 PatrolStatus.msg
- [ ] 定义状态枚举 IDLE=0 / PATROLLING=1 / PAUSED=2 / CHARGING=3 / STOPPED=4
- [ ] 定义字段 state / current_route_index / current_waypoint_index / battery_level

### 1.1.4 PatrolAlarm.msg
- [ ] 定义严重性枚举 WARNING=0 / CRITICAL=1
- [ ] 定义字段 severity / alarm_type / x / y / timestamp

### 1.1.5 patrol_bot_interfaces/CMakeLists.txt
- [ ] 编写 CMakeLists.txt（使用 rosidl_generate_interfaces）

### 1.1.6 编译验证
- [ ] 编译 patrol_bot_interfaces 包，确认无错误
- [ ] 验证生成的头文件（`ros2 interface show patrol_bot_interfaces/msg/PatrolStatus`）

---

## 1.2 patrol_types.hpp 内部数据结构

> 参考设计: detailed_design.md §2.1

- [ ] 创建 `include/patrol_bot/patrol_types.hpp`
- [ ] 定义 `Pose2D` 结构体（x, y, yaw）
- [ ] 定义 `AlarmConfig` 结构体（type, severity）
- [ ] 定义 `Waypoint` 结构体（pose, wait_seconds, has_alarm, alarm）
- [ ] 定义 `Route` 结构体（name, priority, waypoints）
- [ ] 定义 `BatteryConfig` 结构体（initial_level, low_threshold, recovery_threshold, moving_rate, charge_rate）
- [ ] 定义 `CameraConfig` 结构体（topic, image_format, save_directory）
- [ ] 定义 `Config` 结构体（charging_station, battery, routes, waypoint_timeout, camera）
- [ ] 编译验证 patrol_types.hpp 可被正确包含
