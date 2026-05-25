# 05. 报警管理器 (AlarmManager)

> 对应需求: FR-08 环境异常模拟、FR-09 系统状态异常、FR-10 报警分级响应
> 参考设计: detailed_design.md §2.4

## 5.1 头文件

- [ ] 创建 `include/patrol_bot/alarm_manager.hpp`
- [ ] 声明 `AlarmManager` 类
- [ ] 声明构造函数 `AlarmManager(rclcpp::Node* node, PatrolLogger& logger)`
- [ ] 声明 `void raise_alarm(type, severity, x, y)` 方法
- [ ] 声明 Publisher `<PatrolAlarm>` → `/patrol/alarm`
- [ ] 声明 `PatrolLogger& logger_` 引用成员

## 5.2 实现

- [ ] 创建 `src/alarm_manager.cpp`
- [ ] 实现构造函数：创建 Publisher `/patrol/alarm`（QoS reliable, depth=10）
- [ ] 实现 `raise_alarm()`：

### 5.2.1 消息构造

- [ ] 创建 `PatrolAlarm` 消息
- [ ] 设置 `severity`：`"critical"` → `CRITICAL`，否则 → `WARNING`
- [ ] 设置 `alarm_type = type`
- [ ] 设置 `x = x`, `y = y`
- [ ] 设置 `timestamp = node_->now()`

### 5.2.2 发布与日志

- [ ] 调用 `alarm_pub_->publish(msg)` 发布 Topic
- [ ] critical 时调用 `logger_.error("ALARM", ...)` 写错误日志
- [ ] warning 时调用 `logger_.warn("ALARM", ...)` 写警告日志

### 5.2.3 注意事项

- [ ] AlarmManager 不修改 `patrol_state`（由 BT HandleAlarm 节点负责）
- [ ] 确保日志输出同时到控制台和文件

## 5.3 编译验证

- [ ] 编译通过，依赖 patrol_bot_interfaces、patrol_logger 正确链接
- [ ] 验证 raise_alarm 产生正确的 Topic 输出
