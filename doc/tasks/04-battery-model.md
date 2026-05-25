# 04. 电池模型 (BatteryModel)

> 对应需求: FR-05 电量模拟、FR-06 低电回充、FR-07 充电恢复
> 参考设计: detailed_design.md §2.3, §8.2

## 4.1 头文件

- [ ] 创建 `include/patrol_bot/battery_model.hpp`
- [ ] 声明 `BatteryModel` 类
- [ ] 声明构造函数 `BatteryModel(rclcpp::Node* node, const BatteryConfig& config)`
- [ ] 声明 `start()` / `stop()` 生命周期方法
- [ ] 声明成员变量：`node_`、`config_`、`level_`、`is_charging_`
- [ ] 声明 Publisher `<sensor_msgs::msg::BatteryState>` → `/patrol/battery`
- [ ] 声明 Subscription `<PatrolStatus>` → `/patrol/status`
- [ ] 声明 1Hz 定时器 `timer_`

## 4.2 实现

- [ ] 创建 `src/battery_model.cpp`
- [ ] 实现构造函数：
  - 初始化 `level_ = config.initial_level`
  - 创建 Publisher `/patrol/battery`（QoS reliable, depth=10）
  - 创建 Subscription `/patrol/status`（QoS reliable, depth=10）
  - 暂不启动定时器（等待 `start()` 调用）

### 4.2.1 定时器回调 (1Hz)

- [ ] 实现 `timer_callback()`
- [ ] 充电模式：`level_ = min(level_ + config_.charge_rate, 100.0)`
- [ ] 放电模式：`level_ = max(level_ - config_.moving_rate, 0.0)`
- [ ] 构造 `sensor_msgs::msg::BatteryState` 消息
  - `header.stamp = node_->now()`
  - `percentage = level_ / 100.0`
  - `power_supply_status` = CHARGING 或 DISCHARGING
  - `power_supply_health = GOOD`
- [ ] 发布消息

### 4.2.2 状态订阅回调

- [ ] 实现 `status_callback()`
- [ ] 从 `msg.state` 推演充放电模式：CHARGING(3) → `is_charging_ = true`
- [ ] 其他状态 → `is_charging_ = false`

### 4.2.3 生命周期

- [ ] 实现 `start()`：创建 1Hz 定时器
- [ ] 实现 `stop()`：取消定时器

## 4.3 编译验证

- [ ] 编译通过，依赖 sensor_msgs / patrol_bot_interfaces 正确链接
- [ ] 验证定时器回调：放电时电量下降、充电时电量上升
- [ ] 验证电量边界钳位 [0, 100]
