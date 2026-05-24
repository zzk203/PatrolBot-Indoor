#include "nav2_demo/battery_simulator_node.hpp"

#include <algorithm>
#include <string>

namespace nav2_demo
{

BatterySimulatorNode::BatterySimulatorNode(const rclcpp::NodeOptions & options)
: Node("battery_simulator", options)
{
  // === 声明参数 ===
  this->declare_parameter("initial_percentage", 100.0);
  this->declare_parameter("discharge_rate", 0.5);
  this->declare_parameter("charge_rate", 5.0);
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("voltage_full", 12.5);
  this->declare_parameter("voltage_empty", 10.0);

  // === 读取参数 ===
  initial_percentage_ = this->get_parameter("initial_percentage").as_double();
  discharge_rate_ = this->get_parameter("discharge_rate").as_double();
  charge_rate_ = this->get_parameter("charge_rate").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  voltage_full_ = this->get_parameter("voltage_full").as_double();
  voltage_empty_ = this->get_parameter("voltage_empty").as_double();

  // === 初始化状态 ===
  battery_percentage_ = std::clamp(initial_percentage_, 0.0, 100.0);
  last_update_ = this->now();

  // === 创建发布者 ===
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", rclcpp::QoS(10).transient_local());

  // === 创建服务 ===
  charging_service_ = this->create_service<std_srvs::srv::SetBool>(
    "/battery_simulator/set_charging",
    std::bind(&BatterySimulatorNode::setChargingCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // === 创建定时器 ===
  double period_ms = 1000.0 / publish_rate_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double, std::milli>(period_ms),
    std::bind(&BatterySimulatorNode::updateBattery, this));

  RCLCPP_INFO(this->get_logger(),
    "BatterySimulatorNode 启动: initial=%.0f%%, discharge=%.2f%%/s, "
    "charge=%.2f%%/s, rate=%.1fHz",
    battery_percentage_, discharge_rate_, charge_rate_, publish_rate_);
}

void BatterySimulatorNode::updateBattery()
{
  auto now = this->now();
  double dt = (now - last_update_).seconds();
  last_update_ = now;

  // 防止 dt 异常（首次调用或跳跃）
  if (dt <= 0.0 || dt > 10.0) {
    dt = 1.0 / publish_rate_;
  }

  // 更新电量
  if (charging_) {
    battery_percentage_ += charge_rate_ * dt;
    RCLCPP_DEBUG(this->get_logger(),
      "充电中: +%.2f%%, 当前 %.2f%%", charge_rate_ * dt, battery_percentage_);
  } else {
    battery_percentage_ -= discharge_rate_ * dt;
    RCLCPP_DEBUG(this->get_logger(),
      "放电中: -%.2f%%, 当前 %.2f%%", discharge_rate_ * dt, battery_percentage_);
  }

  // 钳制到 [0, 100]
  battery_percentage_ = std::clamp(battery_percentage_, 0.0, 100.0);

  // 构建 BatteryState 消息
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = now;
  msg.header.frame_id = "base_link";

  msg.voltage = percentageToVoltage(static_cast<float>(battery_percentage_));
  msg.temperature = 25.0;  // 常温
  msg.current = computeCurrent();
  msg.charge = static_cast<float>(battery_percentage_ / 100.0 * 10.0);  // 假设 10Ah
  msg.capacity = 10.0f;
  msg.design_capacity = 10.0f;
  msg.percentage = static_cast<float>(battery_percentage_ / 100.0);  // [0, 1]

  // 供电状态
  if (charging_) {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (battery_percentage_ < 20.0) {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  msg.present = true;

  // 发布
  battery_pub_->publish(msg);
}

void BatterySimulatorNode::setChargingCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  charging_ = request->data;
  response->success = true;
  if (charging_) {
    response->message = "Charging enabled";
    RCLCPP_INFO(this->get_logger(), "充电模式: 开启");
  } else {
    response->message = "Charging disabled";
    RCLCPP_INFO(this->get_logger(), "充电模式: 关闭");
  }
}

float BatterySimulatorNode::percentageToVoltage(float percentage) const
{
  // 线性映射: 0% → voltage_empty, 100% → voltage_full
  float ratio = percentage / 100.0f;
  float v = static_cast<float>(voltage_empty_) +
            ratio * static_cast<float>(voltage_full_ - voltage_empty_);
  return v;
}

float BatterySimulatorNode::computeCurrent() const
{
  // 简单模型: 放电时 -2.0A, 充电时 +3.0A
  return charging_ ? 3.0f : -2.0f;
}

}  // namespace nav2_demo

// ===================== main 入口 =====================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_demo::BatterySimulatorNode>();
  RCLCPP_INFO(node->get_logger(), "BatterySimulatorNode 就绪");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
