#include "nav2_demo/bt_nodes/battery_monitor_node.hpp"

#include <string>
#include <mutex>

namespace nav2_demo
{

BatteryMonitorNode::BatteryMonitorNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::ConditionNode(name, config)
, node_(node)
{
  // 订阅电池状态话题
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state",
    rclcpp::QoS(10).transient_local(),
    std::bind(&BatteryMonitorNode::batteryCallback, this, std::placeholders::_1));

  // M7.4: 初始化最后消息时间为当前时间
  last_msg_time_ = node_->now();

  // M7.4: 创建 /patrol_alerts 发布者
  alert_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/patrol_alerts", rclcpp::QoS(10).transient_local());

  RCLCPP_DEBUG(node_->get_logger(), "BatteryMonitorNode: 已订阅 /battery_state");
}

BT::PortsList BatteryMonitorNode::providedPorts()
{
  return {
    BT::InputPort<float>("battery_threshold", 20.0f, "低电阈值 (%)"),
    BT::OutputPort<float>("battery_level", "当前电池电量 [0, 100]"),
    BT::OutputPort<bool>("is_low_battery", "低电标志"),
  };
}

BT::NodeStatus BatteryMonitorNode::tick()
{
  // 读取阈值
  float threshold = getInput<float>("battery_threshold").value_or(20.0f);

  // 获取最新电池状态
  float battery_level = 0.0f;
  bool has_data = false;

  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    if (last_battery_msg_) {
      // sensor_msgs/BatteryState.percentage 范围是 [0, 1]
      battery_level = last_battery_msg_->percentage * 100.0f;
      has_data = true;
    }
  }

  if (!has_data) {
    // 尚未收到电池数据：保守策略，返回 FAILURE 表示低电
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "BatteryMonitorNode: 尚未收到电池数据，默认低电保护");
    setOutput("battery_level", 0.0f);
    setOutput("is_low_battery", true);
    return BT::NodeStatus::FAILURE;
  }

  // M7.4: 检测话题超时（超过 5 秒未收到新数据）
  auto now = node_->now();
  double elapsed;
  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    elapsed = (now - last_msg_time_).seconds();
  }

  if (elapsed > BATTERY_TIMEOUT_SECONDS) {
    // 话题超时：视为满电并告警，保证主流程继续
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "BatteryMonitorNode: 电池话题超时 %.1fs（>%.0fs），视为满电继续巡逻",
      elapsed, BATTERY_TIMEOUT_SECONDS);

    // 发布告警
    auto alert_msg = std::make_unique<std_msgs::msg::String>();
    alert_msg->data = "Battery data lost (topic timeout)";
    alert_pub_->publish(std::move(alert_msg));

    // 覆盖为满电状态，保证主流程不中断
    setOutput("battery_level", 100.0f);
    setOutput("is_low_battery", false);
    return BT::NodeStatus::SUCCESS;
  }

  // 写入黑板
  setOutput("battery_level", battery_level);

  // 判断是否低电
  bool low = (battery_level < threshold);
  setOutput("is_low_battery", low);

  if (low) {
    RCLCPP_WARN(node_->get_logger(),
      "BatteryMonitorNode: 低电警报! battery=%.1f%% < threshold=%.1f%%",
      battery_level, threshold);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(node_->get_logger(),
    "BatteryMonitorNode: 电量正常 battery=%.1f%% >= threshold=%.1f%%",
    battery_level, threshold);

  return BT::NodeStatus::SUCCESS;
}

void BatteryMonitorNode::batteryCallback(
  const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  last_battery_msg_ = msg;
  last_msg_time_ = node_->now();
}

}  // namespace nav2_demo
