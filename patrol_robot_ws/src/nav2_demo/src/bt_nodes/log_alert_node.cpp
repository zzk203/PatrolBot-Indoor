#include "nav2_demo/bt_nodes/log_alert_node.hpp"

#include <string>
#include <algorithm>

namespace nav2_demo
{

LogAlertNode::LogAlertNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config)
, node_(node)
{
  // 创建 /patrol_alerts 话题发布者
  alert_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/patrol_alerts", rclcpp::QoS(10).transient_local());
}

BT::PortsList LogAlertNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("alert_msg", "Alert message to log and publish"),
    BT::InputPort<std::string>("severity", "ERROR", "Log severity (ERROR/WARN/INFO)"),
  };
}

BT::NodeStatus LogAlertNode::tick()
{
  // 读取告警消息
  auto alert_msg = getInput<std::string>("alert_msg").value_or("Unspecified alert");

  // 读取严重级别，转大写
  auto severity_str = getInput<std::string>("severity").value_or("ERROR");
  std::transform(severity_str.begin(), severity_str.end(), severity_str.begin(),
                 ::toupper);

  // 根据 severity 选择日志级别输出
  if (severity_str == "WARN") {
    RCLCPP_WARN(node_->get_logger(), "[PatrolAlert] %s", alert_msg.c_str());
  } else if (severity_str == "INFO") {
    RCLCPP_INFO(node_->get_logger(), "[PatrolAlert] %s", alert_msg.c_str());
  } else {
    // 默认 ERROR
    RCLCPP_ERROR(node_->get_logger(), "[PatrolAlert] %s", alert_msg.c_str());
  }

  // 发布到 /patrol_alerts 话题
  publishAlert(alert_msg);

  // 始终返回 SUCCESS（fail-safe）
  return BT::NodeStatus::SUCCESS;
}

void LogAlertNode::publishAlert(const std::string & msg)
{
  auto alert_msg = std::make_unique<std_msgs::msg::String>();
  alert_msg->data = msg;
  alert_pub_->publish(std::move(alert_msg));
}

}  // namespace nav2_demo
