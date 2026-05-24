#include "nav2_demo/bt_nodes/set_charging_mode_node.hpp"

#include <chrono>
#include <string>

namespace nav2_demo
{

SetChargingModeNode::SetChargingModeNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config)
, node_(node)
{
  charging_client_ = node_->create_client<std_srvs::srv::SetBool>(
    "/battery_simulator/set_charging");
}

BT::PortsList SetChargingModeNode::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "启用充电模式"),
  };
}

BT::NodeStatus SetChargingModeNode::tick()
{
  bool enabled = getInput<bool>("enabled").value_or(true);

  // 等待服务端就绪
  if (!charging_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(),
      "SetChargingModeNode: battery_simulator/set_charging 服务未就绪");
    return BT::NodeStatus::FAILURE;
  }

  // 构建请求
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;

  // 异步调用服务 + 自旋等待（确保服务端的回调被处理）
  auto future = charging_client_->async_send_request(request);

  // 自旋等待服务响应（同时处理其他 ROS 回调）
  auto start = node_->now();
  rclcpp::Duration timeout(std::chrono::seconds(3));
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      break;
    }
    if ((node_->now() - start) > timeout) {
      RCLCPP_ERROR(node_->get_logger(),
        "SetChargingModeNode: 服务调用超时 (enabled=%d)", enabled);
      return BT::NodeStatus::FAILURE;
    }
  }

  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(),
      "SetChargingModeNode: 服务调用失败: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
    "SetChargingModeNode: %s 充电模式", enabled ? "启用" : "禁用");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_demo
