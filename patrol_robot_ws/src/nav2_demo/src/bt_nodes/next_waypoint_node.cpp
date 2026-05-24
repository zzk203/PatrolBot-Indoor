#include "nav2_demo/bt_nodes/next_waypoint_node.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace nav2_demo
{

NextWaypointNode::NextWaypointNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList NextWaypointNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<Waypoint>>("patrol_waypoints", "Patrol waypoints list"),
    BT::InputPort<int>("current_index", "Current waypoint index (read before increment)"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_waypoint",
      "Current waypoint as PoseStamped"),
  };
}

BT::NodeStatus NextWaypointNode::tick()
{
  auto waypoints = getInput<std::vector<Waypoint>>("patrol_waypoints");
  if (!waypoints) {
    RCLCPP_ERROR(rclcpp::get_logger("NextWaypointNode"),
      "Missing 'patrol_waypoints' on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  auto index_opt = getInput<int>("current_index");
  if (!index_opt) {
    RCLCPP_ERROR(rclcpp::get_logger("NextWaypointNode"),
      "Missing 'current_index' on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  int idx = index_opt.value();

  if (idx < 0 || static_cast<size_t>(idx) >= waypoints.value().size()) {
    RCLCPP_INFO(rclcpp::get_logger("NextWaypointNode"),
      "All %zu waypoints visited (index=%d). Patrol complete.",
      waypoints.value().size(), idx);
    return BT::NodeStatus::FAILURE;
  }

  // 获取当前巡逻点并转换为 PoseStamped
  const auto & wp = waypoints.value()[idx];
  auto pose = wp.toPoseStamped();

  // 设置时间戳
  // 注意：此处无法获取 ROS 时间，由 NavigateToPoseNode 的 onStart() 补充时间戳
  // 这里留下空时间戳，NavigateToPoseNode 会覆盖

  setOutput("current_waypoint", pose);

  // 自增索引并写回黑板
  int next_idx = idx + 1;
  config().blackboard->set<int>("current_index", next_idx);

  RCLCPP_DEBUG(rclcpp::get_logger("NextWaypointNode"),
    "Waypoint %d/%zu: (%.2f, %.2f, θ=%.2f)",
    idx + 1, waypoints.value().size(), wp.x, wp.y, wp.yaw);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_demo
