#include "nav2_demo/bt_nodes/is_patrol_complete_condition.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_demo
{

IsPatrolCompleteCondition::IsPatrolCompleteCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsPatrolCompleteCondition::providedPorts()
{
  return {
    BT::InputPort<bool>("patrol_complete", "Patrol completion flag"),
  };
}

BT::NodeStatus IsPatrolCompleteCondition::tick()
{
  auto complete = getInput<bool>("patrol_complete");
  if (!complete) {
    RCLCPP_WARN(rclcpp::get_logger("IsPatrolCompleteCondition"),
      "Missing 'patrol_complete' on blackboard, assuming not complete");
    return BT::NodeStatus::FAILURE;
  }

  if (complete.value()) {
    RCLCPP_INFO(rclcpp::get_logger("IsPatrolCompleteCondition"),
      "Patrol completed!");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_demo
