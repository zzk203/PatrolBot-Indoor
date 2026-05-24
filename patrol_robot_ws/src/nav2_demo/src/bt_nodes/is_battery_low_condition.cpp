#include "nav2_demo/bt_nodes/is_battery_low_condition.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nav2_demo
{

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsBatteryLowCondition::providedPorts()
{
  return {
    BT::InputPort<bool>("is_low_battery", false, "低电标志"),
  };
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  bool is_low = getInput<bool>("is_low_battery").value_or(false);

  if (is_low) {
    RCLCPP_DEBUG(rclcpp::get_logger("IsBatteryLowCondition"),
      "电池低电，选择回充分支");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_demo
