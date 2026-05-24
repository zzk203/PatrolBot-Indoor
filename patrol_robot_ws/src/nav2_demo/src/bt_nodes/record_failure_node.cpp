#include "nav2_demo/bt_nodes/record_failure_node.hpp"

#include <string>

namespace nav2_demo
{

RecordFailureNode::RecordFailureNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList RecordFailureNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("reason", "timeout", "Failure reason description"),
    BT::BidirectionalPort<int>("failure_count", "Accumulated failure counter"),
    BT::OutputPort<std::string>("last_failure_reason", "Last failure reason"),
  };
}

BT::NodeStatus RecordFailureNode::tick()
{
  auto reason = getInput<std::string>("reason").value_or("unknown");

  // 读取并递增失败计数器（双向端口可以读写）
  int count = 0;
  auto count_val = getInput<int>("failure_count");
  if (count_val) {
    count = count_val.value();
  }
  count++;
  setOutput("failure_count", count);  // 写回递增后的值

  // 记录最近一次失败原因
  setOutput("last_failure_reason", reason);

  RCLCPP_WARN(rclcpp::get_logger("RecordFailureNode"),
    "Patrol failure #%d recorded: %s. Continuing to next waypoint.",
    count, reason.c_str());

  // 始终返回 SUCCESS，使 Fallback 继续执行巡逻
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_demo
