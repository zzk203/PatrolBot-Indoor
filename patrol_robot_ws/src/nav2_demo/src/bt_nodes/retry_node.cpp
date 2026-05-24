#include "nav2_demo/bt_nodes/retry_node.hpp"

#include <string>
#include <chrono>

namespace nav2_demo
{

RetryNode::RetryNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::DecoratorNode(name, config)
{
}

BT::PortsList RetryNode::providedPorts()
{
  return {
    BT::InputPort<int>("max_attempts", 3, "Maximum number of retry attempts"),
    BT::OutputPort<int>("retry_count", "Current attempt count (1-based)"),
  };
}

BT::NodeStatus RetryNode::tick()
{
  max_attempts_ = getInput<int>("max_attempts").value_or(3);

  setOutput("retry_count", attempt_count_ + 1);

  auto status = child_node_->executeTick();

  switch (status) {
    case BT::NodeStatus::SUCCESS:
      attempt_count_ = 0;
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE: {
      attempt_count_++;
      if (attempt_count_ < max_attempts_) {
        RCLCPP_WARN(rclcpp::get_logger("RetryNode"),
          "重试 %d/%d: 子节点失败，即将重试",
          attempt_count_, max_attempts_);
        child_node_->haltNode();
        return BT::NodeStatus::RUNNING;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("RetryNode"),
          "对接重试 %d/%d 全部失败！请检查充电桩或机器人状态。",
          attempt_count_, max_attempts_);
        return BT::NodeStatus::FAILURE;
      }
    }

    case BT::NodeStatus::RUNNING:
      // 子节点还在执行
      return BT::NodeStatus::RUNNING;

    default:
      return status;
  }
}

}  // namespace nav2_demo
