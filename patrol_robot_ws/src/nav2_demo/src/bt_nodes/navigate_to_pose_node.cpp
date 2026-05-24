#include "nav2_demo/bt_nodes/navigate_to_pose_node.hpp"

#include <chrono>
#include <cmath>
#include <string>

namespace nav2_demo
{

NavigateToPoseNode::NavigateToPoseNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
}

BT::PortsList NavigateToPoseNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal pose"),
    BT::InputPort<double>("timeout", 30.0, "Navigation timeout in seconds"),
    BT::OutputPort<std::string>("nav_result", "Result: SUCCESS/FAILURE/TIMEOUT"),
  };
}

BT::NodeStatus NavigateToPoseNode::onStart()
{
  // 读取输入端口
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseNode: missing 'goal' input");
    setOutput("nav_result", std::string("FAILURE"));
    return BT::NodeStatus::FAILURE;
  }

  timeout_ = getInput<double>("timeout").value_or(30.0);

  // 重置状态
  result_received_ = false;
  goal_accepted_ = false;

  // 创建或复用动作客户端
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    RCLCPP_DEBUG(node_->get_logger(), "NavigateToPoseNode: created action client");
  }

  // 检查动作服务端是否就绪
  if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_WARN(node_->get_logger(),
      "NavigateToPoseNode: action server not ready (timeout=3s)");
    setOutput("nav_result", std::string("FAILURE"));
    return BT::NodeStatus::FAILURE;
  }

  // 构建导航目标消息
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.behavior_tree = "";
  goal_msg.pose = goal.value();

  // 确保 frame_id 和 timestamp 正确
  if (goal_msg.pose.header.frame_id.empty()) {
    goal_msg.pose.header.frame_id = "map";
  }
  goal_msg.pose.header.stamp = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "NavigateToPoseNode: sending goal (%.2f, %.2f)",
    goal_msg.pose.pose.position.x,
    goal_msg.pose.pose.position.y);

  // 设置回调
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr & goal_handle) {
      if (goal_handle) {
        goal_accepted_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "NavigateToPoseNode: goal accepted");
      } else {
        goal_accepted_ = false;
        result_received_ = true;
        RCLCPP_WARN(node_->get_logger(), "NavigateToPoseNode: goal rejected");
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult & wrapped_result) {
      result_ = wrapped_result;
      result_received_ = true;

      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "NavigateToPoseNode: navigation succeeded");
      } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "NavigateToPoseNode: navigation canceled");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseNode: navigation aborted");
      }
    };

  // 异步发送目标
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseNode::onRunning()
{
  // 检查超时
  auto elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_) {
    RCLCPP_WARN(node_->get_logger(),
      "NavigateToPoseNode: timeout after %.1f seconds (limit=%.1f)",
      elapsed, timeout_);
    cancelGoal();
    setOutput("nav_result", std::string("TIMEOUT"));
    return BT::NodeStatus::FAILURE;
  }

  // 检查是否已收到结果
  if (result_received_) {
    if (result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      setOutput("nav_result", std::string("SUCCESS"));
      return BT::NodeStatus::SUCCESS;
    } else {
      setOutput("nav_result", std::string("FAILURE"));
      return BT::NodeStatus::FAILURE;
    }
  }

  // 还在执行中
  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseNode::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "NavigateToPoseNode: halted by parent control node");
  cancelGoal();
}

void NavigateToPoseNode::cancelGoal()
{
  if (goal_handle_future_.valid()) {
    auto status = goal_handle_future_.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      auto goal_handle = goal_handle_future_.get();
      if (goal_handle) {
        action_client_->async_cancel_goal(goal_handle);
        RCLCPP_DEBUG(node_->get_logger(), "NavigateToPoseNode: cancel request sent");
      }
    }
  }
}

}  // namespace nav2_demo
