#include "nav2_demo/nav2_demo_node.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <chrono>

namespace nav2_demo
{

Nav2DemoNode::Nav2DemoNode(const rclcpp::NodeOptions &options)
: Node("nav2_demo_node", options), goal_index_(0)
{
  // 声明并加载参数
  declare_parameters();

  // 创建 action 客户端（连接到 bt_navigator 的 navigate_to_pose 动作服务端）
  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // 订阅全局路径 /plan（由 planner_server 发布）
  plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(10).transient_local(),
    std::bind(&Nav2DemoNode::plan_callback, this, std::placeholders::_1));

  // 订阅里程计 /odom
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10),
    std::bind(&Nav2DemoNode::odom_callback, this, std::placeholders::_1));

  // 从参数加载航点（如果提供了 goals 参数，则覆盖默认）
  load_goals_from_parameters();

  if (goals_.empty()) {
    // 默认演示航点（与 10m×10m 地图匹配）
    goals_ = {
      {1.0, 0.0, 0.0},
      {2.0, 1.0, 1.57},
      {0.0, 2.0, 3.14},
    };
  }

  RCLCPP_INFO(this->get_logger(),
    "Nav2DemoNode 就绪: 监听 /plan, /odom, 共 %zu 个航点", goals_.size());
  for (size_t i = 0; i < goals_.size(); ++i) {
    RCLCPP_DEBUG(this->get_logger(), "  航点 %zu: (%.1f, %.1f, %.2f)",
                 i + 1, goals_[i][0], goals_[i][1], goals_[i][2]);
  }
}

void Nav2DemoNode::declare_parameters()
{
  // 航点参数（可动态配置，支持重复多次）
  // 用法: goals.x=[1.0,2.0,...] goals.y=[0.0,1.0,...] goals.yaw=[0.0,1.57,...]
  this->declare_parameter<std::vector<double>>("goals.x", std::vector<double>());
  this->declare_parameter<std::vector<double>>("goals.y", std::vector<double>());
  this->declare_parameter<std::vector<double>>("goals.yaw", std::vector<double>());
}

void Nav2DemoNode::load_goals_from_parameters()
{
  auto goals_x = this->get_parameter("goals.x").as_double_array();
  auto goals_y = this->get_parameter("goals.y").as_double_array();
  auto goals_yaw = this->get_parameter("goals.yaw").as_double_array();

  // 三个数组必须等长
  if (goals_x.empty() || goals_y.empty() || goals_yaw.empty()) {
    return;  // 未配置参数，使用默认值
  }

  if (goals_x.size() != goals_y.size() || goals_y.size() != goals_yaw.size()) {
    RCLCPP_WARN(this->get_logger(),
      "航点参数长度不一致: x=%zu, y=%zu, yaw=%zu，使用默认值",
      goals_x.size(), goals_y.size(), goals_yaw.size());
    return;
  }

  goals_.clear();
  for (size_t i = 0; i < goals_x.size(); ++i) {
    goals_.push_back({goals_x[i], goals_y[i], goals_yaw[i]});
  }
  RCLCPP_INFO(this->get_logger(), "从参数加载了 %zu 个航点", goals_.size());
}

bool Nav2DemoNode::is_action_server_ready(const std::chrono::seconds &timeout)
{
  return action_client_->wait_for_action_server(timeout);
}

void Nav2DemoNode::send_next_goal()
{
  if (goal_index_ >= goals_.size()) {
    RCLCPP_INFO(this->get_logger(), "✅ 所有 %zu 个航点已完成!", goals_.size());
    return;
  }

  if (!is_action_server_ready()) {
    RCLCPP_ERROR(this->get_logger(),
      "❌ Nav2 Action Server (navigate_to_pose) 未就绪，请确保 Nav2 栈已启动");
    return;
  }

  const auto &g = goals_[goal_index_];
  auto goal_msg = NavigateToPose::Goal();
  // 空字符串表示使用 bt_navigator 的 default_bt_xml_filename
  goal_msg.behavior_tree = "";
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = g[0];
  goal_msg.pose.pose.position.y = g[1];
  // yaw → quaternion (只绕 Z 轴旋转)
  goal_msg.pose.pose.orientation.z = std::sin(g[2] / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(g[2] / 2.0);

  RCLCPP_INFO(this->get_logger(),
    "🚀 发送航点 %zu/%zu: (%.1f, %.1f, θ=%.2f)",
    goal_index_ + 1, goals_.size(), g[0], g[1], g[2]);

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.result_callback =
    std::bind(&Nav2DemoNode::result_callback, this, std::placeholders::_1);
  options.feedback_callback =
    std::bind(&Nav2DemoNode::feedback_callback, this,
              std::placeholders::_1, std::placeholders::_2);

  // 保存 future 以便后续取消
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, options);
  goal_active_ = true;
}

void Nav2DemoNode::cancel_current_goal()
{
  if (!goal_active_) {
    RCLCPP_WARN(this->get_logger(), "当前无活跃目标可取消");
    return;
  }

  // 等待目标句柄就绪
  if (goal_handle_future_.valid()) {
    auto status = goal_handle_future_.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
      auto goal_handle = goal_handle_future_.get();
      if (goal_handle) {
        auto cancel_future = action_client_->async_cancel_goal(goal_handle);
        RCLCPP_INFO(this->get_logger(), "取消航点 %zu", goal_index_ + 1);
      }
    }
  }
  goal_active_ = false;
}

// ===================== 私有回调 =====================

void Nav2DemoNode::plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) return;
  RCLCPP_INFO(this->get_logger(),
    "🗺️ 收到全局路径: %zu 个路点, 总长约 %.2fm",
    msg->poses.size(),
    calculate_path_length(*msg));
}

void Nav2DemoNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(),
    "里程计: pos=(%.2f, %.2f) v=(%.2f, %.2f)",
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->twist.twist.linear.x,
    msg->twist.twist.angular.z);
}

void Nav2DemoNode::result_callback(const GoalHandleNav::WrappedResult &result)
{
  goal_active_ = false;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "✅ 航点 %zu 到达成功!", goal_index_ + 1);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "⚠️ 航点 %zu 被取消", goal_index_ + 1);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "❌ 航点 %zu 失败（ABORTED）", goal_index_ + 1);
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        // 输出错误信息（如果存在）
        if (result.result) {
          RCLCPP_ERROR(this->get_logger(), "   错误码: %d",
                       result.result->error_code);
        }
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "❓ 航点 %zu 未知结果码: %d",
                   goal_index_ + 1, static_cast<int>(result.code));
  }

  // 发送下一个航点（无论成功或失败都继续）
  goal_index_++;
  send_next_goal();
}

void Nav2DemoNode::feedback_callback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(),
    "📊 剩余距离: %.2fm, 预计时间: %.1fs",
    feedback->distance_remaining,
    rclcpp::Duration(feedback->estimated_time_remaining).seconds());
}

double Nav2DemoNode::calculate_path_length(const nav_msgs::msg::Path &path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); i++) {
    const double dx = path.poses[i].pose.position.x -
                      path.poses[i - 1].pose.position.x;
    const double dy = path.poses[i].pose.position.y -
                      path.poses[i - 1].pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

}  // namespace nav2_demo

// ===================== main 入口 =====================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<nav2_demo::Nav2DemoNode>();

  // 等待 Nav2 栈就绪后发送第一个航点
  RCLCPP_INFO(node->get_logger(), "等待 Nav2 Action Server 就绪...");
  if (node->is_action_server_ready(std::chrono::seconds(10))) {
    RCLCPP_INFO(node->get_logger(), "Nav2 就绪，开始导航");
    node->send_next_goal();
  } else {
    RCLCPP_ERROR(node->get_logger(), "Nav2 未就绪，退出");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
