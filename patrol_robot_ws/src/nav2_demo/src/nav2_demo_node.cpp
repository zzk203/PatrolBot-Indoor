#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Nav2DemoNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2DemoNode() : Node("nav2_demo_node"), goal_index_(0)
  {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      std::bind(&Nav2DemoNode::plan_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&Nav2DemoNode::odom_callback, this, std::placeholders::_1));

    goals_ = {
      {1.0, 0.0, 0.0},
      {2.0, 1.0, 1.57},
      {0.0, 2.0, 3.14},
    };

    RCLCPP_INFO(this->get_logger(),
      "Nav2DemoNode 就绪: 监听 /plan, /odom, 发送导航目标");
  }

  void send_next_goal()
  {
    if (goal_index_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(), "所有航点完成!");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server 未就绪");
      return;
    }

    auto &g = goals_[goal_index_];
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.behavior_tree = "";
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = g[0];
    goal_msg.pose.pose.position.y = g[1];
    goal_msg.pose.pose.orientation.z = sin(g[2] / 2.0);
    goal_msg.pose.pose.orientation.w = cos(g[2] / 2.0);

    RCLCPP_INFO(this->get_logger(), "发送航点 %zu: (%.1f, %.1f, %.2f)",
                goal_index_ + 1, g[0], g[1], g[2]);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback =
      std::bind(&Nav2DemoNode::result_callback, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(&Nav2DemoNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    action_client_->async_send_goal(goal_msg, options);
  }

private:
  void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;
    RCLCPP_INFO(this->get_logger(),
      "收到全局路径: %zu 个路点, 总长约 %.2fm",
      msg->poses.size(),
      calculate_path_length(*msg));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(),
      "里程计: pos=(%.2f,%.2f) v=(%.2f,%.2f)",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->twist.twist.linear.x,
      msg->twist.twist.angular.z);
  }

  void result_callback(const GoalHandleNav::WrappedResult &result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "航点 %zu 到达成功!", goal_index_ + 1);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "航点 %zu 被取消", goal_index_ + 1);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "航点 %zu 失败", goal_index_ + 1);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "航点 %zu 未知结果", goal_index_ + 1);
    }
    goal_index_++;
    send_next_goal();
  }

  void feedback_callback(
      GoalHandleNav::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(),
      "剩余距离: %.2fm, 预计时间: %.1fs",
      feedback->distance_remaining,
      rclcpp::Duration(feedback->estimated_time_remaining).seconds());
  }

  double calculate_path_length(const nav_msgs::msg::Path &path)
  {
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); i++) {
      double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
      double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
      length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<std::array<double, 3>> goals_;
  size_t goal_index_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2DemoNode>();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  node->send_next_goal();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
