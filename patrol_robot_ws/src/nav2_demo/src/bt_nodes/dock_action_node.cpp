#include "nav2_demo/bt_nodes/dock_action_node.hpp"

#include <cmath>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory>

#include "std_msgs/msg/string.hpp"

namespace nav2_demo
{

DockActionNode::DockActionNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
  // 默认位姿源
  pose_source_ = std::bind(&DockActionNode::defaultPoseSource, this);

  // M7.3: 创建 /patrol_alerts 发布者（用于对接失败告警）
  alert_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/patrol_alerts", rclcpp::QoS(10).transient_local());
}

/// @brief 发布告警到 /patrol_alerts 话题
static void publishAlert(
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub,
  const std::string & msg)
{
  if (pub) {
    auto alert_msg = std::make_unique<std_msgs::msg::String>();
    alert_msg->data = msg;
    pub->publish(std::move(alert_msg));
  }
}

BT::PortsList DockActionNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("dock_prep_pose", "Navigation prep pose before docking"),
    BT::InputPort<double>("dock_pose_x", 7.5, "Dock pose X coordinate"),
    BT::InputPort<double>("dock_pose_y", 6.0, "Dock pose Y coordinate"),
    BT::InputPort<double>("dock_pose_yaw", -1.57, "Dock pose yaw (radians)"),
    BT::InputPort<double>("navigation_timeout", 60.0, "Navigation timeout in seconds"),
    BT::InputPort<double>("servo_timeout", 15.0, "Visual servo timeout in seconds"),
    BT::OutputPort<bool>("dock_success", "Docking success flag"),
    BT::OutputPort<std::string>("nav_result", "Navigation phase result"),
  };
}

void DockActionNode::setRobotPoseSource(PoseSourceCallback callback)
{
  if (callback) {
    pose_source_ = std::move(callback);
  } else {
    pose_source_ = std::bind(&DockActionNode::defaultPoseSource, this);
  }
}

// ==================== onStart ====================

BT::NodeStatus DockActionNode::onStart()
{
  // 读取端口参数
  navigation_timeout_ = getInput<double>("navigation_timeout").value_or(60.0);
  servo_timeout_ = getInput<double>("servo_timeout").value_or(15.0);
  dock_pose_x_ = getInput<double>("dock_pose_x").value_or(7.5);
  dock_pose_y_ = getInput<double>("dock_pose_y").value_or(6.0);
  dock_pose_yaw_ = getInput<double>("dock_pose_yaw").value_or(-1.57);

  // 重置状态
  phase_ = DockPhase::INIT;
  servo_retry_count_ = 0;
  nav_result_received_ = false;
  nav_goal_accepted_ = false;
  latest_robot_pose_.reset();

  RCLCPP_INFO(node_->get_logger(),
    "DockActionNode: 开始分段对接 (dock=(%.2f, %.2f, %.2f))",
    dock_pose_x_, dock_pose_y_, dock_pose_yaw_);

  // 直接进入导航阶段
  return startNavigation();
}

// ==================== onRunning ====================

BT::NodeStatus DockActionNode::onRunning()
{
  switch (phase_) {
    case DockPhase::NAVIGATING:
      return runNavigation();
    case DockPhase::SERVOING:
      return runServoing();
    case DockPhase::DONE:
      return BT::NodeStatus::SUCCESS;
    case DockPhase::FAILED:
      return BT::NodeStatus::FAILURE;
    default:
      return BT::NodeStatus::FAILURE;
  }
}

// ==================== onHalted ====================

void DockActionNode::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "DockActionNode: 被父节点中断");

  if (phase_ == DockPhase::NAVIGATING) {
    cancelNavigation();
  }
  stopRobot();
}

// ==================== 导航阶段 ====================

BT::NodeStatus DockActionNode::startNavigation()
{
  auto prep_pose = getInput<geometry_msgs::msg::PoseStamped>("dock_prep_pose");
  if (!prep_pose) {
    RCLCPP_ERROR(node_->get_logger(), "DockActionNode: 缺少 dock_prep_pose 输入");
    setOutput("nav_result", std::string("FAILURE"));
    setOutput("dock_success", false);
    phase_ = DockPhase::FAILED;
    return BT::NodeStatus::FAILURE;
  }

  phase_ = DockPhase::NAVIGATING;
  nav_result_received_ = false;
  nav_goal_accepted_ = false;

  // 创建或复用动作客户端
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  }

  // 检查动作服务端
  if (!action_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
      "DockActionNode: navigate_to_pose 动作服务端未就绪");
    setOutput("nav_result", std::string("FAILURE"));
    setOutput("dock_success", false);
    phase_ = DockPhase::FAILED;
    return BT::NodeStatus::FAILURE;
  }

  // 构建导航目标
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.behavior_tree = "";
  goal_msg.pose = prep_pose.value();

  if (goal_msg.pose.header.frame_id.empty()) {
    goal_msg.pose.header.frame_id = "map";
  }
  goal_msg.pose.header.stamp = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "DockActionNode: 导航到预备位姿 (%.2f, %.2f)",
    goal_msg.pose.pose.position.x,
    goal_msg.pose.pose.position.y);

  // 设置回调
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr & goal_handle) {
      if (goal_handle) {
        nav_goal_accepted_ = true;
      } else {
        nav_goal_accepted_ = false;
        nav_result_received_ = true;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult & wrapped_result) {
      nav_result_ = wrapped_result;
      nav_result_received_ = true;
    };

  // 异步发送
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  nav_start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockActionNode::runNavigation()
{
  // 检查超时
  auto elapsed = (node_->now() - nav_start_time_).seconds();
  if (elapsed > navigation_timeout_) {
    RCLCPP_WARN(node_->get_logger(),
      "DockActionNode: 导航超时 (%.1fs > %.1fs)", elapsed, navigation_timeout_);
    cancelNavigation();
    setOutput("nav_result", std::string("TIMEOUT"));
    setOutput("dock_success", false);
    phase_ = DockPhase::FAILED;
    return BT::NodeStatus::FAILURE;
  }

  // 等待结果
  if (nav_result_received_) {
    if (nav_result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "DockActionNode: 已到达预备位姿，开始视觉伺服");
      setOutput("nav_result", std::string("SUCCESS"));
      // 进入伺服阶段
      return startServoing();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "DockActionNode: 导航失败");
      // M7.3: 发布对接导航失败告警
      publishAlert(alert_pub_, "Docking navigation failed");
      setOutput("nav_result", std::string("FAILURE"));
      setOutput("dock_success", false);
      phase_ = DockPhase::FAILED;
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void DockActionNode::cancelNavigation()
{
  if (goal_handle_future_.valid()) {
    auto status = goal_handle_future_.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      auto goal_handle = goal_handle_future_.get();
      if (goal_handle) {
        action_client_->async_cancel_goal(goal_handle);
      }
    }
  }
}

// ==================== 伺服阶段 ====================

BT::NodeStatus DockActionNode::startServoing()
{
  phase_ = DockPhase::SERVOING;
  servo_start_time_ = node_->now();
  latest_robot_pose_.reset();

  // 创建发布者
  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  // 订阅 /odom
  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      std::bind(&DockActionNode::odomCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(node_->get_logger(),
    "DockActionNode: 视觉伺服开始 (重试 %d/%d)",
    servo_retry_count_ + 1, MAX_SERVO_RETRIES_);

  // 第一次伺服直接进入运行状态
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockActionNode::runServoing()
{
  // 检查超时
  auto elapsed = (node_->now() - servo_start_time_).seconds();
  if (elapsed > servo_timeout_) {
    RCLCPP_WARN(node_->get_logger(),
      "DockActionNode: 伺服超时 (%.1fs > %.1fs)", elapsed, servo_timeout_);
    stopRobot();

    servo_retry_count_++;
    if (servo_retry_count_ < MAX_SERVO_RETRIES_) {
      RCLCPP_WARN(node_->get_logger(),
        "DockActionNode: 伺服重试 %d/%d", servo_retry_count_ + 1, MAX_SERVO_RETRIES_);
      return startServoing();
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "DockActionNode: 伺服重试 %d 次全部失败！", MAX_SERVO_RETRIES_);
      // M7.3: 发布对接失败告警
      publishAlert(alert_pub_, "Docking failed after 3 retries");
      setOutput("dock_success", false);
      phase_ = DockPhase::FAILED;
      return BT::NodeStatus::FAILURE;
    }
  }

  // 获取机器人位姿
  auto robot_pose = pose_source_();
  if (!robot_pose) {
    return BT::NodeStatus::RUNNING;
  }

  // 计算机器人在本体坐标系下的 dock 误差
  double forward_error, lateral_error, yaw_error;
  computeErrors(
    robot_pose.value(),
    dock_pose_x_, dock_pose_y_, dock_pose_yaw_,
    forward_error, lateral_error, yaw_error);

  RCLCPP_DEBUG(node_->get_logger(),
    "DockActionNode: 伺服误差 forward=%.3f lateral=%.3f yaw=%.3f",
    forward_error, lateral_error, yaw_error);

  // 检查成功条件
  if (std::abs(yaw_error) < ANGLE_THRESHOLD_ &&
      std::abs(lateral_error) < LATERAL_THRESHOLD_ &&
      std::abs(forward_error) < FORWARD_THRESHOLD_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "DockActionNode: 对接成功! (yaw=%.4f rad, lat=%.4f m)",
      yaw_error, lateral_error);
    stopRobot();
    setOutput("dock_success", true);
    phase_ = DockPhase::DONE;
    return BT::NodeStatus::SUCCESS;
  }

  // P 控制器
  double linear_x = std::clamp(KP_LINEAR_ * forward_error,
    -MAX_LINEAR_SPEED_, MAX_LINEAR_SPEED_);

  double lateral_correction = std::clamp(KP_ANGULAR_ * 0.3 * lateral_error,
    -0.3, 0.3);
  double angular_z = std::clamp(
    KP_ANGULAR_ * yaw_error + lateral_correction,
    -MAX_ANGULAR_SPEED_, MAX_ANGULAR_SPEED_);

  publishVelocity(linear_x, angular_z);

  return BT::NodeStatus::RUNNING;
}

// ==================== 辅助方法 ====================

void DockActionNode::stopRobot()
{
  publishVelocity(0.0, 0.0);
}

void DockActionNode::publishVelocity(double linear_x, double angular_z)
{
  if (cmd_vel_pub_) {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    twist->linear.x = linear_x;
    twist->angular.z = angular_z;
    cmd_vel_pub_->publish(std::move(twist));
  }
}

std::optional<geometry_msgs::msg::Pose> DockActionNode::defaultPoseSource()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (latest_robot_pose_) {
    return *latest_robot_pose_;
  }
  return std::nullopt;
}

void DockActionNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_robot_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
}

double DockActionNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double DockActionNode::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void DockActionNode::computeErrors(
  const geometry_msgs::msg::Pose & robot_pose,
  double dock_x, double dock_y, double dock_yaw,
  double & forward_error,
  double & lateral_error,
  double & yaw_error) const
{
  double robot_yaw = getYawFromQuaternion(robot_pose.orientation);

  double dx_world = dock_x - robot_pose.position.x;
  double dy_world = dock_y - robot_pose.position.y;

  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);
  forward_error = cos_yaw * dx_world + sin_yaw * dy_world;
  lateral_error = -sin_yaw * dx_world + cos_yaw * dy_world;

  yaw_error = normalizeAngle(dock_yaw - robot_yaw);
}

}  // namespace nav2_demo
