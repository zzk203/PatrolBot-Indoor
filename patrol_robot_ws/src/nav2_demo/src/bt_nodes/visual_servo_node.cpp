#include "nav2_demo/bt_nodes/visual_servo_node.hpp"

#include <cmath>
#include <string>
#include <algorithm>

namespace nav2_demo
{

VisualServoNode::VisualServoNode(
  const std::string & name,
  const BT::NodeConfig & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config)
, node_(node)
{
  // 默认位姿源：订阅 /odom
  pose_source_ = std::bind(&VisualServoNode::defaultPoseSource, this);
}

BT::PortsList VisualServoNode::providedPorts()
{
  return {
    BT::InputPort<double>("timeout", 15.0, "Visual servo timeout in seconds"),
    BT::InputPort<double>("dock_pose_x", 7.5, "Dock pose X coordinate"),
    BT::InputPort<double>("dock_pose_y", 6.0, "Dock pose Y coordinate"),
    BT::InputPort<double>("dock_pose_yaw", -1.57, "Dock pose yaw (radians)"),
    BT::OutputPort<bool>("dock_success", "Docking success flag"),
  };
}

BT::NodeStatus VisualServoNode::onStart()
{
  // 读取端口参数
  timeout_ = getInput<double>("timeout").value_or(15.0);
  dock_pose_x_ = getInput<double>("dock_pose_x").value_or(7.5);
  dock_pose_y_ = getInput<double>("dock_pose_y").value_or(6.0);
  dock_pose_yaw_ = getInput<double>("dock_pose_yaw").value_or(-1.57);

  // 重置状态
  succeeded_ = false;
  latest_robot_pose_.reset();
  start_time_ = node_->now();

  // 创建 /cmd_vel 发布者
  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  // 订阅 /odom（仅首次启动时创建）
  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      std::bind(&VisualServoNode::odomCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(node_->get_logger(),
    "VisualServoNode: 开始视觉伺服 (dock=(%.2f, %.2f, %.2f), timeout=%.1fs)",
    dock_pose_x_, dock_pose_y_, dock_pose_yaw_, timeout_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VisualServoNode::onRunning()
{
  // 检查超时
  auto elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_) {
    RCLCPP_WARN(node_->get_logger(),
      "VisualServoNode: 超时 (%.1fs > %.1fs)，对接失败", elapsed, timeout_);
    stopRobot();
    setOutput("dock_success", false);
    return BT::NodeStatus::FAILURE;
  }

  // 获取机器人位姿
  auto robot_pose = pose_source_();
  if (!robot_pose) {
    // 尚未收到位姿数据
    RCLCPP_DEBUG(node_->get_logger(), "VisualServoNode: 等待位姿数据...");
    return BT::NodeStatus::RUNNING;
  }

  // 计算机器人在本体坐标系下的 dock 误差
  double forward_error, lateral_error, yaw_error;
  computeErrors(
    robot_pose.value(),
    dock_pose_x_, dock_pose_y_, dock_pose_yaw_,
    forward_error, lateral_error, yaw_error);

  RCLCPP_DEBUG(node_->get_logger(),
    "VisualServoNode: 误差 forward=%.3f lateral=%.3f yaw=%.3f",
    forward_error, lateral_error, yaw_error);

  // 检查成功条件
  if (std::abs(yaw_error) < ANGLE_THRESHOLD_ &&
      std::abs(lateral_error) < LATERAL_THRESHOLD_ &&
      std::abs(forward_error) < FORWARD_THRESHOLD_)
  {
    RCLCPP_INFO(node_->get_logger(),
      "VisualServoNode: 对准成功! (yaw=%.4f rad, lat=%.4f m)",
      yaw_error, lateral_error);
    stopRobot();
    succeeded_ = true;
    setOutput("dock_success", true);
    return BT::NodeStatus::SUCCESS;
  }

  // P 控制器：计算机器人速度
  double linear_x = std::clamp(KP_LINEAR_ * forward_error,
    -MAX_LINEAR_SPEED_, MAX_LINEAR_SPEED_);

  // 角度控制 = 航向误差 + 横向误差补偿
  double lateral_correction = std::clamp(KP_ANGULAR_ * 0.3 * lateral_error,
    -0.3, 0.3);
  double angular_z = std::clamp(
    KP_ANGULAR_ * yaw_error + lateral_correction,
    -MAX_ANGULAR_SPEED_, MAX_ANGULAR_SPEED_);

  publishVelocity(linear_x, angular_z);

  return BT::NodeStatus::RUNNING;
}

void VisualServoNode::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "VisualServoNode: 被父节点中断");
  stopRobot();
}

void VisualServoNode::setRobotPoseSource(PoseSourceCallback callback)
{
  if (callback) {
    pose_source_ = std::move(callback);
  } else {
    // 重置为默认源
    pose_source_ = std::bind(&VisualServoNode::defaultPoseSource, this);
  }
}

std::optional<geometry_msgs::msg::Pose> VisualServoNode::defaultPoseSource()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (latest_robot_pose_) {
    return *latest_robot_pose_;
  }
  return std::nullopt;
}

void VisualServoNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_robot_pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
}

void VisualServoNode::publishVelocity(double linear_x, double angular_z)
{
  auto twist = std::make_unique<geometry_msgs::msg::Twist>();
  twist->linear.x = linear_x;
  twist->angular.z = angular_z;
  cmd_vel_pub_->publish(std::move(twist));
}

void VisualServoNode::stopRobot()
{
  publishVelocity(0.0, 0.0);
}

double VisualServoNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  // 从四元数 (x=0, y=0, z, w) 提取偏航角
  // 对于绕 Z 轴纯旋转: z = sin(theta/2), w = cos(theta/2)
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double VisualServoNode::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void VisualServoNode::computeErrors(
  const geometry_msgs::msg::Pose & robot_pose,
  double dock_x, double dock_y, double dock_yaw,
  double & forward_error,
  double & lateral_error,
  double & yaw_error) const
{
  double robot_yaw = getYawFromQuaternion(robot_pose.orientation);

  // 世界坐标系下的位移向量 (dock - robot)
  double dx_world = dock_x - robot_pose.position.x;
  double dy_world = dock_y - robot_pose.position.y;

  // 转换到机器人本体坐标系
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);
  forward_error = cos_yaw * dx_world + sin_yaw * dy_world;
  lateral_error = -sin_yaw * dx_world + cos_yaw * dy_world;

  // 角度误差
  yaw_error = normalizeAngle(dock_yaw - robot_yaw);
}

}  // namespace nav2_demo
