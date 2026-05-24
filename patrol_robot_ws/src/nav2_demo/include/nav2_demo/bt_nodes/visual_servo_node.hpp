#ifndef NAV2_DEMO__BT_NODES__VISUAL_SERVO_NODE_HPP_
#define NAV2_DEMO__BT_NODES__VISUAL_SERVO_NODE_HPP_

#include <memory>
#include <string>
#include <atomic>
#include <functional>
#include <optional>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_demo
{

/**
 * @brief M5.2 模拟视觉伺服行为树节点
 *
 * 基类: StatefulActionNode (支持长时间运行的异步操作)
 *
 * 功能:
 *  从订阅的 /odom 获取机器人当前位姿，结合黑板上已知的充电桩位姿，
 *  使用 P 控制器计算机器人线速度和角速度并发布到 /cmd_vel，
 *  实现从预备位姿到充电桩的精细对接。
 *
 * 可测试性:
 *  通过 setRobotPoseSource() 方法可注入自定义机器人位姿回调，
 *  在单元测试中可完全控制机器人位姿，无需依赖 Gazebo 仿真。
 *
 * 黑板端口:
 *  - timeout (input, double, 默认 15.0): 视觉伺服超时秒数
 *  - dock_pose_x (input, double): 充电桩 X 坐标
 *  - dock_pose_y (input, double): 充电桩 Y 坐标
 *  - dock_pose_yaw (input, double): 充电桩朝向 (弧度)
 *  - dock_success (output, bool): 对接是否成功
 *
 * 成功条件 (满足所有):
 *  1. 角度偏差 |yaw_error| < 0.05 rad
 *  2. 横向偏差 |lateral_error| < 0.02 m
 *
 * 返回值:
 *  - SUCCESS: 对准成功（角度和横向偏差均在阈值内）
 *  - FAILURE: 超时未对准
 *  - RUNNING: 正在进行视觉伺服控制
 */
class VisualServoNode : public BT::StatefulActionNode
{
public:
  /// @brief 机器人位姿源回调类型（返回 map 坐标系下的机器人位姿）
  using PoseSourceCallback = std::function<std::optional<geometry_msgs::msg::Pose>()>;

  VisualServoNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  /// @brief 注入自定义机器人位姿源（用于单元测试）
  void setRobotPoseSource(PoseSourceCallback callback);

private:
  /// @brief 默认位姿源：从 /odom 订阅获取
  std::optional<geometry_msgs::msg::Pose> defaultPoseSource();

  /// @brief /odom 话题回调
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// @brief 发布速度指令到 /cmd_vel
  void publishVelocity(double linear_x, double angular_z);

  /// @brief 停止机器人
  void stopRobot();

  /// @brief 从四元数提取偏航角
  static double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  /// @brief 角度归一化到 [-PI, PI]
  static double normalizeAngle(double angle);

  /// @brief 计算 dock_pose 在 robot 本体坐标系下的误差
  /// @param robot_pose 机器人位姿
  /// @param dock_x 充电桩 X
  /// @param dock_y 充电桩 Y
  /// @param dock_yaw 充电桩朝向
  /// @param[out] forward_error 前进方向误差
  /// @param[out] lateral_error 横向误差
  /// @param[out] yaw_error 角度误差
  void computeErrors(
    const geometry_msgs::msg::Pose & robot_pose,
    double dock_x, double dock_y, double dock_yaw,
    double & forward_error,
    double & lateral_error,
    double & yaw_error) const;

  // === P 控制器参数 ===
  static constexpr double KP_LINEAR_ = 0.3;
  static constexpr double KP_ANGULAR_ = 0.5;
  static constexpr double MAX_LINEAR_SPEED_ = 0.2;
  static constexpr double MAX_ANGULAR_SPEED_ = 0.5;

  // === 成功阈值 ===
  static constexpr double ANGLE_THRESHOLD_ = 0.05;    // rad
  static constexpr double LATERAL_THRESHOLD_ = 0.02;  // m
  static constexpr double FORWARD_THRESHOLD_ = 0.15;  // m (接近距离)

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 位姿源（默认使用 /odom，测试时可注入）
  PoseSourceCallback pose_source_;

  // 最新机器人位姿缓存
  geometry_msgs::msg::Pose::SharedPtr latest_robot_pose_;
  std::mutex pose_mutex_;

  // 控制状态
  rclcpp::Time start_time_;
  double timeout_{15.0};
  double dock_pose_x_{7.5};
  double dock_pose_y_{6.0};
  double dock_pose_yaw_{-1.57};

  // 是否已成功（避免重复成功后的继续运行）
  bool succeeded_{false};
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__VISUAL_SERVO_NODE_HPP_
