#ifndef NAV2_DEMO__BT_NODES__DOCK_ACTION_NODE_HPP_
#define NAV2_DEMO__BT_NODES__DOCK_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <atomic>
#include <future>
#include <functional>
#include <optional>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_demo
{

/**
 * @brief M5.1 高层次分段对接行为树节点
 *
 * 封装了整个对接过程，包含两个阶段:
 *  Phase 1 (NAVIGATE): 使用 Nav2 navigate_to_pose 动作导航到预备位姿
 *  Phase 2 (SERVO):    使用 P 控制器视觉伺服到充电桩位置
 *
 * 内部实现了重试机制（最多 3 次视觉伺服重试）。
 *
 * 黑板端口:
 *  - dock_prep_pose (input, PoseStamped): 导航预备位姿
 *  - dock_pose_x (input, double): 充电桩 X 坐标
 *  - dock_pose_y (input, double): 充电桩 Y 坐标
 *  - dock_pose_yaw (input, double): 充电桩朝向 (弧度)
 *  - navigation_timeout (input, double, 默认 60.0): 导航超时
 *  - servo_timeout (input, double, 默认 15.0): 视觉伺服超时
 *  - dock_success (output, bool): 对接是否成功
 *  - nav_result (output, string): 导航阶段结果
 *
 * 返回值:
 *  - SUCCESS: 导航和视觉伺服均成功
 *  - FAILURE: 任一阶段失败（导航失败或伺服重试耗尽）
 *  - RUNNING: 正在执行
 */
class DockActionNode : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /// @brief 对接阶段枚举
  enum class DockPhase : uint8_t {
    INIT = 0,
    NAVIGATING,
    SERVOING,
    DONE,
    FAILED
  };

  /// @brief 机器人位姿源回调类型
  using PoseSourceCallback = std::function<std::optional<geometry_msgs::msg::Pose>()>;

  DockActionNode(
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
  /// @brief 启动导航阶段
  BT::NodeStatus startNavigation();

  /// @brief 处理导航阶段运行
  BT::NodeStatus runNavigation();

  /// @brief 启动伺服阶段
  BT::NodeStatus startServoing();

  /// @brief 处理伺服阶段运行
  BT::NodeStatus runServoing();

  /// @brief 取消当前导航目标
  void cancelNavigation();

  /// @brief 停止机器人运动
  void stopRobot();

  /// @brief 发布速度指令
  void publishVelocity(double linear_x, double angular_z);

  /// @brief 默认位姿源
  std::optional<geometry_msgs::msg::Pose> defaultPoseSource();

  /// @brief /odom 回调
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// @brief 从四元数提取偏航角
  static double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

  /// @brief 角度归一化
  static double normalizeAngle(double angle);

  /// @brief 计算相对误差
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
  static constexpr double ANGLE_THRESHOLD_ = 0.05;
  static constexpr double LATERAL_THRESHOLD_ = 0.02;
  static constexpr double FORWARD_THRESHOLD_ = 0.15;

  // === 重试参数 ===
  static constexpr int MAX_SERVO_RETRIES_ = 3;

  rclcpp::Node::SharedPtr node_;

  // === 导航部分 ===
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;
  std::atomic<bool> nav_result_received_{false};
  std::atomic<bool> nav_goal_accepted_{false};
  GoalHandleNav::WrappedResult nav_result_;
  rclcpp::Time nav_start_time_;
  double navigation_timeout_{60.0};

  // === 伺服部分 ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose::SharedPtr latest_robot_pose_;
  std::mutex pose_mutex_;
  PoseSourceCallback pose_source_;
  rclcpp::Time servo_start_time_;
  double servo_timeout_{15.0};
  double dock_pose_x_{7.5};
  double dock_pose_y_{6.0};
  double dock_pose_yaw_{-1.57};

  // === 状态管理 ===
  DockPhase phase_{DockPhase::INIT};
  int servo_retry_count_{0};

  // M7.3: /patrol_alerts 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__DOCK_ACTION_NODE_HPP_
