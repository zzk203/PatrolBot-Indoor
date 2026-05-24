#ifndef NAV2_DEMO__BT_NODES__AMCL_POSE_MONITOR_NODE_HPP_
#define NAV2_DEMO__BT_NODES__AMCL_POSE_MONITOR_NODE_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <array>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_demo
{

/**
 * @brief M7.5 定位丢失检测行为树条件节点
 *
 * 订阅 /amcl_pose 话题，监控 AMCL 位姿协方差。
 * 当协方差对角元素超过阈值时，认为定位质量下降，
 * 发布告警并返回 FAILURE（可被上层 Fallback 捕获）。
 *
 * 黑板端口:
 *  - covariance_threshold (input, double, 默认 0.5): 协方差阈值
 *  - localization_valid (output, bool): 定位是否有效
 *
 * 返回值:
 *  - SUCCESS: 定位正常（协方差在阈值内）
 *  - FAILURE: 定位质量差（协方差超限）或尚未收到数据
 *
 * 注意:
 *  该节点需要 rclcpp::Node::SharedPtr 用于订阅话题。
 *  在注册到工厂时需传入 ROS 节点指针。
 */
class AmclPoseMonitorNode : public BT::ConditionNode
{
public:
  AmclPoseMonitorNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  /// @brief /amcl_pose 话题回调
  void amclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /// @brief 发布告警到 /patrol_alerts
  void publishAlert(const std::string & msg);

  rclcpp::Node::SharedPtr node_;

  /// @brief /amcl_pose 话题订阅
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_sub_;

  /// @brief /patrol_alerts 话题发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;

  /// @brief 缓存最新的协方差矩阵对角元素
  std::array<double, 6> latest_covariance_diag_;
  bool has_covariance_{false};
  std::mutex cov_mutex_;

  /// @brief 默认协方差阈值
  static constexpr double DEFAULT_COVARIANCE_THRESHOLD = 0.5;

  /// @brief 协方差指数滤波平滑系数
  static constexpr double SMOOTHING_ALPHA = 0.3;

  /// @brief 平滑后的协方差值（防止瞬态抖动导致误报）
  double smoothed_cov_{0.0};
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__AMCL_POSE_MONITOR_NODE_HPP_
