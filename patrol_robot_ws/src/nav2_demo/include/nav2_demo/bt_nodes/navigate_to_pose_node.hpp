#ifndef NAV2_DEMO__BT_NODES__NAVIGATE_TO_POSE_NODE_HPP_
#define NAV2_DEMO__BT_NODES__NAVIGATE_TO_POSE_NODE_HPP_

#include <memory>
#include <string>
#include <atomic>
#include <future>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_demo
{

/**
 * @brief M3.3 封装 navigate_to_pose 动作客户端的行为树节点
 *
 * 基类: StatefulActionNode (支持异步操作)
 *
 * 功能:
 *  - onStart(): 解析输入端口，创建/复用动作客户端，发送导航目标
 *  - onRunning(): 检查结果是否到达，检查超时
 *  - onHalted(): 取消当前导航目标
 *
 * 黑板端口:
 *  - goal (input, PoseStamped): 导航目标位姿
 *  - timeout (input, double, 默认 30.0): 导航超时秒数
 *  - nav_result (output, string): 结果 "SUCCESS" / "FAILURE" / "TIMEOUT"
 *
 * 返回值:
 *  - SUCCESS: 导航成功完成
 *  - FAILURE: 导航被中止或拒绝
 *  - RUNNING: 导航进行中
 *  - 超时时返回 FAILURE 并设置 nav_result="TIMEOUT"
 */
class NavigateToPoseNode : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// @brief 取消当前导航目标（线程安全）
  void cancelGoal();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // 异步操作状态
  std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;
  std::atomic<bool> result_received_{false};
  std::atomic<bool> goal_accepted_{false};
  GoalHandleNav::WrappedResult result_;

  rclcpp::Time start_time_;
  double timeout_{30.0};
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__NAVIGATE_TO_POSE_NODE_HPP_
