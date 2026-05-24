#ifndef NAV2_DEMO__BT_NODES__LOG_ALERT_NODE_HPP_
#define NAV2_DEMO__BT_NODES__LOG_ALERT_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_demo
{

/**
 * @brief M7.1 日志告警行为树节点
 *
 * 接受 alert_msg 输入，写入 ROS2 日志（RCLCPP_ERROR/WARN/INFO），
 * 同时发布到 /patrol_alerts 话题。
 *
 * 黑板端口:
 *  - alert_msg (input, string, required): 告警消息内容
 *  - severity (input, string, 默认 "ERROR"): 日志级别 (ERROR/WARN/INFO)
 *
 * 返回值:
 *  - 始终返回 SUCCESS（fail-safe，不中断主流程）
 */
class LogAlertNode : public BT::SyncActionNode
{
public:
  LogAlertNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  /// @brief 发布告警消息到 /patrol_alerts 话题
  void publishAlert(const std::string & msg);

  rclcpp::Node::SharedPtr node_;

  /// @brief /patrol_alerts 话题发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__LOG_ALERT_NODE_HPP_
