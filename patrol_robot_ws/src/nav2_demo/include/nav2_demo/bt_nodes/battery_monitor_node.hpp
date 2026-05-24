#ifndef NAV2_DEMO__BT_NODES__BATTERY_MONITOR_NODE_HPP_
#define NAV2_DEMO__BT_NODES__BATTERY_MONITOR_NODE_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav2_demo
{

/**
 * @brief M4.2 电池状态监测行为树条件节点
 *
 * 功能:
 *  - 订阅 /battery_state 话题，缓存最新电池状态
 *  - 在 tick() 中将当前电量与阈值比较
 *  - 将电量水平和低电标志写入黑板供其他节点使用
 *
 * 黑板端口:
 *  - battery_threshold (input, float, 默认 20.0): 低电阈值百分比
 *  - battery_level (output, float): 当前电池电量 [0, 100]
 *  - is_low_battery (output, bool): 是否低电（低于阈值）
 *
 * 返回值:
 *  - SUCCESS: 电量 >= 阈值（电池正常）
 *  - FAILURE: 电量 < 阈值（低电）
 *
 * 注意:
 *  该节点需要 rclcpp::Node::SharedPtr 用于订阅话题。
 *  在注册到工厂时需传入 ROS 节点指针。
 */
class BatteryMonitorNode : public BT::ConditionNode
{
public:
  BatteryMonitorNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  /// @brief 电池话题回调
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  // 电池话题订阅
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // 缓存最新电池状态
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_msg_;
  std::mutex battery_mutex_;

  // M7.4: 话题超时检测
  rclcpp::Time last_msg_time_;
  static constexpr double BATTERY_TIMEOUT_SECONDS = 5.0;

  // M7.4: /patrol_alerts 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__BATTERY_MONITOR_NODE_HPP_
