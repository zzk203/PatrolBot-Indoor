#ifndef NAV2_DEMO__BT_NODES__SET_CHARGING_MODE_NODE_HPP_
#define NAV2_DEMO__BT_NODES__SET_CHARGING_MODE_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace nav2_demo
{

/**
 * @brief M4.4 设置充电模式 BT 动作节点
 *
 * 通过调用 BatterySimulatorNode 的 /battery_simulator/set_charging 服务
 * 来启用或禁用充电模式。
 *
 * 黑板端口:
 *  - enabled (input, bool, 默认 true): 是否启用充电
 *
 * 返回值:
 *  - SUCCESS: 服务调用成功
 *  - FAILURE: 服务调用失败或服务端未就绪
 */
class SetChargingModeNode : public BT::SyncActionNode
{
public:
  SetChargingModeNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr charging_client_;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__SET_CHARGING_MODE_NODE_HPP_
