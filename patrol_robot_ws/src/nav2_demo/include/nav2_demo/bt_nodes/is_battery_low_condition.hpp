#ifndef NAV2_DEMO__BT_NODES__IS_BATTERY_LOW_CONDITION_HPP_
#define NAV2_DEMO__BT_NODES__IS_BATTERY_LOW_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace nav2_demo
{

/**
 * @brief M4.3 低电判断条件节点
 *
 * 从黑板读取 is_low_battery 标志，判断是否为低电状态。
 * 用于 Selector 中的低电回充分支选择。
 *
 * 黑板端口:
 *  - is_low_battery (input, bool): 低电标志（由 BatteryMonitorNode 维护）
 *
 * 返回值:
 *  - SUCCESS: is_low_battery == true（需要回充）
 *  - FAILURE: is_low_battery == false（电量正常）
 */
class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  IsBatteryLowCondition(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__IS_BATTERY_LOW_CONDITION_HPP_
