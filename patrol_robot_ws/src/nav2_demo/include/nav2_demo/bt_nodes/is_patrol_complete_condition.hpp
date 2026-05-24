#ifndef NAV2_DEMO__BT_NODES__IS_PATROL_COMPLETE_CONDITION_HPP_
#define NAV2_DEMO__BT_NODES__IS_PATROL_COMPLETE_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace nav2_demo
{

/**
 * @brief M6.1 巡逻完成判断条件节点
 *
 * 从黑板读取 patrol_complete 标志，判断巡逻是否完成。
 * 用于轮次调度中，在 PatrolRound 完成所有巡逻点后
 * 通过 Inverter 反转返回 FAILURE，从而跳出内层 Repeat 循环。
 *
 * 黑板端口:
 *  - patrol_complete (input, bool): 巡逻完成标志（由 PatrolRound 设置）
 *
 * 返回值:
 *  - SUCCESS: patrol_complete == true（巡逻已完成）
 *  - FAILURE: patrol_complete == false（巡逻进行中）
 */
class IsPatrolCompleteCondition : public BT::ConditionNode
{
public:
  IsPatrolCompleteCondition(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__IS_PATROL_COMPLETE_CONDITION_HPP_
