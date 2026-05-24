#ifndef NAV2_DEMO__BT_NODES__RECORD_FAILURE_NODE_HPP_
#define NAV2_DEMO__BT_NODES__RECORD_FAILURE_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_demo
{

/**
 * @brief M3.5 记录堵赛/失败事件到黑板
 *
 * 在 Fallback 中作为备选子节点：
 *  当 NavigateToPoseNode 超时/失败后执行，
 *  将失败信息写入黑板并打印日志，然后返回 SUCCESS
 *  （确保 Fallback 返回 SUCCESS，巡逻继续）。
 *
 * 黑板端口:
 *  - failure_count (input/output, int): 累计失败次数，自增
 *  - last_failure_reason (output, string): 最近一次失败原因
 *  - last_failure_waypoint (input, string): 失败的航点名称/描述
 *
 * 输入端口:
 *  - reason (input, string, 默认 "timeout"): 失败原因描述
 */
class RecordFailureNode : public BT::SyncActionNode
{
public:
  RecordFailureNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__RECORD_FAILURE_NODE_HPP_
