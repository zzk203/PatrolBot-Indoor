#ifndef NAV2_DEMO__BT_NODES__RETRY_NODE_HPP_
#define NAV2_DEMO__BT_NODES__RETRY_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_demo
{

/**
 * @brief M5.3 重试装饰节点
 *
 * 包装一个子节点，当子节点返回 FAILURE 时自动重试。
 * 最多重试 max_attempts 次（包括首次执行），
 * 如果所有尝试均失败，记录报警日志并返回 FAILURE。
 *
 * 用法:
 *  <RetryNode max_attempts="3">
 *    <VisualServoNode .../>
 *  </RetryNode>
 *
 * 黑板端口:
 *  - max_attempts (input, int, 默认 3): 最大尝试次数
 *  - retry_count (output, int): 当前已尝试次数（从 1 开始计数）
 *
 * 返回值:
 *  - SUCCESS: 子节点在重试次数内成功
 *  - FAILURE: 所有重试均失败
 *  - RUNNING: 子节点正在执行，或准备下一次重试
 */
class RetryNode : public BT::DecoratorNode
{
public:
  RetryNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  int max_attempts_{3};
  int attempt_count_{0};
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BT_NODES__RETRY_NODE_HPP_
