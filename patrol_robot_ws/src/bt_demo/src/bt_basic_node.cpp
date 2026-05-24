#include "behaviortree_cpp/bt_factory.h"
#include <chrono>
#include <iostream>

class SaySomething : public BT::SyncActionNode {
public:
  SaySomething(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override {
    auto msg = getInput<std::string>("message");
    std::cout << "  [SaySomething] " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class FlakyDetector : public BT::SyncActionNode {
public:
  FlakyDetector(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    attempt_++;
    std::cout << "  [FlakyDetector] 第 " << attempt_ << " 次尝试" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (attempt_ >= 3) {
      std::cout << "  [FlakyDetector] 检测成功!" << std::endl;
      attempt_ = 0;
      return BT::NodeStatus::SUCCESS;
    }
    std::cout << "  [FlakyDetector] 检测失败，继续重试..." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

private:
  int attempt_ = 0;
};

int main() {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<FlakyDetector>("FlakyDetector");

  const std::string tree_xml = R"XML(
<root BTCPP_format="4">
  <BehaviorTree ID="BasicDemo">

    <Sequence name="Root">

      <SaySomething message="====================================="/>
      <SaySomething message="  BT.CPP v4 核心概念演示"/>
      <SaySomething message="====================================="/>

      <SaySomething message=""/>
      <SaySomething message="--- 演示1: Sequence (顺序执行) ---"/>
      <SaySomething message="特点: 子节点按顺序执行，任一失败则中断"/>

      <ForceSuccess>
      <Sequence name="演示1_Sequence">
        <SaySomething message="步骤1/3: 初始化传感器"/>
        <SaySomething message="步骤2/3: 加载地图"/>
        <SaySomething message="步骤3/3: 就绪"/>
      </Sequence>
      </ForceSuccess>

      <SaySomething message="[演示1 结果] Sequence 所有节点执行成功"/>
      <SaySomething message=""/>

      <SaySomething message="--- 演示2: Fallback (选择执行) ---"/>
      <SaySomething message="特点: 依次尝试子节点，任一成功则停止"/>
      <SaySomething message="场景: 尝试开门策略，三种方案选一种能用的"/>

      <Fallback name="演示2_Fallback">
        <ForceFailure>
          <SaySomething message="方案A: 人脸识别 → 失败"/>
        </ForceFailure>
        <ForceFailure>
        <SaySomething message="方案B: 指纹识别 → 失败"/>
        </ForceFailure>
        <SaySomething message="方案C: 钥匙开门 → 成功！"/>
        <SaySomething message="方案D: 无 → 上面运行成功，该行动不执行"/>
      </Fallback>

      <SaySomething message="[演示2 结果] Fallback 前2个失败，第3个成功"/>
      <SaySomething message=""/>

      <SaySomething message="--- 演示3: ForceSuccess (强制成功) ---"/>
      <SaySomething message="特点: 无论子节点什么结果，都返回 SUCCESS"/>
      <SaySomething message="场景: 开门失败不阻塞巡逻，忽略继续"/>

      <ForceSuccess name="演示3_ForceSuccess">
        <Sequence>
          <ForceFailure>
            <SaySomething message="尝试推门 → 推不动!"/>
          </ForceFailure>
          <SaySomething message="这条不会执行(Sequence已中断)"/>
        </Sequence>
      </ForceSuccess>

      <SaySomething message="[演示3 结果] ForceSuccess 将子节点失败转成功，巡逻继续"/>
      <SaySomething message=""/>

      <SaySomething message="--- 演示4: RetryUntilSuccessful (重试) ---"/>
      <SaySomething message="特点: 子节点返回失败则重试，直到成功或达上限"/>
      <SaySomething message="场景: 传感器不稳定，需要重试检测"/>

      <RetryUntilSuccessful num_attempts="5">
        <FlakyDetector name="不稳定传感器"/>
      </RetryUntilSuccessful>

      <SaySomething message="[演示4 结果] 重试3次后传感器检测成功"/>
      <SaySomething message=""/>

      <SaySomething message="--- 全部演示完毕 ---"/>

    </Sequence>

  </BehaviorTree>
</root>
  )XML";

  auto tree = factory.createTreeFromText(tree_xml);

  std::cout << "\n====== 行为树开始 tick ======\n" << std::endl;

  BT::NodeStatus status = tree.tickOnce();

  std::cout << "\n====== 行为树结束 ======" << std::endl;
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
