#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include <gtest/gtest.h>

class TestAction : public BT::SyncActionNode
{
public:
  TestAction(const std::string &name, const BT::NodeConfig &config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("value")};
  }

  BT::NodeStatus tick() override
  {
    auto val = getInput<int>("value");
    return val.value() == 1 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

TEST(BasicTreeTest, SequenceAllSuccess)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestAction>("TestAction");

  const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestSeq">
    <Sequence>
      <TestAction value="1"/>
      <TestAction value="1"/>
      <TestAction value="1"/>
    </Sequence>
  </BehaviorTree>
</root>
  )";

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BasicTreeTest, SequenceFailureShortCircuits)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestAction>("TestAction");

  const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestSeqFail">
    <Sequence>
      <TestAction value="0"/>
      <TestAction value="1"/>
    </Sequence>
  </BehaviorTree>
</root>
  )";

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(BasicTreeTest, FallbackFirstSuccessShortCircuits)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestAction>("TestAction");

  const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestFallback">
    <Fallback>
      <TestAction value="1"/>
      <TestAction value="0"/>
    </Fallback>
  </BehaviorTree>
</root>
  )";

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(BasicTreeTest, FallbackAllFailure)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestAction>("TestAction");

  const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestFallbackAllFail">
    <Fallback>
      <TestAction value="0"/>
      <TestAction value="0"/>
    </Fallback>
  </BehaviorTree>
</root>
  )";

  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
