#include "behaviortree_cpp/bt_factory.h"
#include "bt_demo/patrol_nodes.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

int main(int argc, char *argv[])
{
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<patrol_bot::MoveTo>("MoveTo");
  factory.registerNodeType<patrol_bot::CheckBattery>("CheckBattery");
  factory.registerNodeType<patrol_bot::ChargeBattery>("ChargeBattery");
  factory.registerNodeType<patrol_bot::DetectAnomaly>("DetectAnomaly");
  factory.registerNodeType<patrol_bot::ReportAnomaly>("ReportAnomaly");
  factory.registerNodeType<patrol_bot::Wait>("Wait");

  factory.registerSimpleAction("SaySomething",
    [](BT::TreeNode &node) {
      auto msg = node.getInput<std::string>("message");
      std::cout << "  [SaySomething] " << msg.value() << std::endl;
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<std::string>("message")});

  auto tree = [&]() {
    if (argc >= 2) {
      std::string file_path = argv[1];
      if (std::filesystem::exists(file_path)) {
        std::cout << "[加载] 从文件: " << file_path << std::endl;
        return factory.createTreeFromFile(file_path);
      }
    }

    const char *fallback_paths[] = {
      "src/bt_demo/trees/patrol_tree.xml",
      "install/bt_demo/share/bt_demo/trees/patrol_tree.xml",
    };
    for (auto p : fallback_paths) {
      if (std::filesystem::exists(p)) {
        std::cout << "[加载] 从文件: " << p << std::endl;
        return factory.createTreeFromFile(p);
      }
    }

    std::cout << "[加载] 未找到XML文件，使用内置行为树" << std::endl;
    return factory.createTreeFromText(R"XML(
<root BTCPP_format="4">
  <BehaviorTree ID="PatrolDemo">

    <Fallback name="Root">
      <Sequence name="正常巡逻">
        <CheckBattery battery_level="{battery}"/>
        <SaySomething message="电量充足，继续巡逻"/>
      </Sequence>
      <Sequence name="充电流程">
        <SaySomething message="电量低，前往充电站"/>
        <MoveTo waypoint="充电站"/>
        <ChargeBattery/>
        <SaySomething message="充电完成"/>
      </Sequence>
    </Fallback>

  </BehaviorTree>
</root>
    )XML");
  }();

  std::cout << "\n====== 巡逻行为树(多次Tick演示) ======\n" << std::endl;

  for (int tick_num = 1; tick_num <= 10; tick_num++) {
    std::cout << "\n--- Tick " << tick_num << " ---" << std::endl;
    tree.tickOnce();

    float battery = -1.0f;
    tree.rootBlackboard()->get("battery", battery);
    std::cout << "  [状态] 电量=" << battery << "%" << std::endl;
  }

  std::cout << "\n====== 巡逻行为树结束 ======" << std::endl;
  return 0;
}
