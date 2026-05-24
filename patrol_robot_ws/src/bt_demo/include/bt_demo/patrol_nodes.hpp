#ifndef BT_DEMO__PATROL_NODES_HPP_
#define BT_DEMO__PATROL_NODES_HPP_

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"

namespace patrol_bot {

class MoveTo : public BT::SyncActionNode {
public:
  MoveTo(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("waypoint")};
  }

  BT::NodeStatus tick() override;
};

class CheckBattery : public BT::ConditionNode {
public:
  CheckBattery(const std::string &name, const BT::NodeConfig &config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::OutputPort<float>("battery_level"),
            BT::InputPort<bool>("is_charge_battery")};
  }

  BT::NodeStatus tick() override;
};

class ChargeBattery : public BT::SyncActionNode {
public:
  ChargeBattery(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

class DetectAnomaly : public BT::SyncActionNode {
public:
  DetectAnomaly(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

class ReportAnomaly : public BT::SyncActionNode {
public:
  ReportAnomaly(const std::string &name, const BT::NodeConfig &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

class Wait : public BT::StatefulActionNode {
public:
  Wait(const std::string &name, const BT::NodeConfig &config)
      : BT::StatefulActionNode(name, config), count_(0) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("seconds")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int count_;
  int target_;
};

} // namespace patrol_bot

#endif // BT_DEMO__PATROL_NODES_HPP_
