#include "patrol_bot/bt_nodes.hpp"

namespace patrol_bot {

IsAlarmCritical::IsAlarmCritical(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList IsAlarmCritical::providedPorts() {
    return {
        BT::InputPort<Waypoint>("current_waypoint")
    };
}

BT::NodeStatus IsAlarmCritical::tick() {
    auto wp = getInput<Waypoint>("current_waypoint");
    if (!wp) {
        throw BT::RuntimeError("IsAlarmCritical: missing current_waypoint: ",
                               wp.error());
    }

    const auto& waypoint = wp.value();
    if (!waypoint.has_alarm) {
        return BT::NodeStatus::FAILURE;
    }

    return (waypoint.alarm.severity == "critical")
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace patrol_bot
