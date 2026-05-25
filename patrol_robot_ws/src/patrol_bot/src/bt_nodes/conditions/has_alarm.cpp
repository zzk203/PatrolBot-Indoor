#include "patrol_bot/bt_nodes.hpp"

namespace patrol_bot {

HasAlarm::HasAlarm(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList HasAlarm::providedPorts() {
    return {
        BT::InputPort<Waypoint>("current_waypoint")
    };
}

BT::NodeStatus HasAlarm::tick() {
    auto wp = getInput<Waypoint>("current_waypoint");
    if (!wp) {
        throw BT::RuntimeError("HasAlarm: missing current_waypoint: ",
                               wp.error());
    }

    return wp.value().has_alarm
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace patrol_bot
