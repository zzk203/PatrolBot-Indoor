#include "patrol_bot/bt_nodes.hpp"

namespace patrol_bot {

IsBatteryLow::IsBatteryLow(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList IsBatteryLow::providedPorts() {
    return {
        BT::InputPort<double>("battery_level"),
        BT::InputPort<double>("low_threshold")
    };
}

BT::NodeStatus IsBatteryLow::tick() {
    auto battery_level = getInput<double>("battery_level");
    auto low_threshold = getInput<double>("low_threshold");

    if (!battery_level) {
        throw BT::RuntimeError("IsBatteryLow: missing battery_level: ",
                               battery_level.error());
    }
    if (!low_threshold) {
        throw BT::RuntimeError("IsBatteryLow: missing low_threshold: ",
                               low_threshold.error());
    }

    return (battery_level.value() < low_threshold.value())
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace patrol_bot
