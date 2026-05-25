#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

SimulateCharging::SimulateCharging(const std::string& name,
                                   const BT::NodeConfig& config,
                                   std::shared_ptr<PatrolLogger> logger)
    : BT::StatefulActionNode(name, config)
    , logger_(std::move(logger))
{}

BT::PortsList SimulateCharging::providedPorts() {
    return {
        BT::OutputPort<int>("patrol_state"),
        BT::InputPort<double>("recovery_threshold"),
        BT::InputPort<double>("battery_level")
    };
}

BT::NodeStatus SimulateCharging::onStart() {
    auto recovery = getInput<double>("recovery_threshold");
    if (!recovery) {
        throw BT::RuntimeError(
            "SimulateCharging: missing recovery_threshold: ",
            recovery.error());
    }

    recovery_threshold_ = recovery.value();

    // 设置为充电状态
    static constexpr int STATE_CHARGING =
        static_cast<int>(PatrolState::CHARGING);  // = 3
    setOutput<int>("patrol_state", STATE_CHARGING);

    logger_->info("SimulateCharging",
                  "Started charging, recovery threshold = " +
                  std::to_string(recovery_threshold_) + "%");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SimulateCharging::onRunning() {
    auto level = getInput<double>("battery_level");
    if (!level) {
        throw BT::RuntimeError(
            "SimulateCharging: missing battery_level: ",
            level.error());
    }

    if (level.value() >= recovery_threshold_) {
        static constexpr int STATE_PATROLLING =
            static_cast<int>(PatrolState::PATROLLING);  // = 1
        setOutput<int>("patrol_state", STATE_PATROLLING);

        logger_->info("SimulateCharging",
                      "Battery recovered to " +
                      std::to_string(level.value()) +
                      "%, resuming patrol");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void SimulateCharging::onHalted() {
    // 无需特殊处理
}

}  // namespace patrol_bot
