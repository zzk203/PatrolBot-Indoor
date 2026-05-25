#include "patrol_bot/bt_nodes.hpp"

namespace patrol_bot {

WaitAtWaypoint::WaitAtWaypoint(const std::string& name,
                               const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{}

BT::PortsList WaitAtWaypoint::providedPorts() {
    return {
        BT::InputPort<Waypoint>("current_waypoint")
    };
}

BT::NodeStatus WaitAtWaypoint::onStart() {
    auto wp = getInput<Waypoint>("current_waypoint");
    if (!wp) {
        throw BT::RuntimeError(
            "WaitAtWaypoint: missing current_waypoint: ",
            wp.error());
    }

    double wait_seconds = wp.value().wait_seconds;
    if (wait_seconds <= 0.0) {
        return BT::NodeStatus::SUCCESS;
    }

    wait_duration_ = wait_seconds;
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAtWaypoint::onRunning() {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed >= std::chrono::duration<double>(wait_duration_)) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitAtWaypoint::onHalted() {
    // 无需特殊清理
}

}  // namespace patrol_bot
