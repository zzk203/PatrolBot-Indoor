#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

NavigateToCharger::NavigateToCharger(const std::string& name,
                                     const BT::NodeConfig& config,
                                     std::shared_ptr<Nav2ActionClient> nav2_client,
                                     std::shared_ptr<PatrolLogger> logger)
    : BT::StatefulActionNode(name, config)
    , nav2_client_(std::move(nav2_client))
    , logger_(std::move(logger))
{}

BT::PortsList NavigateToCharger::providedPorts() {
    return {
        BT::InputPort<Pose2D>("charging_station")
    };
}

BT::NodeStatus NavigateToCharger::onStart() {
    auto station = getInput<Pose2D>("charging_station");
    if (!station) {
        throw BT::RuntimeError(
            "NavigateToCharger: missing charging_station: ",
            station.error());
    }

    const auto& pose = station.value();
    nav2_client_->send_goal(pose.x, pose.y, pose.yaw);

    logger_->info("NavigateToCharger",
                  "Navigating to charger at (" +
                  std::to_string(pose.x) + ", " +
                  std::to_string(pose.y) + ", " +
                  std::to_string(pose.yaw) + ")");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToCharger::onRunning() {
    auto result = nav2_client_->check_result();

    switch (result) {
        case NavResult::SUCCESS:
            logger_->info("NavigateToCharger", "Reached charger");
            return BT::NodeStatus::SUCCESS;

        case NavResult::FAILURE:
        case NavResult::ERROR:
            logger_->warn("NavigateToCharger", "Failed to reach charger");
            return BT::NodeStatus::FAILURE;

        default:
            return BT::NodeStatus::RUNNING;
    }
}

void NavigateToCharger::onHalted() {
    nav2_client_->cancel_goal();
    logger_->warn("NavigateToCharger", "Navigation to charger halted");
}

}  // namespace patrol_bot
