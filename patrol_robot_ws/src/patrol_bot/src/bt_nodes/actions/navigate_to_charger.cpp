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
{
    using namespace std::chrono_literals;
    logger_->info("NavigateToCharger", "Waiting for Nav2 action server...");
    if (!nav2_client_->wait_for_server(10s)) {
        logger_->warn("NavigateToCharger",
                  "Nav2 action server not ready after 10s, retrying...");
        bool ready = false;
        for (int i = 0; i < 3; ++i) {
            if (nav2_client_->wait_for_server(5s)) {
            ready = true;
            break;
        }
    }
        if (!ready) {
            logger_->error("NavigateToCharger", "Nav2 action server unavailable after retries");
            throw std::runtime_error("Nav2 action server not ready");
        }
    }
    logger_->info("NavigateToCharger", "Nav2 action server is ready");
}

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
            logger_->warn("NavigateToCharger", "Failed to reach charger by FAILURE");
            return BT::NodeStatus::FAILURE;
        case NavResult::ERROR:
            logger_->warn("NavigateToCharger", "Failed to reach charger by ERROR");
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
