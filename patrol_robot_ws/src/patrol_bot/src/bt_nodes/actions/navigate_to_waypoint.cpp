#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

NavigateToWaypoint::NavigateToWaypoint(const std::string& name,
                                       const BT::NodeConfig& config,
                                       std::shared_ptr<Nav2ActionClient> nav2_client,
                                       std::shared_ptr<PatrolLogger> logger)
    : BT::StatefulActionNode(name, config)
    , nav2_client_(std::move(nav2_client))
    , logger_(std::move(logger))
{}

BT::PortsList NavigateToWaypoint::providedPorts() {
    return {
        BT::InputPort<Waypoint>("current_waypoint"),
        BT::InputPort<double>("nav_timeout")
    };
}

BT::NodeStatus NavigateToWaypoint::onStart() {
    auto wp = getInput<Waypoint>("current_waypoint");
    auto timeout = getInput<double>("nav_timeout");

    if (!wp) {
        throw BT::RuntimeError(
            "NavigateToWaypoint: missing current_waypoint: ",
            wp.error());
    }
    if (!timeout) {
        throw BT::RuntimeError(
            "NavigateToWaypoint: missing nav_timeout: ",
            timeout.error());
    }

    nav_timeout_ = timeout.value();
    start_time_ = std::chrono::steady_clock::now();

    const auto& pose = wp.value().pose;
    nav2_client_->send_goal(pose.x, pose.y, pose.yaw);

    logger_->info("NavigateToWaypoint",
                  "Navigating to (" + std::to_string(pose.x) + ", " +
                  std::to_string(pose.y) + ", " + std::to_string(pose.yaw) +
                  ") with timeout " + std::to_string(nav_timeout_) + "s");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaypoint::onRunning() {
    auto result = nav2_client_->check_result();

    switch (result) {
        case NavResult::SUCCESS:
            logger_->info("NavigateToWaypoint", "Navigation succeeded");
            return BT::NodeStatus::SUCCESS;

        case NavResult::FAILURE:
        case NavResult::ERROR:
            logger_->warn("NavigateToWaypoint", "Navigation failed");
            return BT::NodeStatus::FAILURE;

        default:
            break;
    }

    // 超时检测
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed > std::chrono::duration<double>(nav_timeout_)) {
        logger_->warn("NavigateToWaypoint",
                      "Navigation timeout after " +
                      std::to_string(nav_timeout_) + "s");
        nav2_client_->cancel_goal();
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void NavigateToWaypoint::onHalted() {
    nav2_client_->cancel_goal();
    logger_->warn("NavigateToWaypoint", "Navigation halted");
}

}  // namespace patrol_bot
