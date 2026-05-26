#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

NavigateToWaypoint::NavigateToWaypoint(
    const std::string &name, const BT::NodeConfig &config,
    std::shared_ptr<Nav2ActionClient> nav2_client,
    std::shared_ptr<PatrolLogger> logger)
    : BT::StatefulActionNode(name, config),
      nav2_client_(std::move(nav2_client)), logger_(std::move(logger)) {}

BT::PortsList NavigateToWaypoint::providedPorts() {
  return {BT::InputPort<Route>("current_route"),
          BT::InputPort<int>("current_waypoint_index"),
          BT::InputPort<double>("nav_timeout"),
          BT::OutputPort<Waypoint>("current_waypoint"),
          BT::OutputPort<int>("current_waypoint_index")};
}

BT::NodeStatus NavigateToWaypoint::onStart() {
  auto route = getInput<Route>("current_route");
  auto wp_idx = getInput<int>("current_waypoint_index");
  auto timeout = getInput<double>("nav_timeout");

  if (!route) {
    throw BT::RuntimeError("NavigateToWaypoint: missing current_route: ",
                           route.error());
  }
  if (!wp_idx) {
    throw BT::RuntimeError(
        "NavigateToWaypoint: missing current_waypoint_index: ", wp_idx.error());
  }
  if (!timeout) {
    throw BT::RuntimeError("NavigateToWaypoint: missing nav_timeout: ",
                           timeout.error());
  }

  const auto &waypoints = route.value().waypoints;
  int idx = wp_idx.value();

  if (idx < 0 || idx >= static_cast<int>(waypoints.size())) {
    logger_->warn("NavigateToWaypoint",
                  "Waypoint index " + std::to_string(idx) + " out of range");
    return BT::NodeStatus::FAILURE;
  }

  const auto &wp = waypoints[idx];
  setOutput<Waypoint>("current_waypoint", wp);

  nav_timeout_ = timeout.value();
  start_time_ = std::chrono::steady_clock::now();
  waypoint_count_ = static_cast<int>(waypoints.size());

  const auto &pose = wp.pose;
  target_x_ = pose.x;
  target_y_ = pose.y;
  target_yaw_ = pose.yaw;
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
  case NavResult::NOT_STARTED:
    return BT::NodeStatus::RUNNING;

  case NavResult::SUCCESS: {
    logger_->info("NavigateToWaypoint", "Navigation succeeded");

    auto wp_idx = getInput<int>("current_waypoint_index");
    int next_idx = wp_idx.value() + 1;
    if (next_idx >= waypoint_count_) {
      next_idx = 0;
    }
    setOutput<int>("current_waypoint_index", next_idx);

    return BT::NodeStatus::SUCCESS;
  }

  case NavResult::FAILURE:
  case NavResult::ERROR:
    logger_->warn("NavigateToWaypoint", "Navigation failed");
    return BT::NodeStatus::FAILURE;

  default:
    break;
  }

  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::duration<double>(nav_timeout_)) {
    logger_->warn("NavigateToWaypoint", "Navigation timeout after " +
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

} // namespace patrol_bot
