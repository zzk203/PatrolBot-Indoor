#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

SavePatrolContext::SavePatrolContext(const std::string& name,
                                     const BT::NodeConfig& config,
                                     std::shared_ptr<PatrolLogger> logger)
    : BT::SyncActionNode(name, config)
    , logger_(std::move(logger))
{}

BT::PortsList SavePatrolContext::providedPorts() {
    return {
        BT::InputPort<int>("current_route_index"),
        BT::InputPort<int>("current_waypoint_idx"),
        BT::OutputPort<int>("saved_route_index"),
        BT::OutputPort<int>("saved_waypoint_idx")
    };
}

BT::NodeStatus SavePatrolContext::tick() {
    auto route_idx = getInput<int>("current_route_index");
    auto wp_idx = getInput<int>("current_waypoint_idx");

    if (!route_idx) {
        throw BT::RuntimeError(
            "SavePatrolContext: missing current_route_index: ",
            route_idx.error());
    }
    if (!wp_idx) {
        throw BT::RuntimeError(
            "SavePatrolContext: missing current_waypoint_idx: ",
            wp_idx.error());
    }

    setOutput<int>("saved_route_index", route_idx.value());
    setOutput<int>("saved_waypoint_idx", wp_idx.value());

    logger_->info("SavePatrolContext",
                  "Saved context: route=" +
                  std::to_string(route_idx.value()) +
                  ", waypoint=" + std::to_string(wp_idx.value()));

    return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bot
