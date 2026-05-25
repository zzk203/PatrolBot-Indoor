#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

RestorePatrolContext::RestorePatrolContext(const std::string& name,
                                           const BT::NodeConfig& config,
                                           std::shared_ptr<PatrolLogger> logger)
    : BT::SyncActionNode(name, config)
    , logger_(std::move(logger))
{}

BT::PortsList RestorePatrolContext::providedPorts() {
    return {
        BT::InputPort<int>("saved_route_index"),
        BT::InputPort<int>("saved_waypoint_idx"),
        BT::OutputPort<int>("current_route_index"),
        BT::OutputPort<int>("current_waypoint_idx"),
        BT::OutputPort<Waypoint>("current_waypoint"),
        BT::InputPort<std::vector<Route>>("patrol_routes")
    };
}

BT::NodeStatus RestorePatrolContext::tick() {
    auto saved_route = getInput<int>("saved_route_index");
    auto saved_wp = getInput<int>("saved_waypoint_idx");
    auto routes = getInput<std::vector<Route>>("patrol_routes");

    if (!routes) {
        throw BT::RuntimeError("RestorePatrolContext: missing patrol_routes: ",
                               routes.error());
    }

    // 有效断点：saved_route_index != -1 && saved_waypoint_idx != -1
    if (saved_route && saved_wp &&
        saved_route.value() != -1 && saved_wp.value() != -1)
    {
        int route_idx = saved_route.value();
        int wp_idx = saved_wp.value();

        setOutput<int>("current_route_index", route_idx);
        setOutput<int>("current_waypoint_idx", wp_idx);

        // 恢复 current_waypoint
        const auto& all_routes = routes.value();
        if (route_idx >= 0 &&
            route_idx < static_cast<int>(all_routes.size()) &&
            wp_idx >= 0 &&
            wp_idx < static_cast<int>(all_routes[route_idx].waypoints.size()))
        {
            setOutput<Waypoint>("current_waypoint",
                                all_routes[route_idx].waypoints[wp_idx]);
        }

        // 清除 saved 断点
        config().blackboard->set<int>("saved_route_index", -1);
        config().blackboard->set<int>("saved_waypoint_idx", -1);

        logger_->info("RestorePatrolContext",
                      "Restored checkpoint: route=" +
                      std::to_string(route_idx) +
                      ", waypoint=" + std::to_string(wp_idx));
    } else {
        // 无断点，从头开始
        setOutput<int>("current_route_index", 0);
        setOutput<int>("current_waypoint_idx", 0);

        logger_->info("RestorePatrolContext",
                      "No saved checkpoint, starting from beginning");
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bot
