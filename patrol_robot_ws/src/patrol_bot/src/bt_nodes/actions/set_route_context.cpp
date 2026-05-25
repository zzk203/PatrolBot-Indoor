#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

SetRouteContext::SetRouteContext(const std::string& name,
                                 const BT::NodeConfig& config,
                                 std::shared_ptr<PatrolLogger> logger)
    : BT::SyncActionNode(name, config)
    , logger_(std::move(logger))
{}

BT::PortsList SetRouteContext::providedPorts() {
    return {
        BT::InputPort<int>("current_route_index"),
        BT::InputPort<std::vector<Route>>("patrol_routes"),
        BT::OutputPort<Route>("current_route")
    };
}

BT::NodeStatus SetRouteContext::tick() {
    auto route_idx = getInput<int>("current_route_index");
    auto routes = getInput<std::vector<Route>>("patrol_routes");

    if (!route_idx) {
        throw BT::RuntimeError("SetRouteContext: missing current_route_index: ",
                               route_idx.error());
    }
    if (!routes) {
        throw BT::RuntimeError("SetRouteContext: missing patrol_routes: ",
                               routes.error());
    }

    int idx = route_idx.value();
    const auto& all_routes = routes.value();

    if (idx < 0 || idx >= static_cast<int>(all_routes.size())) {
        logger_->warn("SetRouteContext",
                      "Route index " + std::to_string(idx) +
                      " out of range (0-" +
                      std::to_string(all_routes.size() - 1) + ")");
        return BT::NodeStatus::FAILURE;
    }

    setOutput<Route>("current_route", all_routes[idx]);

    logger_->info("SetRouteContext",
                  "Set current route to \"" + all_routes[idx].name +
                  "\" (index " + std::to_string(idx) + ")");

    return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bot
