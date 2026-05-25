#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

LoadRoutes::LoadRoutes(const std::string& name, const BT::NodeConfig& config,
                       std::shared_ptr<PatrolLogger> logger)
    : BT::SyncActionNode(name, config)
    , logger_(std::move(logger))
{}

BT::PortsList LoadRoutes::providedPorts() {
    return {
        BT::InputPort<Config>("config"),
        BT::OutputPort<std::vector<Route>>("patrol_routes"),
        BT::OutputPort<Pose2D>("charging_station"),
        BT::OutputPort<double>("low_threshold"),
        BT::OutputPort<double>("recovery_threshold"),
        BT::OutputPort<double>("nav_timeout")
    };
}

BT::NodeStatus LoadRoutes::tick() {
    auto config = getInput<Config>("config");
    if (!config) {
        throw BT::RuntimeError("LoadRoutes: missing config: ",
                               config.error());
    }

    const auto& cfg = config.value();

    setOutput<std::vector<Route>>("patrol_routes", cfg.routes);
    setOutput<Pose2D>("charging_station", cfg.charging_station);
    setOutput<double>("low_threshold", cfg.battery.low_threshold);
    setOutput<double>("recovery_threshold", cfg.battery.recovery_threshold);
    setOutput<double>("nav_timeout", cfg.waypoint_timeout);

    logger_->info("LoadRoutes",
                  "Loaded " + std::to_string(cfg.routes.size()) + " patrol route(s)");

    return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bot
