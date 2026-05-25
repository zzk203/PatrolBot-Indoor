#include "patrol_bot/bt_nodes.hpp"
#include <string>

namespace patrol_bot {

HandleAlarm::HandleAlarm(const std::string& name,
                         const BT::NodeConfig& config,
                         std::shared_ptr<AlarmManager> alarm_manager,
                         std::shared_ptr<PatrolLogger> logger)
    : BT::SyncActionNode(name, config)
    , alarm_manager_(std::move(alarm_manager))
    , logger_(std::move(logger))
{}

BT::PortsList HandleAlarm::providedPorts() {
    return {
        BT::InputPort<Waypoint>("current_waypoint"),
        BT::OutputPort<int>("patrol_state")
    };
}

BT::NodeStatus HandleAlarm::tick() {
    auto wp = getInput<Waypoint>("current_waypoint");
    if (!wp) {
        throw BT::RuntimeError("HandleAlarm: missing current_waypoint: ",
                               wp.error());
    }

    const auto& waypoint = wp.value();
    if (!waypoint.has_alarm) {
        return BT::NodeStatus::SUCCESS;
    }

    // 调用 AlarmManager 触发报警
    alarm_manager_->raise_alarm(
        waypoint.alarm.type,
        waypoint.alarm.severity,
        waypoint.pose.x,
        waypoint.pose.y);

    if (waypoint.alarm.severity == "critical") {
        // critical 报警：暂停巡逻
        static constexpr int STATE_PAUSED =
            static_cast<int>(PatrolState::PAUSED);  // = 2
        setOutput<int>("patrol_state", STATE_PAUSED);
        logger_->error("HandleAlarm",
                       "Critical alarm triggered at (" +
                       std::to_string(waypoint.pose.x) + ", " +
                       std::to_string(waypoint.pose.y) + "): " +
                       waypoint.alarm.type + " — patrol paused");
    } else {
        // warning 报警：仅记录，不改变巡逻状态
        logger_->warn("HandleAlarm",
                      "Warning alarm triggered at (" +
                      std::to_string(waypoint.pose.x) + ", " +
                      std::to_string(waypoint.pose.y) + "): " +
                      waypoint.alarm.type);
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bot
