#include "patrol_bot/alarm_manager.hpp"
#include <string>

namespace patrol_bot {

AlarmManager::AlarmManager(rclcpp::Node* node, PatrolLogger& logger)
    : logger_(logger)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    alarm_pub_ = node->create_publisher<patrol_bot_interfaces::msg::PatrolAlarm>(
        "/patrol/alarm", qos);
}

void AlarmManager::raise_alarm(const std::string& type,
                               const std::string& severity,
                               double x, double y)
{
    patrol_bot_interfaces::msg::PatrolAlarm msg;

    if (severity == "critical") {
        msg.severity = patrol_bot_interfaces::msg::PatrolAlarm::CRITICAL;
        logger_.error("AlarmManager",
            "CRITICAL alarm: type=" + type +
            " at (" + std::to_string(x) + ", " + std::to_string(y) + ")");
    } else {
        msg.severity = patrol_bot_interfaces::msg::PatrolAlarm::WARNING;
        logger_.warn("AlarmManager",
            "WARNING alarm: type=" + type +
            " at (" + std::to_string(x) + ", " + std::to_string(y) + ")");
    }

    msg.alarm_type = type;
    msg.x = static_cast<float>(x);
    msg.y = static_cast<float>(y);
    msg.timestamp = rclcpp::Clock().now();

    alarm_pub_->publish(msg);
}

}  // namespace patrol_bot
