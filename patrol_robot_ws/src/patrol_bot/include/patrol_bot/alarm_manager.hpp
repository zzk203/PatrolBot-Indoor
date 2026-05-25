#pragma once

#include <rclcpp/rclcpp.hpp>
#include "patrol_bot_interfaces/msg/patrol_alarm.hpp"
#include "patrol_bot/patrol_logger.hpp"

namespace patrol_bot {

class AlarmManager {
public:
    AlarmManager(rclcpp::Node* node, PatrolLogger& logger);

    /// @brief 触发报警
    /// @param type  报警类型 (e.g. "smoke_detected", "temperature_high")
    /// @param severity "warning" | "critical"
    /// @param x, y  触发位置坐标
    void raise_alarm(const std::string& type,
                     const std::string& severity,
                     double x, double y);

private:
    rclcpp::Publisher<patrol_bot_interfaces::msg::PatrolAlarm>::SharedPtr alarm_pub_;
    PatrolLogger& logger_;
};

}  // namespace patrol_bot
