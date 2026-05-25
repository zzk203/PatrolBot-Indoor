#pragma once

#include "patrol_bot/patrol_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "patrol_bot_interfaces/msg/patrol_status.hpp"

namespace patrol_bot {

class BatteryModel {
public:
    BatteryModel(rclcpp::Node* node,
                 const BatteryConfig& config);

    void start();
    void stop();

private:
    void timer_callback();
    void status_callback(const patrol_bot_interfaces::msg::PatrolStatus::SharedPtr msg);

    rclcpp::Node* node_;
    BatteryConfig config_;
    double level_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Subscription<patrol_bot_interfaces::msg::PatrolStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_charging_ = false;
};

}  // namespace patrol_bot
