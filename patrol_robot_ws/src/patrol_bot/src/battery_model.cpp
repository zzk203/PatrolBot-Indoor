#include "patrol_bot/battery_model.hpp"
#include <algorithm>

namespace patrol_bot {

BatteryModel::BatteryModel(rclcpp::Node* node, const BatteryConfig& config)
    : node_(node), config_(config), level_(config.initial_level) {

    battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "/patrol/battery", rclcpp::QoS(10).reliable());

    status_sub_ = node_->create_subscription<patrol_bot_interfaces::msg::PatrolStatus>(
        "/patrol/status", 10,
        [this](const patrol_bot_interfaces::msg::PatrolStatus::SharedPtr msg) {
            status_callback(msg);
        });
}

void BatteryModel::start() {
    timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { timer_callback(); });
}

void BatteryModel::stop() {
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
}

void BatteryModel::timer_callback() {
    // 更新电量
    if (is_charging_) {
        level_ = std::min(level_ + config_.charge_rate, 100.0);
    } else {
        level_ = std::max(level_ - config_.moving_rate, 0.0);
    }

    // 构造并发布消息
    auto msg = sensor_msgs::msg::BatteryState();
    msg.header.stamp = node_->now();
    msg.percentage = level_ / 100.0;
    if (is_charging_) {
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    } else {
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    battery_pub_->publish(msg);
}

void BatteryModel::status_callback(const patrol_bot_interfaces::msg::PatrolStatus::SharedPtr msg) {
    is_charging_ = (msg->state == patrol_bot_interfaces::msg::PatrolStatus::CHARGING);
}

}  // namespace patrol_bot
