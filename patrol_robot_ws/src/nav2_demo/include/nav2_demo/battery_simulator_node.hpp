#ifndef NAV2_DEMO__BATTERY_SIMULATOR_NODE_HPP_
#define NAV2_DEMO__BATTERY_SIMULATOR_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace nav2_demo
{

/**
 * @brief M4.1 电池模拟节点
 *
 * 发布 sensor_msgs/BatteryState，模拟充放电过程：
 * - 放电速率（巡航时）和充电速率可通过 ROS 参数配置
 * - 初始电量 100%，随时间递减
 * - 通过 /battery_simulator/set_charging 服务控制充放电模式
 * - 发布到 /battery_state 话题
 *
 * 参数:
 *  - initial_percentage (double, 默认 100.0): 初始电量百分比 [0, 100]
 *  - discharge_rate (double, 默认 0.5): 放电速率（%/秒）
 *  - charge_rate (double, 默认 5.0): 充电速率（%/秒）
 *  - publish_rate (double, 默认 1.0): 发布频率（Hz）
 *  - simulate_motion_discharge (bool, 默认 false): 是否使用里程计增强放电
 */
class BatterySimulatorNode : public rclcpp::Node
{
public:
  explicit BatterySimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// @brief 定时器回调: 更新电池状态并发布
  void updateBattery();

  /// @brief 设置充放电模式服务回调
  void setChargingCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /// @brief 获取电池电压（线性映射百分比→电压）
  float percentageToVoltage(float percentage) const;

  /// @brief 获取电池电流（充电正/放电负）
  float computeCurrent() const;

  // === 参数 ===
  double initial_percentage_;
  double discharge_rate_;       // %/s
  double charge_rate_;          // %/s
  double publish_rate_;         // Hz
  double voltage_full_;
  double voltage_empty_;

  // === 状态 ===
  double battery_percentage_;   // 0.0 ~ 100.0
  bool charging_{false};
  rclcpp::Time last_update_;

  // === ROS ===
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr charging_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_demo

#endif  // NAV2_DEMO__BATTERY_SIMULATOR_NODE_HPP_
