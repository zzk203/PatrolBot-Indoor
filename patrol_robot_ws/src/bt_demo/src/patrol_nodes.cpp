#include "bt_demo/patrol_nodes.hpp"
#include <chrono>
#include <iostream>
#include <thread>

namespace patrol_bot {

BT::NodeStatus MoveTo::tick() {
  auto waypoint = getInput<std::string>("waypoint");
  std::string wp_name = waypoint.value_or("unknown");
  std::cout << "  [MoveTo] 移动到: " << wp_name << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckBattery::tick() {
  static int call_count = 0;
  auto is_charge_battery = getInput<bool>("is_charge_battery");

  float level = 100.0f;
  if (!is_charge_battery.value_or(false))
    level -= call_count * 25.0f;
  if (level < 0.0f)
    level = 0.0f;
  call_count = (call_count + 1) % 6;

  setOutput("battery_level", level);
  std::cout << "  [CheckBattery] 当前电量: " << level << "%" << std::endl;

  if (level < 20.0f) {
    std::cout << "    -> 低电量，需要充电!" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  std::cout << "    -> 电量充足" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ChargeBattery::tick() {
  std::cout << "  [ChargeBattery] 正在充电中..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  std::cout << "  [ChargeBattery] 充电完成!" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DetectAnomaly::tick() {
  static int round = 0;
  round++;
  if (round % 3 == 0) {
    std::cout << "  [DetectAnomaly] 发现异常!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout << "  [DetectAnomaly] 一切正常" << std::endl;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ReportAnomaly::tick() {
  std::cout << "  [ReportAnomaly] 上报异常至管理中心..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  std::cout << "  [ReportAnomaly] 上报完成" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Wait::onStart() {
  auto secs = getInput<int>("seconds");
  target_ = secs.value_or(1);
  count_ = 0;
  std::cout << "  [Wait] 等待 " << target_ << " 秒..." << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning() {
  std::this_thread::sleep_for(std::chrono::seconds(1));
  count_++;
  std::cout << "  [Wait] " << count_ << "/" << target_ << "s" << std::endl;
  if (count_ >= target_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void Wait::onHalted() { std::cout << "  [Wait] 被中断!" << std::endl; }

} // namespace patrol_bot
