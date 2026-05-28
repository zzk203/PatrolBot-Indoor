#include "patrol_bot/bt_nodes.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace patrol_bot {

CaptureImage::CaptureImage(const std::string& name,
                           const BT::NodeConfig& config,
                           std::shared_ptr<CameraBuffer> camera_buffer,
                           std::shared_ptr<PatrolLogger> logger)
    : BT::SyncActionNode(name, config)
    , camera_buffer_(std::move(camera_buffer))
    , logger_(std::move(logger))
{}

BT::PortsList CaptureImage::providedPorts() {
    return {
        BT::InputPort<int>("current_route_index"),
        BT::InputPort<int>("current_waypoint_index")
    };
}

BT::NodeStatus CaptureImage::tick() {
    auto route_idx = getInput<int>("current_route_index");
    auto wp_idx = getInput<int>("current_waypoint_index");

    // 生成文件名: route{R}_wp{W}_{YYYYMMDD_HHMMSS}
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t_now);

    std::ostringstream filename;
    filename << "route" << route_idx.value_or(0)
             << "_wp" << wp_idx.value_or(0)
             << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S");

    bool saved = camera_buffer_->save_latest(filename.str());

    if (saved) {
        logger_->info("CaptureImage",
                      "Saved image: " + filename.str());
    } else {
        logger_->warn("CaptureImage",
                      "No frame available, skipping image capture");
    }

    // 无帧时降级，依然返回 SUCCESS
    return BT::NodeStatus::SUCCESS;
}

}  // namespace patrol_bot
