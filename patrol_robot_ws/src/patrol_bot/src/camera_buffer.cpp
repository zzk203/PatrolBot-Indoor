#include "patrol_bot/camera_buffer.hpp"
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <string>

namespace patrol_bot {

CameraBuffer::CameraBuffer(rclcpp::Node* node,
                           const std::string& topic,
                           const std::string& save_dir,
                           const std::string& format)
    : save_dir_(save_dir)
    , format_(format)
{
    // 自动创建保存目录
    std::filesystem::create_directories(save_dir_);

    // 订阅图像 topic
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        std::bind(&CameraBuffer::image_callback, this, std::placeholders::_1));
}

void CameraBuffer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_ = frame.clone();
        has_frame_ = true;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("CameraBuffer"),
                     "cv_bridge conversion failed: %s", e.what());
    }
}

bool CameraBuffer::has_frame() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_frame_;
}

bool CameraBuffer::save_latest(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_frame_) {
        return false;
    }

    std::string filepath = save_dir_ + "/" + filename + "." + format_;
    return cv::imwrite(filepath, latest_frame_);
}

}  // namespace patrol_bot
