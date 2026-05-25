#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <string>

namespace patrol_bot {

class CameraBuffer {
public:
    CameraBuffer(rclcpp::Node* node,
                 const std::string& topic,
                 const std::string& save_dir,
                 const std::string& format);

    /// @brief 保存最新帧到文件
    /// @param filename 输出文件名（不含路径和扩展名）
    /// @return 成功返回 true，无可用帧返回 false
    bool save_latest(const std::string& filename);

    /// @brief 是否有可用帧
    bool has_frame() const;

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    std::string save_dir_;
    std::string format_;

    cv::Mat latest_frame_;
    bool has_frame_ = false;
    mutable std::mutex mutex_;
};

}  // namespace patrol_bot
