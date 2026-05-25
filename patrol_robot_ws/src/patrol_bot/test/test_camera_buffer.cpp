#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "patrol_bot/camera_buffer.hpp"
#include <filesystem>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

class CameraBufferTest : public ::testing::Test {
protected:
    std::string tmp_dir;
    std::string topic;
    rclcpp::Node::SharedPtr node;
    patrol_bot::CameraBuffer* camera_buffer;

    void SetUp() override {
        rclcpp::init(0, nullptr);

        tmp_dir = (fs::temp_directory_path() / "patrol_test_camera").string();
        fs::remove_all(tmp_dir);

        topic = "/test_camera_image";
        node = std::make_shared<rclcpp::Node>("test_camera_node");
        camera_buffer = new patrol_bot::CameraBuffer(node.get(), topic, tmp_dir, "png");
    }

    void TearDown() override {
        delete camera_buffer;
        node.reset();
        rclcpp::shutdown();
        fs::remove_all(tmp_dir);
    }

    /// @brief 向 topic 发布一幅测试图像
    void publish_test_image() {
        auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, 10);
        cv::Mat img(100, 100, CV_8UC3, cv::Scalar(0, 255, 0));
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        pub->publish(*msg);

        // 等待回调处理
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            rclcpp::spin_some(node);
            if (camera_buffer->has_frame()) {
                break;
            }
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 2000) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

// ============================================================
// 1. 无帧时保存失败 — 构造后直接 save_latest 返回 false
// ============================================================
TEST_F(CameraBufferTest, SaveLatestWithoutFrame) {
    bool result = camera_buffer->save_latest("test_image");
    EXPECT_FALSE(result);
}

// ============================================================
// 2. 无帧时 has_frame 返回 false — 构造后 has_frame() == false
// ============================================================
TEST_F(CameraBufferTest, HasFrameInitiallyFalse) {
    EXPECT_FALSE(camera_buffer->has_frame());
}

// ============================================================
// 3. 发布图像后 has_frame 为 true
// ============================================================
TEST_F(CameraBufferTest, HasFrameAfterPublish) {
    publish_test_image();
    EXPECT_TRUE(camera_buffer->has_frame());
}

// ============================================================
// 4. 发布图像后保存成功 — save_latest 返回 true，且文件存在
// ============================================================
TEST_F(CameraBufferTest, SaveAfterPublish) {
    publish_test_image();

    bool result = camera_buffer->save_latest("captured_frame");
    EXPECT_TRUE(result);

    std::string expected_path = tmp_dir + "/captured_frame.png";
    EXPECT_TRUE(fs::exists(expected_path)) << "File should exist: " << expected_path;
    EXPECT_GT(fs::file_size(expected_path), 0u) << "File should not be empty";
}

// ============================================================
// 5. 保存目录自动创建 — 传入不存在的目录，验证目录被创建
// ============================================================
TEST_F(CameraBufferTest, AutoCreateDirectory) {
    // 使用新的不存在的目录
    std::string new_dir = tmp_dir + "/subdir/nested";
    ASSERT_FALSE(fs::exists(new_dir)) << "Directory should not exist before test";

    // 构造 CameraBuffer 时应自动创建目录
    patrol_bot::CameraBuffer cb(node.get(), topic, new_dir, "jpg");
    EXPECT_TRUE(fs::exists(new_dir)) << "Directory should be auto-created";
}
