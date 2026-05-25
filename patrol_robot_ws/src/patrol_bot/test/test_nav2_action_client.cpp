#include <gtest/gtest.h>
#include "patrol_bot/nav2_action_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

class Nav2ActionClientTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("test_nav2_client");
        client_ = std::make_unique<patrol_bot::Nav2ActionClient>(node_.get());
    }

    void TearDown() override {
        client_.reset();
        node_.reset();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<patrol_bot::Nav2ActionClient> client_;
};

// ==================== 测试用例 ====================

// 测试 1: 构造后状态 - is_navigating() == false, check_result() == NOT_STARTED
TEST_F(Nav2ActionClientTest, InitialState) {
    EXPECT_FALSE(client_->is_navigating());
    EXPECT_EQ(client_->check_result(), patrol_bot::NavResult::NOT_STARTED);
}

// 测试 2: send_goal 时服务器未就绪 - 应返回 ERROR
TEST_F(Nav2ActionClientTest, SendGoalWithoutServer) {
    client_->send_goal(1.0, 2.0, 0.5);
    EXPECT_EQ(client_->check_result(), patrol_bot::NavResult::ERROR);
}

// 测试 3: cancel_goal 重置状态 - cancel 后回到 NOT_STARTED
TEST_F(Nav2ActionClientTest, CancelGoalResetsState) {
    // 先触发 ERROR（服务器未就绪时 send_goal）
    client_->send_goal(1.0, 2.0, 0.5);
    EXPECT_EQ(client_->check_result(), patrol_bot::NavResult::ERROR);

    // cancel 应重置所有状态
    client_->cancel_goal();
    EXPECT_EQ(client_->check_result(), patrol_bot::NavResult::NOT_STARTED);
    EXPECT_FALSE(client_->is_navigating());
}

// 测试 4: yaw -> quaternion 转换 - 验证各角度转换正确
TEST(Nav2ConversionTest, YawToQuaternion) {
    // 0 弧度 → 单位四元数
    auto q0 = patrol_bot::yaw_to_quaternion_msg(0.0);
    EXPECT_NEAR(q0.x, 0.0, 1e-6);
    EXPECT_NEAR(q0.y, 0.0, 1e-6);
    EXPECT_NEAR(q0.z, 0.0, 1e-6);
    EXPECT_NEAR(q0.w, 1.0, 1e-6);

    // π/2 弧度 → 绕 Z 轴 90 度
    auto q90 = patrol_bot::yaw_to_quaternion_msg(M_PI / 2.0);
    EXPECT_NEAR(q90.x, 0.0, 1e-6);
    EXPECT_NEAR(q90.y, 0.0, 1e-6);
    EXPECT_NEAR(q90.z, std::sin(M_PI / 4.0), 1e-6);
    EXPECT_NEAR(q90.w, std::cos(M_PI / 4.0), 1e-6);

    // π 弧度 → 绕 Z 轴 180 度
    auto q180 = patrol_bot::yaw_to_quaternion_msg(M_PI);
    EXPECT_NEAR(q180.x, 0.0, 1e-6);
    EXPECT_NEAR(q180.y, 0.0, 1e-6);
    EXPECT_NEAR(q180.z, 1.0, 1e-6);
    EXPECT_NEAR(q180.w, 0.0, 1e-6);

    // -π/2 弧度 → 绕 Z 轴 -90 度
    auto qn90 = patrol_bot::yaw_to_quaternion_msg(-M_PI / 2.0);
    EXPECT_NEAR(qn90.x, 0.0, 1e-6);
    EXPECT_NEAR(qn90.y, 0.0, 1e-6);
    EXPECT_NEAR(qn90.z, std::sin(-M_PI / 4.0), 1e-6);
    EXPECT_NEAR(qn90.w, std::cos(-M_PI / 4.0), 1e-6);

    // 任意角度 0.723 弧度
    double yaw = 0.723;
    auto q = patrol_bot::yaw_to_quaternion_msg(yaw);
    EXPECT_NEAR(q.x, 0.0, 1e-6);
    EXPECT_NEAR(q.y, 0.0, 1e-6);
    EXPECT_NEAR(q.z, std::sin(yaw / 2.0), 1e-6);
    EXPECT_NEAR(q.w, std::cos(yaw / 2.0), 1e-6);
}

// 测试 5: wait_for_server 超时 - 没有 server 时超时返回 false
TEST_F(Nav2ActionClientTest, WaitForServerTimeout) {
    auto result = client_->wait_for_server(std::chrono::seconds(1));
    EXPECT_FALSE(result);
    // 超时后 server_ready_ 为 false, is_navigating 也应保持 false
    EXPECT_FALSE(client_->is_navigating());
    // 此时 send_goal 应返回 ERROR
    client_->send_goal(0.0, 0.0, 0.0);
    EXPECT_EQ(client_->check_result(), patrol_bot::NavResult::ERROR);
}

// 主函数
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
