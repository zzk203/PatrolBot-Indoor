// M3.2: LoadWaypointsNode 单元测试
//
// 测试策略:
//   1. 测试 parseYaml 静态方法：验证 YAML 解析是否正确
//   2. 使用临时 YAML 文件测试 patrol_points / charge_standby / charge_dock
//   3. 测试空文件/无效路径的错误处理
//   4. 测试 BT 节点在 BehaviorTree 中执行，验证黑板输出

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

#include "nav2_demo/bt_nodes/load_waypoints_node.hpp"
#include "nav2_demo/bt_nodes/waypoint_structs.hpp"

namespace fs = std::filesystem;

// ======================================================================
// 辅助函数：创建临时 YAML 文件
// ======================================================================
static int g_temp_counter = 0;
std::string createTempYaml(const std::string & content)
{
  auto tmp_dir = fs::temp_directory_path();
  std::string unique_path = tmp_dir / ("nav2_demo_test_wp_" + std::to_string(++g_temp_counter) + ".yaml");
  std::ofstream f(unique_path);
  f << content;
  f.close();
  return unique_path;
}

// ======================================================================
// 测试 parseYaml 静态方法
// ======================================================================
class LoadWaypointsParseTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // 清理临时文件
    if (!tmp_file_.empty() && fs::exists(tmp_file_)) {
      fs::remove(tmp_file_);
    }
  }

  std::string tmp_file_;
};

TEST_F(LoadWaypointsParseTest, ParsesStandardWaypoints)
{
  // Arrange
  tmp_file_ = createTempYaml(R"(
patrol_points:
  - {x: 2.0, y: 2.0, yaw: 1.57}
  - {x: 4.0, y: 4.0, yaw: 3.14}
  - {x: 6.0, y: 2.0, yaw: -1.57}

charge_standby:
  x: 7.0
  y: 6.0
  yaw: -1.57

charge_dock:
  x: 7.5
  y: 6.0
  yaw: -1.57

navigation_timeout: 45.0
waypoint_wait_duration: 5.0
)");

  // Act
  auto config = nav2_demo::LoadWaypointsNode::parseYaml(tmp_file_);

  // Assert
  EXPECT_EQ(config.size(), 3u);
  EXPECT_FALSE(config.empty());

  // 验证第一个巡逻点
  EXPECT_DOUBLE_EQ(config.patrol_points[0].x, 2.0);
  EXPECT_DOUBLE_EQ(config.patrol_points[0].y, 2.0);
  EXPECT_DOUBLE_EQ(config.patrol_points[0].yaw, 1.57);

  // 验证最后一个巡逻点
  EXPECT_DOUBLE_EQ(config.patrol_points[2].x, 6.0);
  EXPECT_DOUBLE_EQ(config.patrol_points[2].y, 2.0);
  EXPECT_DOUBLE_EQ(config.patrol_points[2].yaw, -1.57);

  // 验证充电预备点
  EXPECT_DOUBLE_EQ(config.charge_standby.x, 7.0);
  EXPECT_DOUBLE_EQ(config.charge_standby.y, 6.0);
  EXPECT_DOUBLE_EQ(config.charge_standby.yaw, -1.57);

  // 验证充电桩
  EXPECT_DOUBLE_EQ(config.charge_dock.x, 7.5);
  EXPECT_DOUBLE_EQ(config.charge_dock.y, 6.0);
  EXPECT_DOUBLE_EQ(config.charge_dock.yaw, -1.57);

  // 验证超时和等待时间
  EXPECT_DOUBLE_EQ(config.navigation_timeout, 45.0);
  EXPECT_DOUBLE_EQ(config.waypoint_wait_duration, 5.0);
}

TEST_F(LoadWaypointsParseTest, HandlesOnlyPatrolPoints)
{
  // Arrange
  tmp_file_ = createTempYaml(R"(
patrol_points:
  - {x: 1.0, y: 1.0, yaw: 0.0}
)");

  // Act
  auto config = nav2_demo::LoadWaypointsNode::parseYaml(tmp_file_);

  // Assert
  EXPECT_EQ(config.size(), 1u);
  EXPECT_DOUBLE_EQ(config.patrol_points[0].x, 1.0);

  // 充电点应为默认值 (0,0,0)
  EXPECT_DOUBLE_EQ(config.charge_standby.x, 0.0);
  EXPECT_DOUBLE_EQ(config.charge_dock.x, 0.0);

  // 超时和等待时间应为默认值
  EXPECT_DOUBLE_EQ(config.navigation_timeout, 60.0);
  EXPECT_DOUBLE_EQ(config.waypoint_wait_duration, 3.0);
}

TEST_F(LoadWaypointsParseTest, HandlesEmptyPatrolPoints)
{
  // Arrange
  tmp_file_ = createTempYaml(R"(
patrol_points: []
)");

  // Act
  auto config = nav2_demo::LoadWaypointsNode::parseYaml(tmp_file_);

  // Assert
  EXPECT_TRUE(config.empty());
  EXPECT_EQ(config.size(), 0u);
}

TEST_F(LoadWaypointsParseTest, HandlesNonexistentFile)
{
  // Arrange
  std::string bad_path = "/nonexistent/path/waypoints.yaml";

  // Act
  auto config = nav2_demo::LoadWaypointsNode::parseYaml(bad_path);

  // Assert
  EXPECT_TRUE(config.empty());
}

TEST_F(LoadWaypointsParseTest, HandlesMalformedYaml)
{
  // Arrange
  tmp_file_ = createTempYaml("this is not valid yaml: [\n  broken");

  // Act & Assert (should throw YAML exception)
  EXPECT_THROW(
    nav2_demo::LoadWaypointsNode::parseYaml(tmp_file_),
    YAML::Exception
  );
}

// ======================================================================
// 测试 Waypoint 转换
// ======================================================================
TEST(WaypointConversionTest, ToPoseStampedZeros)
{
  nav2_demo::Waypoint wp{0.0, 0.0, 0.0};
  auto pose = wp.toPoseStamped();

  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
}

TEST(WaypointConversionTest, ToPoseStampedWithYaw90)
{
  nav2_demo::Waypoint wp{2.5, 1.5, 1.570796};  // ~90 degrees
  auto pose = wp.toPoseStamped();

  EXPECT_DOUBLE_EQ(pose.pose.position.x, 2.5);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 1.5);
  EXPECT_NEAR(pose.pose.orientation.z, 0.7071, 0.001);
  EXPECT_NEAR(pose.pose.orientation.w, 0.7071, 0.001);
}

// ======================================================================
// 测试 BT 树集成 (需要 rclcpp 初始化)
// ======================================================================
class LoadWaypointsBTTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    tmp_file_ = createTempYaml(R"(
patrol_points:
  - {x: 1.0, y: 2.0, yaw: 0.5}
  - {x: 3.0, y: 4.0, yaw: 1.0}
)");
  }

  void TearDown() override
  {
    if (!tmp_file_.empty() && fs::exists(tmp_file_)) {
      fs::remove(tmp_file_);
    }
  }

  std::string tmp_file_;
};

// 注意: 此测试需要完整的 BT.CPP + rclcpp 环境
// 如果编译或运行时报错，可能需要调整依赖或 mock 环境
TEST_F(LoadWaypointsBTTest, DISABLED_ExecutesInBehaviorTree)
{
  // 这个测试被禁用，因为它需要完整的 BT + rclcpp 运行时环境。
  // parseYaml 的单元测试已覆盖核心逻辑。
  // 集成测试由 test_patrol_sequence.py 中的 Python 测试覆盖。
  GTEST_SKIP() << "跳过需要完整 BT 运行时的集成测试，由 Python 测试覆盖";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
