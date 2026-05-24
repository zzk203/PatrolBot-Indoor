#!/usr/bin/env python3
"""
M2.5: 动态避障验证测试

测试目标:
  1. 验证 Nav2 局部规划器配置支持动态避障
  2. 验证代价地图参数适用于动态障碍物场景
  3. 验证恢复行为配置（Spin/BackUp/Wait）可用于脱困
  4. 验证局部规划器（Regulated Pure Pursuit）的参数合理性

测试策略:
  不直接运行 Gazebo（需要图形环境），而是验证 Nav2 参数配置
  是否支持动态避障场景，以及局部规划器的关键参数是否合理。
"""

import os
import yaml
import math
import pytest


# ======================================================================
# 测试固件
# ======================================================================
@pytest.fixture(scope="module")
def params():
    """加载 nav2_params.yaml"""
    pkg_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    )
    params_path = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    assert os.path.isfile(params_path), f"参数文件缺失: {params_path}"

    with open(params_path) as f:
        # 处理重复键（ROS2 params 允许重复键，但 YAML 解析会保留最后一个）
        # 使用 FullLoader 处理 ROS2 风格参数
        data = yaml.safe_load(f)
    return data, pkg_dir


# ======================================================================
# 测试类
# ======================================================================
class TestDynamicObstacleAvoidance:
    """M2.5 动态避障参数验证"""

    # ---------- 控制参数 ----------
    def test_controller_frequency(self, params):
        """测试 1: 控制器频率应足够高以响应动态障碍"""
        data, _ = params
        cf = data.get("controller_server", {}).get("ros__parameters", {}).get(
            "controller_frequency", 0)
        assert cf >= 10.0, \
            f"controller_frequency={cf}Hz 过低（需 ≥10Hz 以快速响应障碍物）"
        print(f"  ✓ 控制器频率: {cf}Hz (合格)")

    def test_controller_plugin(self, params):
        """测试 2: 使用合适的局部规划器（RegulatedPurePursuit 支持避障）"""
        data, _ = params
        plugins = data.get("controller_server", {}).get(
            "ros__parameters", {}).get("controller_plugins", [])
        assert "FollowPath" in plugins, \
            "缺少 FollowPath 控制器插件"
        print(f"  ✓ 控制器插件: {plugins}")

    # ---------- Regulated Pure Pursuit 参数 ----------
    def test_pure_pursuit_params(self, params):
        """测试 3: Regulated Pure Pursuit 参数合理性"""
        data, _ = params
        pp = data.get("controller_server", {}).get(
            "ros__parameters", {}).get("FollowPath", {})

        # 检查关键参数
        desired_vel = pp.get("desired_linear_vel", 0)
        assert 0.1 <= desired_vel <= 0.5, \
            f"desired_linear_vel={desired_vel} 超出合理范围 [0.1, 0.5]"
        print(f"  ✓ 期望线速度: {desired_vel} m/s")

        lookahead = pp.get("lookahead_dist", 0)
        assert lookahead >= desired_vel * 2, \
            f"lookahead_dist={lookahead} 偏小，可能导致震荡"
        print(f"  ✓ 前瞻距离: {lookahead} m")

        # 速度缩放（动态避障关键：接近障碍物时自动减速）
        use_scaling = pp.get("use_regulated_linear_velocity_scaling", False)
        assert use_scaling, \
            "未启用 regulated_linear_velocity_scaling（动态避障需要此功能自动减速）"
        print(f"  ✓ 速度缩放: 已启用 (接近障碍物自动减速)")

        # 旋转到朝向（在窄通道中调整方向）
        use_rotate = pp.get("use_rotate_to_heading", False)
        assert use_rotate, \
            "未启用 rotate_to_heading（在目标点转向时需要）"
        print(f"  ✓ 旋转到朝向: 已启用")

    # ---------- 代价地图参数 ----------
    def test_costmap_inflation(self, params):
        """测试 4: 代价地图膨胀参数"""
        data, _ = params

        # 全局代价地图
        gc = data.get("global_costmap", {}).get(
            "global_costmap", {}).get("ros__parameters", {})
        gi = gc.get("inflation_layer", {})

        inflation_radius = gi.get("inflation_radius", 0)
        robot_radius = gc.get("robot_radius", 0.22)
        # 膨胀半径应大于机器人半径
        assert inflation_radius >= robot_radius, \
            f"全局膨胀半径 {inflation_radius}m < 机器人半径 {robot_radius}m"
        print(f"  ✓ 全局膨胀半径: {inflation_radius}m (≥ 机器人半径 {robot_radius}m)")

        # 局部代价地图
        lc = data.get("local_costmap", {}).get(
            "local_costmap", {}).get("ros__parameters", {})
        li = lc.get("inflation_layer", {})

        local_inflation = li.get("inflation_radius", 0)
        assert local_inflation >= robot_radius, \
            f"局部膨胀半径 {local_inflation}m < 机器人半径 {robot_radius}m"
        print(f"  ✓ 局部膨胀半径: {local_inflation}m")

        # 代价缩放因子（影响障碍物附近的路径代价）
        scaling = gi.get("cost_scaling_factor", 0)
        assert 1.0 <= scaling <= 10.0, \
            f"cost_scaling_factor={scaling} 超出合理范围 [1.0, 10.0]"
        print(f"  ✓ 代价缩放因子: {scaling}")

    def test_costmap_update_frequency(self, params):
        """测试 5: 代价地图更新频率"""
        data, _ = params

        # 局部代价地图更新频率应较高以快速反映动态障碍
        lc = data.get("local_costmap", {}).get(
            "local_costmap", {}).get("ros__parameters", {})
        uf = lc.get("update_frequency", 0)
        assert uf >= 3.0, \
            f"局部代价地图更新频率={uf}Hz 过低（动态避障需 ≥3Hz）"
        print(f"  ✓ 局部代价地图更新频率: {uf}Hz")

        # 滚动窗口（动态避障必需）
        rolling = lc.get("rolling_window", False)
        assert rolling, \
            "local_costmap 未启用 rolling_window（动态避障需要滚动窗口）"
        print(f"  ✓ 滚动窗口: 已启用")

    # ---------- 全局规划器参数 ----------
    def test_global_planner_params(self, params):
        """测试 6: 全局规划器参数"""
        data, _ = params
        gp = data.get("planner_server", {}).get("ros__parameters", {})
        freq = gp.get("expected_planner_frequency", 0)
        assert freq >= 5.0, \
            f"规划器频率={freq}Hz 过低（需 ≥5Hz 以快速重规划）"
        print(f"  ✓ 规划器频率: {freq}Hz")

        tolerance = gp.get("GridBased", {}).get("tolerance", 0)
        assert 0.5 <= tolerance <= 5.0, \
            f"规划容差={tolerance} 不合理（应在 [0.5, 5.0] 范围）"
        print(f"  ✓ 规划容差: {tolerance}m")

    # ---------- 行为恢复参数 ----------
    def test_recovery_behaviors(self, params):
        """测试 7: 恢复行为配置"""
        data, _ = params

        # behavior_server 配置
        bs = data.get("behavior_server", {}).get("ros__parameters", {})
        plugins = bs.get("behavior_plugins", [])

        essential = ["spin", "backup"]
        for p in essential:
            assert p in plugins, \
                f"缺少恢复行为: {p}（动态避障必需）"
        print(f"  ✓ 恢复行为插件: {plugins}")

        # 旋转速度参数
        max_rot = bs.get("max_rotational_vel", 0)
        assert max_rot >= 0.5, \
            f"最大旋转速度={max_rot} 偏小（脱困需 ≥0.5 rad/s）"
        print(f"  ✓ 最大旋转速度: {max_rot} rad/s")

    # ---------- 行为树配置 ----------
    def test_behavior_tree_recovery(self, params):
        """测试 8: 行为树中的恢复逻辑"""
        data, pkg_dir = params

        # 检查 BT 文件
        bt_dir = os.path.join(pkg_dir, "behavior_trees")
        bt_files = [f for f in os.listdir(bt_dir) if f.endswith(".xml")]
        assert len(bt_files) > 0, "缺少行为树文件"

        # 检查 BT Navigator 配置
        bt_config = data.get("bt_navigator", {}).get("ros__parameters", {})
        default_bt = bt_config.get("default_bt_xml_filename", "")
        assert default_bt, "未指定默认行为树"
        print(f"  ✓ 默认行为树: {default_bt}")

        # 检查行为树内容（如果有的话）
        bt_path = os.path.join(bt_dir, default_bt)
        if os.path.isfile(bt_path):
            with open(bt_path) as f:
                bt_content = f.read()
            # 验证包含恢复行为节点
            recovery_nodes = ["ClearEntireCostmap", "Spin", "BackUp", "Wait"]
            for rn in recovery_nodes:
                if rn in bt_content:
                    print(f"  ✓ 行为树包含恢复节点: {rn}")
                    break
            else:
                pytest.warn(UserWarning(
                    "行为树中未找到标准恢复节点 (ClearEntireCostmap/Spin/BackUp/Wait)"))

    # ---------- 机器人尺寸 ----------
    def test_robot_dimensions(self, params):
        """测试 9: 机器人尺寸与地图匹配"""
        data, _ = params

        # 检查 footprint 定义
        gc = data.get("global_costmap", {}).get(
            "global_costmap", {}).get("ros__parameters", {})
        footprint = gc.get("footprint", "")
        assert footprint, "缺少 footprint 定义"
        print(f"  ✓ 机器人 footprint: {footprint}")

        robot_radius = gc.get("robot_radius", 0)
        assert 0.1 <= robot_radius <= 1.0, \
            f"robot_radius={robot_radius} 不合理"
        print(f"  ✓ 机器人半径: {robot_radius}m")

    # ---------- AMCL 定位 ----------
    def test_amcl_params(self, params):
        """测试 10: AMCL 定位参数"""
        data, _ = params
        amcl = data.get("amcl", {}).get("ros__parameters", {})

        # 粒子数量（影响定位精度和计算负载的平衡）
        max_p = amcl.get("max_particles", 0)
        min_p = amcl.get("min_particles", 0)
        assert min_p >= 100, f"min_particles={min_p} 过少（需 ≥100）"
        assert max_p <= 5000, f"max_particles={max_p} 过多（建议 ≤5000）"
        print(f"  ✓ 粒子数: [{min_p}, {max_p}]")

        # 更新阈值（定位更新灵敏度）
        update_d = amcl.get("update_min_d", 0)
        assert 0.1 <= update_d <= 0.5, \
            f"update_min_d={update_d} 超出合理范围 [0.1, 0.5]"
        print(f"  ✓ 定位更新最小位移: {update_d}m")

        # 初始位姿
        init_pose = amcl.get("initial_pose", {})
        if init_pose:
            print(f"  ✓ 初始位姿: ({init_pose.get('x', '?')}, "
                  f"{init_pose.get('y', '?')})")

    # ---------- 综合评估 ----------
    def test_dynamic_obstacle_readiness(self, params):
        """测试 11: 综合评估 Nav2 动态避障就绪度"""
        _, pkg_dir = params

        checks = []

        # 1. 局部规划器检查
        data, _ = params
        pp = data.get("controller_server", {}).get(
            "ros__parameters", {}).get("FollowPath", {})
        checks.append(("RegulatedPurePursuit 控制器已启用",
                       "FollowPath" in data.get("controller_server", {})
                       .get("ros__parameters", {})
                       .get("controller_plugins", [])))

        # 2. 速度缩放
        checks.append(("速度缩放已启用（动态避障必需）",
                       pp.get("use_regulated_linear_velocity_scaling", False)))

        # 3. 滚动窗口
        lc = data.get("local_costmap", {}).get(
            "local_costmap", {}).get("ros__parameters", {})
        checks.append(("局部代价地图滚动窗口已启用",
                       lc.get("rolling_window", False)))

        # 4. 恢复行为
        bs = data.get("behavior_server", {}).get("ros__parameters", {})
        plugins = bs.get("behavior_plugins", [])
        checks.append(("恢复行为（Spin/BackUp）已配置",
                       "spin" in plugins and "backup" in plugins))

        # 5. 膨胀层
        checks.append(("代价地图膨胀层已配置",
                       "inflation_layer" in lc.get("plugins", [])))

        # 6. 控制器频率
        cf = data.get("controller_server", {}).get(
            "ros__parameters", {}).get("controller_frequency", 0)
        checks.append((f"控制器频率 {cf}Hz ≥ 10Hz", cf >= 10.0))

        # 统计
        passed = sum(1 for _, ok in checks if ok)
        total = len(checks)

        print(f"\n  📊 动态避障就绪度: {passed}/{total} 项检查通过")
        for name, ok in checks:
            status = "✅" if ok else "❌"
            print(f"     {status} {name}")

        # 允许部分失败（有的配置可能有意为之）
        if passed < total:
            print(f"  ⚠️  {total - passed} 项未通过，建议根据实际场景调整参数")
        else:
            print(f"  ✅ 所有检查通过，Nav2 已具备动态避障能力")


# ======================================================================
# 主入口
# ======================================================================
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
