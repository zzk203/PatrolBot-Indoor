#!/usr/bin/env python3
"""
===== 静态地图发布器 =====

绕过 nav2_map_server 的 lifecycle 管理问题，
直接从 PGM 文件加载地图并发布到 /map 话题。
使用 TRANSIENT_LOCAL + RELIABLE QoS，与 RViz2 默认订阅配置匹配。
"""

import os
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__("static_map_publisher")

        self.declare_parameter("yaml_filename", "")

        yaml_path = self.get_parameter("yaml_filename").value
        if not yaml_path:
            self.get_logger().error("未指定 yaml_filename 参数")
            return

        yaml_dir = os.path.dirname(yaml_path)
        with open(yaml_path, "r") as f:
            meta = yaml.safe_load(f)

        image_path = os.path.join(yaml_dir, meta["image"])
        resolution = meta["resolution"]
        origin = meta["origin"]
        occupied_thresh = meta.get("occupied_thresh", 0.65)
        free_thresh = meta.get("free_thresh", 0.25)
        negate = meta.get("negate", 0)
        mode = meta.get("mode", "trinary")

        self.get_logger().info(f"加载地图: {image_path} ({meta['image']})")

        # 读取 PGM
        with open(image_path, "rb") as f:
            header = f.readline().strip()  # P5
            width_h = f.readline().strip()
            while width_h.startswith(b"#"):
                width_h = f.readline().strip()
            parts = width_h.split()
            width = int(parts[0])
            height = int(parts[1])
            maxval = int(f.readline().strip())
            data = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))

        # 坐标翻转: PGM 左上角是原点，需要按 y 轴翻转
        # 用 PIL 替代手工翻转，但减少依赖
        data = np.flipud(data)

        # 转换为 OccupancyGrid
        if negate:
            data = 255 - data

        occ = np.full((height, width), -1, dtype=np.int8)
        if mode == "trinary":
            occ[data < 0.5 * 255] = 0  # 白色 = 空闲
            occ[data > occupied_thresh * 255] = 100  # 黑色 = 障碍
        elif mode == "scale":
            frac = data.astype(float) / 255.0
            occ[(frac >= 0) & (frac <= free_thresh)] = 0
            occ[(frac >= occupied_thresh) & (frac <= 1.0)] = 100
            occ[(frac > free_thresh) & (frac < occupied_thresh)] = (1.0 - frac[(
                frac > free_thresh) & (frac < occupied_thresh)]) * 100.0
            occ = np.clip(occ, 0, 100).astype(np.int8)

        # 发布（TRANSIENT_LOCAL + RELIABLE，与 costmap static_layer 的订阅匹配）
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(OccupancyGrid, "/map", qos)
        self.timer = self.create_timer(5.0, self.publish_map)

        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "map"
        self.grid.info = MapMetaData()
        self.grid.info.resolution = resolution
        self.grid.info.width = width
        self.grid.info.height = height
        self.grid.info.origin = Pose()
        self.grid.info.origin.position.x = origin[0]
        self.grid.info.origin.position.y = origin[1]
        self.grid.info.origin.orientation.w = 1.0

        # 展平为一维数组，顺序: row-major (y increasing)
        self.grid.data = occ.flatten().tolist()

        self.get_logger().info(
            f"地图就绪: {width}x{height} @ {resolution}m/px, "
            f"origin=({origin[0]}, {origin[1]})"
        )

    def publish_map(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)
        self.get_logger().info("发布 /map")


def main():
    rclpy.init()
    node = StaticMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
