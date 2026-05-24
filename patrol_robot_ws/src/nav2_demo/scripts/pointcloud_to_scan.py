#!/usr/bin/env python3
"""
===== 点云 → 激光扫描转换器 =====

Gazebo GPU Lidar 发布 sensor_msgs/PointCloud2，
Nav2 需要 sensor_msgs/LaserScan。

此节点从 pointcloud 中提取 z≈0 平面的 2D 切片，生成 LaserScan。
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_scan")

        self.declare_parameter("target_frame", "lidar_link")
        self.declare_parameter("min_height", -0.1)
        self.declare_parameter("max_height", 0.1)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(1.0))
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("scan_time", 0.1)

        self.sub = self.create_subscription(
            PointCloud2, "/scan_cloud", self.cloud_cb, 10
        )
        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        self.get_logger().info("PointCloudToScan started: /scan_cloud -> /scan")

    def cloud_cb(self, msg: PointCloud2):
        target = self.get_parameter("target_frame").value
        min_z = self.get_parameter("min_height").value
        max_z = self.get_parameter("max_height").value
        angle_min = self.get_parameter("angle_min").value
        angle_max = self.get_parameter("angle_max").value
        angle_inc = self.get_parameter("angle_increment").value
        range_min = self.get_parameter("range_min").value
        range_max = self.get_parameter("range_max").value
        scan_time = self.get_parameter("scan_time").value

        # 读取点云 (x, y, z, intensity)
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return

        # 过滤 z 高度
        filtered = [(p[0], p[1]) for p in points if min_z <= p[2] <= max_z]
        if not filtered:
            return

        xs = np.array([p[0] for p in filtered], dtype=np.float64)
        ys = np.array([p[1] for p in filtered], dtype=np.float64)
        dists = np.sqrt(xs * xs + ys * ys)
        angles = np.arctan2(ys, xs)

        num_bins = int((angle_max - angle_min) / angle_inc) + 1
        ranges = [float("inf")] * num_bins

        for i in range(len(dists)):
            bin_idx = int((angles[i] - angle_min) / angle_inc)
            if 0 <= bin_idx < num_bins:
                if dists[i] < ranges[bin_idx]:
                    ranges[bin_idx] = dists[i]

        # 构建 LaserScan
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = target
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = scan_time
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = [r if r < range_max else float("inf") for r in ranges]

        self.pub.publish(scan)


def main():
    rclpy.init()
    rclpy.spin(PointCloudToScan())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
