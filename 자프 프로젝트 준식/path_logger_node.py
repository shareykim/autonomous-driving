#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
path_logger_node.py
- LiDAR + SLAM → TF(map→base_link)에서 포즈를 받아서 raw_path를 만든다.
- nav_msgs/Path 로 /raw_path 토픽에 publish
- 동시에 CSV로 /home/misys/shared_dir/raw_path.csv 에 저장
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import csv
import os
import math
from rclpy.time import Time

class PathLoggerNode(Node):
    def __init__(self):
        super().__init__("path_logger_node")
        self.path_pub = self.create_publisher(Path, "/raw_path", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.points = []  # (x, y, yaw)

        # 최소 이동 거리(노이즈 방지)
        self.min_dist = 0.05
        self.last_x = None
        self.last_y = None

        self.csv_path = "/home/misys/shared_dir/raw_path.csv"
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

    def timer_cb(self):
        # TF: map → base_link
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                "map", "base_link", now, rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().debug(f"TF error: {e}")
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_dist:
                return

        self.last_x = x
        self.last_y = y

        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation = q

        self.path_msg.header.stamp = ps.header.stamp
        self.path_msg.poses.append(ps)
        self.points.append((x, y, yaw))

        self.path_pub.publish(self.path_msg)

    def destroy_node(self):
        # 종료 시 CSV 저장
        self.get_logger().info(f"Saving raw_path to {self.csv_path}")
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "yaw"])
            for x, y, yaw in self.points:
                writer.writerow([x, y, yaw])
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
