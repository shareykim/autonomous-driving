#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
catmullrom_raceline_node.py
- 입력: /raw_path (nav_msgs/Path)
- 출력: /raceline_catmullrom (nav_msgs/Path)
- 저장: /home/misys/shared_dir/raceline_catmullrom.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv
import os


class CatmullRomRacelineNode(Node):
    def __init__(self):
        super().__init__("catmullrom_raceline_node")
        self.sub = self.create_subscription(Path, "/raw_path", self.path_cb, 10)
        self.pub = self.create_publisher(Path, "/raceline_catmullrom", 10)

        self.output_csv = "/home/misys/shared_dir/raceline_catmullrom.csv"
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.generated = False

    def catmull_rom(self, pts, num_per_seg=10):
        """
        pts: N x 2 (control points)
        반환: dense points
        """
        n = len(pts)
        if n < 4:
            return pts

        out = []
        for i in range(1, n - 2):
            p0 = np.array(pts[i - 1])
            p1 = np.array(pts[i])
            p2 = np.array(pts[i + 1])
            p3 = np.array(pts[i + 2])

            for j in range(num_per_seg):
                u = j / float(num_per_seg)
                u2 = u * u
                u3 = u2 * u

                # Catmull-Rom basis
                a = 2 * p1
                b = -p0 + p2
                c = 2 * p0 - 5 * p1 + 4 * p2 - p3
                d = -p0 + 3 * p1 - 3 * p2 + p3

                p = 0.5 * (a + b * u + c * u2 + d * u3)
                out.append((float(p[0]), float(p[1])))

        # 마지막 점들 추가
        out.append(tuple(pts[-2]))
        out.append(tuple(pts[-1]))
        return out

    def path_cb(self, msg: Path):
        if self.generated:
            return

        if len(msg.poses) < 4:
            self.get_logger().warn("Need at least 4 points for Catmull-Rom.")
            return

        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        pts = list(zip(xs, ys))

        cr_pts = self.catmull_rom(pts, num_per_seg=10)
        xs_new = [p[0] for p in cr_pts]
        ys_new = [p[1] for p in cr_pts]

        raceline = Path()
        raceline.header.frame_id = msg.header.frame_id
        raceline.header.stamp = msg.header.stamp

        for x, y in zip(xs_new, ys_new):
            ps = PoseStamped()
            ps.header = raceline.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            raceline.poses.append(ps)

        self.pub.publish(raceline)
        self.save_csv(xs_new, ys_new)
        self.get_logger().info(f"Catmull-Rom raceline generated. points={len(xs_new)}")
        self.generated = True

    def save_csv(self, xs, ys):
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            for x, y in zip(xs, ys):
                writer.writerow([float(x), float(y)])


def main(args=None):
    rclpy.init(args=args)
    node = CatmullRomRacelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
