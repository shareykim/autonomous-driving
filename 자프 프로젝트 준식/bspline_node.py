#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bspline_raceline_node.py
- 입력: /raw_path (nav_msgs/Path)
- 처리:
  1) Douglas–Peucker 로 점 줄이기
  2) Uniform cubic B-spline 으로 부드러운 레이스라인 생성
- 출력: /raceline_bspline (nav_msgs/Path)
- 저장: /home/misys/shared_dir/raceline_bspline.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv
import os
import math


class BSplineRacelineNode(Node):
    def __init__(self):
        super().__init__("bspline_raceline_node")
        self.sub = self.create_subscription(Path, "/raw_path", self.path_cb, 10)
        self.pub = self.create_publisher(Path, "/raceline_bspline", 10)

        self.output_csv = "/home/misys/shared_dir/raceline_bspline.csv"
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.eps = 0.02  # DP tolerance
        self.generated = False

    # ===== Douglas-Peucker =====
    def dp_simplify(self, points, eps):
        if len(points) < 3:
            return points

        start = points[0]
        end = points[-1]

        max_dist = -1.0
        index = -1

        sx, sy = start
        ex, ey = end
        vx = ex - sx
        vy = ey - sy
        v_len2 = vx * vx + vy * vy + 1e-9

        for i in range(1, len(points) - 1):
            px, py = points[i]
            # 거리: 점-선분
            t = ((px - sx) * vx + (py - sy) * vy) / v_len2
            t = max(0.0, min(1.0, t))
            projx = sx + t * vx
            projy = sy + t * vy
            dist = math.hypot(px - projx, py - projy)
            if dist > max_dist:
                max_dist = dist
                index = i

        if max_dist > eps:
            left = self.dp_simplify(points[: index + 1], eps)
            right = self.dp_simplify(points[index:], eps)
            return left[:-1] + right
        else:
            return [start, end]

    # ===== Uniform Cubic B-spline =====
    def bspline_uniform_cubic(self, ctrl_pts, num_per_seg=10):
        """
        ctrl_pts: N x 2
        return: M x 2, dense points
        """
        pts = []
        n = len(ctrl_pts)
        if n < 4:
            return ctrl_pts

        for i in range(1, n - 2):
            p0 = ctrl_pts[i - 1]
            p1 = ctrl_pts[i]
            p2 = ctrl_pts[i + 1]
            p3 = ctrl_pts[i + 2]

            for j in range(num_per_seg):
                u = j / float(num_per_seg)
                u2 = u * u
                u3 = u2 * u

                # basis (uniform cubic B-spline)
                b0 = (-u3 + 3 * u2 - 3 * u + 1) / 6.0
                b1 = (3 * u3 - 6 * u2 + 4) / 6.0
                b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.0
                b3 = u3 / 6.0

                x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
                y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
                pts.append((x, y))

        # 마지막 점 추가
        pts.append(tuple(ctrl_pts[-2]))
        pts.append(tuple(ctrl_pts[-1]))
        return pts

    def path_cb(self, msg: Path):
        if self.generated:
            return

        if len(msg.poses) < 4:
            self.get_logger().warn("Need at least 4 points for B-spline.")
            return

        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])
        raw_pts = list(zip(xs, ys))

        # 1) Douglas-Peucker
        simplified = self.dp_simplify(raw_pts, self.eps)
        self.get_logger().info(f"DP simplify: {len(raw_pts)} -> {len(simplified)} pts")

        # 2) B-spline 보간
        bspline_pts = self.bspline_uniform_cubic(simplified, num_per_seg=10)
        xs_new = [p[0] for p in bspline_pts]
        ys_new = [p[1] for p in bspline_pts]

        # Path publish
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
        self.get_logger().info(f"B-spline raceline generated. points={len(xs_new)}")
        self.generated = True

    def save_csv(self, xs, ys):
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            for x, y in zip(xs, ys):
                writer.writerow([float(x), float(y)])


def main(args=None):
    rclpy.init(args=args)
    node = BSplineRacelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
