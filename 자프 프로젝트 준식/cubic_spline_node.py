#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
cubic_spline_raceline_node.py
- 입력: /raw_path (nav_msgs/Path)
- 출력: /raceline_cubic (nav_msgs/Path)
- 저장: /home/misys/shared_dir/raceline_cubic.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv
import os
import math


class CubicSplineRacelineNode(Node):
    def __init__(self):
        super().__init__("cubic_spline_raceline_node")
        self.sub = self.create_subscription(Path, "/raw_path", self.path_cb, 10)
        self.pub = self.create_publisher(Path, "/raceline_cubic", 10)

        self.output_csv = "/home/misys/shared_dir/raceline_cubic.csv"
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)

        self.generated = False

    # ====== Cubic Spline 1D 구현 ======
    def compute_natural_cubic_spline(self, t, y):
        n = len(t) - 1
        h = np.diff(t)
        alpha = np.zeros(n + 1)

        for i in range(1, n):
            alpha[i] = (3.0 / h[i]) * (y[i + 1] - y[i]) - (3.0 / h[i - 1]) * (y[i] - y[i - 1])

        l = np.ones(n + 1)
        mu = np.zeros(n + 1)
        z = np.zeros(n + 1)
        c = np.zeros(n + 1)
        b = np.zeros(n)
        d = np.zeros(n)

        for i in range(1, n):
            l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1]
            mu[i] = h[i] / l[i]
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i]

        for j in range(n - 1, -1, -1):
            c[j] = z[j] - mu[j] * c[j + 1]
            b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j])

        a = y[:-1]
        return a, b, c[:-1], d

    def eval_spline(self, t, a, b, c, d, t_new):
        # t: 원래 노드의 파라미터, t_new: 샘플링할 파라미터
        x_new = np.zeros_like(t_new)
        n = len(a)
        for idx, tn in enumerate(t_new):
            # tn 이 속하는 구간 찾기
            i = np.searchsorted(t, tn) - 1
            if i < 0:
                i = 0
            if i >= n:
                i = n - 1
            dt = tn - t[i]
            x_new[idx] = a[i] + b[i] * dt + c[i] * dt ** 2 + d[i] * dt ** 3
        return x_new

    # ====== 콜백 ======
    def path_cb(self, msg: Path):
        if self.generated:
            return

        if len(msg.poses) < 4:
            self.get_logger().warn("Need at least 4 points to generate cubic spline raceline.")
            return

        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])

        # 누적 거리로 파라미터 t 생성
        ds = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
        s = np.concatenate(([0.0], np.cumsum(ds)))
        total_length = s[-1]

        # 0.05 m 간격으로 샘플링
        step = 0.05
        num_samples = int(total_length / step) + 1
        s_new = np.linspace(0.0, total_length, num_samples)

        ax, bx, cx, dx = self.compute_natural_cubic_spline(s, xs)
        ay, by, cy, dy = self.compute_natural_cubic_spline(s, ys)

        xs_new = self.eval_spline(s, ax, bx, cx, dx, s_new)
        ys_new = self.eval_spline(s, ay, by, cy, dy, s_new)

        # Path 메시지 생성
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
        self.get_logger().info(f"Cubic spline raceline generated. points={len(xs_new)}")
        self.generated = True

    def save_csv(self, xs, ys):
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            for x, y in zip(xs, ys):
                writer.writerow([float(x), float(y)])


def main(args=None):
    rclpy.init(args=args)
    node = CubicSplineRacelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
