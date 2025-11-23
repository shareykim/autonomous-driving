#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
clothoid_raceline_node.py
- 입력: /raw_path (nav_msgs/Path)
- 처리:
  1) 원래 path에서 heading, curvature 대략 추정
  2) 곡률이 각 segment에서 선형적으로 변한다고 가정하고 세밀히 적분 → clothoid-like path
- 출력: /raceline_clothoid (nav_msgs/Path)
- 저장: /home/misys/shared_dir/raceline_clothoid.csv
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv
import os
import math


class ClothoidRacelineNode(Node):
    def __init__(self):
        super().__init__("clothoid_raceline_node")
        self.sub = self.create_subscription(Path, "/raw_path", self.path_cb, 10)
        self.pub = self.create_publisher(Path, "/raceline_clothoid", 10)

        self.output_csv = "/home/misys/shared_dir/raceline_clothoid.csv"
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.generated = False

    def estimate_heading(self, xs, ys):
        headings = []
        for i in range(len(xs)):
            if i == 0:
                dx = xs[1] - xs[0]
                dy = ys[1] - ys[0]
            elif i == len(xs) - 1:
                dx = xs[-1] - xs[-2]
                dy = ys[-1] - ys[-2]
            else:
                dx = xs[i + 1] - xs[i - 1]
                dy = ys[i + 1] - ys[i - 1]
            headings.append(math.atan2(dy, dx))
        return np.unwrap(np.array(headings))

    def estimate_curvature(self, xs, ys, headings):
        curv = [0.0]
        for i in range(1, len(xs)):
            dx = xs[i] - xs[i - 1]
            dy = ys[i] - ys[i - 1]
            ds = max(math.hypot(dx, dy), 1e-6)
            dtheta = headings[i] - headings[i - 1]
            curv.append(dtheta / ds)
        return np.array(curv)

    def generate_clothoid_like(self, xs, ys, step=0.05):
        xs = np.array(xs)
        ys = np.array(ys)
        # arc-length 파라미터
        ds = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
        s = np.concatenate(([0.0], np.cumsum(ds)))
        total_len = s[-1]

        headings = self.estimate_heading(xs, ys)
        curv = self.estimate_curvature(xs, ys, headings)

        # s 를 일정 step으로 샘플링
        num_samples = int(total_len / step) + 1
        s_new = np.linspace(0.0, total_len, num_samples)

        # 곡률과 heading을 s에 대해 선형보간
        curv_interp = np.interp(s_new, s, curv)
        head_interp = np.interp(s_new, s, headings)

        # clothoid-like path 적분
        x0 = xs[0]
        y0 = ys[0]
        theta0 = headings[0]

        xs_out = [x0]
        ys_out = [y0]
        theta = theta0

        for i in range(1, len(s_new)):
            ds_i = s_new[i] - s_new[i - 1]
            # 해당 segment의 곡률 (선형변화라고 보지만, 여기서는 보간값 사용)
            k = curv_interp[i]

            # curvature를 이용해 heading 적분
            theta = theta + k * ds_i

            x_new = xs_out[-1] + ds_i * math.cos(theta)
            y_new = ys_out[-1] + ds_i * math.sin(theta)

            xs_out.append(x_new)
            ys_out.append(y_new)

        return xs_out, ys_out

    def path_cb(self, msg: Path):
        if self.generated:
            return

        if len(msg.poses) < 4:
            self.get_logger().warn("Need at least 4 points for clothoid-like raceline.")
            return

        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]

        xs_new, ys_new = self.generate_clothoid_like(xs, ys, step=0.05)

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
        self.get_logger().info(f"Clothoid-like raceline generated. points={len(xs_new)}")
        self.generated = True

    def save_csv(self, xs, ys):
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            for x, y in zip(xs, ys):
                writer.writerow([float(x), float(y)])


def main(args=None):
    rclpy.init(args=args)
    node = ClothoidRacelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
