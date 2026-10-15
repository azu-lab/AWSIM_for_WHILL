#!/usr/bin/env python3
"""
Check that an infrastructure LiDAR sees the WHILL.

Takes one PointCloud2 of the infrastructure LiDAR, transforms it into the map frame with
the static transform from /tf_static, and counts the points inside a box around the WHILL
(pose from /ground_truth/odom, i.e. base_link = center of the rear axle).

    python3 ros2_examples/infra_lidar_check.py [--topic /infrastructure/lidar_0/points] [--samples 5]
"""

import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

# Box around the WHILL in its own frame (x forward from the rear axle, y left, z up from the axle) [m]
BOX_X = (-0.25, 0.75)
BOX_Y = (-0.35, 0.35)
BOX_Z = (-0.10, 0.85)


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def xyz_of(cloud: PointCloud2) -> np.ndarray:
    offsets = {f.name: f.offset for f in cloud.fields}
    raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step)
    cols = [raw[:, offsets[a]:offsets[a] + 4].copy().view(np.float32).reshape(-1) for a in ("x", "y", "z")]
    return np.stack(cols, axis=1).astype(np.float64)


class Check(Node):
    def __init__(self, topic):
        super().__init__("infra_lidar_check")
        self.cloud = None
        self.odom = None
        self.tf = {}
        self.create_subscription(PointCloud2, topic, self._on_cloud, SENSOR_QOS)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, 10)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf, LATCHED_QOS)

    def _on_cloud(self, msg):
        self.cloud = msg

    def _on_odom(self, msg):
        self.odom = msg

    def _on_tf(self, msg):
        for t in msg.transforms:
            self.tf[t.child_frame_id] = t


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/infrastructure/lidar_0/points")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--min-points", type=int, default=20, help="PASS threshold")
    args = parser.parse_args()

    rclpy.init()
    node = Check(args.topic)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    counts = []
    t0 = time.time()
    while len(counts) < args.samples and time.time() - t0 < 30:
        time.sleep(0.2)
        cloud, odom = node.cloud, node.odom
        if cloud is None or odom is None or cloud.header.frame_id not in node.tf:
            continue
        node.cloud = None
        tf = node.tf[cloud.header.frame_id].transform
        r = quat_to_matrix(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
        pts = xyz_of(cloud) @ r.T + np.array([tf.translation.x, tf.translation.y, tf.translation.z])

        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        d = pts - np.array([p.x, p.y, p.z])
        local_x = d[:, 0] * math.cos(yaw) + d[:, 1] * math.sin(yaw)
        local_y = -d[:, 0] * math.sin(yaw) + d[:, 1] * math.cos(yaw)
        inside = ((BOX_X[0] < local_x) & (local_x < BOX_X[1]) & (BOX_Y[0] < local_y) & (local_y < BOX_Y[1])
                  & (BOX_Z[0] < d[:, 2]) & (d[:, 2] < BOX_Z[1]))
        n = int(inside.sum())
        dist = math.dist((p.x, p.y), (tf.translation.x, tf.translation.y))
        counts.append(n)
        print(f"cloud: {len(pts)} points, WHILL at ({p.x:.2f}, {p.y:.2f}), {dist:.2f} m from the sensor -> "
              f"{n} points on the WHILL")

    if not counts:
        print("FAIL: no point cloud / odometry / static transform received")
        sys.exit(1)
    median = sorted(counts)[len(counts) // 2]
    passed = median >= args.min_points
    print(f"{'PASS' if passed else 'FAIL'}  median {median} points on the WHILL (threshold {args.min_points})")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
