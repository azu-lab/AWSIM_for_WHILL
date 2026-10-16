#!/usr/bin/env python3
# Copyright 2026 shoumexi-03, Apache License 2.0
"""Count the messages of sensor topics (check that every copy publishes; resource measurement).

    python3 scripts/bench/topic_check.py --duration 6 \
        --lidar /infrastructure/lidar_0/points --camera /sensing/camera/front/image_raw

Subscribes all topics at once and prints one JSON line {topic: Hz} after `duration` seconds.
Called by scripts/bench_resources.sh in the ROS 2 container.
"""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--lidar", nargs="*", default=[])
    ap.add_argument("--camera", nargs="*", default=[])
    args = ap.parse_args()

    rclpy.init()
    node = Node("awsim_bench_topic_check")
    counts = {t: 0 for t in args.lidar + args.camera}

    def counter(topic):
        def cb(_msg):
            counts[topic] += 1
        return cb

    # A BEST_EFFORT subscriber matches RELIABLE and BEST_EFFORT publishers.
    for t in args.lidar:
        node.create_subscription(PointCloud2, t, counter(t), qos_profile_sensor_data)
    for t in args.camera:
        node.create_subscription(Image, t, counter(t), qos_profile_sensor_data)

    # Do not count the discovery time: drop the first 2 s.
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    for t in counts:
        counts[t] = 0
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.monotonic() - start

    print(json.dumps({t: round(c / elapsed, 2) for t, c in counts.items()}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
