#!/usr/bin/env python3
"""
Measure the ROS 2 outputs of AWSIM for WHILL: rate, frame_id, size, point count and the
offset of header.stamp from /clock, for each topic. Prints a Markdown table.

    python3 ros2_examples/check_interfaces.py [--duration 10] [--json result.json]

Every topic is subscribed with BEST_EFFORT + VOLATILE, which matches both reliable and
best-effort publishers; /tf_static is subscribed with TRANSIENT_LOCAL to get the latched
message.
"""

import argparse
import json
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Bool
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point

TOPICS = [
    ("/clock", Clock),
    ("/ground_truth/odom", Odometry),
    ("/infrastructure/lidar_0/points", PointCloud2),
    ("/sensing/lidar/top/pointcloud_raw", PointCloud2),
    ("/sensing/lidar/top/pointcloud_raw_ex", PointCloud2),
    ("/sensing/camera/front/image_raw", Image),
    ("/sensing/camera/front/camera_info", CameraInfo),
    ("/whill/wheel_velocities_sim", Float32MultiArray),
    ("/collision_sensor", Bool),
    ("/awsim/goal_position", Point),
    ("/tf_static", TFMessage),
]

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=10)


def stamp_of(msg):
    if hasattr(msg, "header"):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if isinstance(msg, Clock):
        return msg.clock.sec + msg.clock.nanosec * 1e-9
    return None


class Probe(Node):
    def __init__(self):
        super().__init__("awsim_interface_probe")
        self.lock = threading.Lock()
        self.stats = {t: {"type": c.__module__.split(".")[0] + "/msg/" + c.__name__, "count": 0, "first": None,
                          "last": None, "frame_id": None, "child_frame_id": None, "bytes": None, "points": None,
                          "clock_offsets": [], "extra": None} for t, c in TOPICS}
        self.clock = None
        for topic, cls in TOPICS:
            qos = LATCHED_QOS if topic == "/tf_static" else SENSOR_QOS
            self.create_subscription(cls, topic, lambda m, t=topic: self._on(t, m), qos)

    def _on(self, topic, msg):
        now = time.time()
        with self.lock:
            s = self.stats[topic]
            s["count"] += 1
            s["first"] = s["first"] or now
            s["last"] = now
            if isinstance(msg, Clock):
                self.clock = stamp_of(msg)
            st = stamp_of(msg)
            if st is not None and self.clock is not None and topic != "/clock":
                s["clock_offsets"].append(st - self.clock)
            if hasattr(msg, "header"):
                s["frame_id"] = msg.header.frame_id
            if isinstance(msg, Odometry):
                s["child_frame_id"] = msg.child_frame_id
            if isinstance(msg, PointCloud2):
                s["points"] = msg.width * msg.height
                s["bytes"] = len(msg.data)
                s["extra"] = "fields=" + ",".join(f.name for f in msg.fields)
            if isinstance(msg, Image):
                s["bytes"] = len(msg.data)
                s["extra"] = f"{msg.width}x{msg.height} {msg.encoding}"
            if isinstance(msg, TFMessage):
                s["extra"] = "; ".join(f"{t.header.frame_id}->{t.child_frame_id} "
                                       f"t=({t.transform.translation.x:.2f},{t.transform.translation.y:.2f},"
                                       f"{t.transform.translation.z:.2f})" for t in msg.transforms)
            if isinstance(msg, Point):
                s["extra"] = f"({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
            if isinstance(msg, Float32MultiArray):
                s["extra"] = f"len={len(msg.data)}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--json", type=str, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print(f"Listening for {args.duration:.0f} s ...", file=sys.stderr)
    time.sleep(args.duration)

    rows = []
    with node.lock:
        for topic, _ in TOPICS:
            s = node.stats[topic]
            span = (s["last"] - s["first"]) if s["count"] > 1 else 0.0
            rate = (s["count"] - 1) / span if span > 0 else 0.0
            offs = s["clock_offsets"]
            rows.append({
                "topic": topic, "type": s["type"], "count": s["count"], "rate_hz": round(rate, 2),
                "frame_id": s["frame_id"], "child_frame_id": s["child_frame_id"], "bytes": s["bytes"],
                "points": s["points"],
                "stamp_minus_clock_s": None if not offs else round(sorted(offs)[len(offs) // 2], 4),
                "extra": s["extra"],
            })

    print("| topic | type | msgs | rate [Hz] | frame_id | size [B] | points | stamp - /clock [s] | note |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        frame = r["frame_id"] or ""
        if r["child_frame_id"]:
            frame += f" -> {r['child_frame_id']}"
        print(f"| `{r['topic']}` | {r['type']} | {r['count']} | {r['rate_hz']} | {frame} | {r['bytes'] or ''} | "
              f"{r['points'] or ''} | {'' if r['stamp_minus_clock_s'] is None else r['stamp_minus_clock_s']} | {r['extra'] or ''} |")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(rows, f, indent=2)
    executor.shutdown()   # stop spinning before destroying the node
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
