#!/usr/bin/env python3
"""
Drive the WHILL in AWSIM with geometry_msgs/Twist and check the motion with /ground_truth/odom.

Sequence: forward -> stop -> backward -> stop -> turn left -> stop -> turn right -> stop.
Each phase is judged from the odometry (displacement along the initial heading, yaw change,
speed at the end of the stop phases) and printed as PASS / FAIL.

    python3 ros2_examples/drive_test.py [--csv drive_test.csv]

Requires AWSIM to be playing, ROS 2 Humble and autoware_vehicle_msgs (gear command).
"""

import argparse
import csv
import math
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from autoware_vehicle_msgs.msg import GearCommand, GearReport

CMD_VEL_TOPIC = "/whill/controller/cmd_vel"
GEAR_CMD_TOPIC = "/control/command/gear_cmd"
GEAR_STATUS_TOPIC = "/vehicle/status/gear_status"
ODOM_TOPIC = "/ground_truth/odom"

# AWSIM subscribes to the command topics with RELIABLE + TRANSIENT_LOCAL.
# A VOLATILE publisher is not compatible with a TRANSIENT_LOCAL subscriber.
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# (name, linear.x [m/s], angular.z [rad/s], duration [s])
PHASES = [
    ("forward", 0.4, 0.0, 3.0),
    ("stop_1", 0.0, 0.0, 2.0),
    ("backward", -0.3, 0.0, 3.0),
    ("stop_2", 0.0, 0.0, 2.0),
    ("turn_left", 0.0, 0.5, 3.0),
    ("stop_3", 0.0, 0.0, 2.0),
    ("turn_right", 0.0, -0.5, 3.0),
    ("stop_4", 0.0, 0.0, 2.0),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DriveTest(Node):
    def __init__(self):
        super().__init__("whill_drive_test")
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, COMMAND_QOS)
        self.gear_pub = self.create_publisher(GearCommand, GEAR_CMD_TOPIC, COMMAND_QOS)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.create_subscription(GearReport, GEAR_STATUS_TOPIC, self._on_gear, 10)
        self._lock = threading.Lock()
        self.odom = []          # (wall time, x, y, yaw, vx, wz, stamp)
        self.gear = None

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self.odom.append((time.time(), p.x, p.y, yaw_of(msg.pose.pose.orientation),
                              msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                              msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))

    def _on_gear(self, msg):
        self.gear = msg.report

    def latest(self):
        with self._lock:
            return self.odom[-1] if self.odom else None

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def set_drive(self, timeout=5.0):
        msg = GearCommand()
        msg.command = GearCommand.DRIVE
        start = time.time()
        while time.time() - start < timeout:
            msg.stamp = self.get_clock().now().to_msg()
            self.gear_pub.publish(msg)
            if self.gear == GearReport.DRIVE:
                return True
            time.sleep(0.1)
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", type=str, default=None, help="Save the odometry to this CSV file")
    args = parser.parse_args()

    rclpy.init()
    node = DriveTest()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"Waiting for {ODOM_TOPIC} ...")
    start = time.time()
    while node.latest() is None:
        if time.time() - start > 15.0:
            print("FAIL: no odometry. Is AWSIM playing with the same ROS_DOMAIN_ID / RMW?")
            sys.exit(1)
        time.sleep(0.1)

    ok_gear = node.set_drive()
    print(f"Gear DRIVE: {'OK' if ok_gear else 'no GearReport=DRIVE within 5 s'}")

    results = []
    for name, v, w, duration in PHASES:
        s0 = node.latest()
        t_end = time.time() + duration
        while time.time() < t_end:
            node.send(v, w)
            time.sleep(0.05)
        s1 = node.latest()

        heading = s0[3]
        dx, dy = s1[1] - s0[1], s1[2] - s0[2]
        along = dx * math.cos(heading) + dy * math.sin(heading)
        lateral = -dx * math.sin(heading) + dy * math.cos(heading)
        dyaw = wrap(s1[3] - s0[3])
        expected_dist = v * duration
        expected_yaw = w * duration

        if name.startswith("stop"):
            passed = abs(s1[4]) < 0.05 and abs(s1[5]) < 0.05
            criterion = "|v| < 0.05 m/s and |w| < 0.05 rad/s at the end"
        elif w == 0.0:
            passed = along * expected_dist > 0 and abs(along) > 0.5 * abs(expected_dist)
            criterion = f"moved > 50% of {expected_dist:+.2f} m along the heading"
        else:
            passed = dyaw * expected_yaw > 0 and abs(dyaw) > 0.5 * abs(expected_yaw)
            criterion = f"turned > 50% of {math.degrees(expected_yaw):+.0f} deg"
        results.append((name, passed))
        print(f"{'PASS' if passed else 'FAIL'}  {name:<10} cmd=({v:+.2f} m/s, {w:+.2f} rad/s, {duration:.0f} s)  "
              f"along={along:+.3f} m lateral={lateral:+.3f} m dyaw={math.degrees(dyaw):+.1f} deg  "
              f"v_end={s1[4]:+.3f} w_end={s1[5]:+.3f}  [{criterion}]")

    node.send(0.0, 0.0)

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["wall_time", "x", "y", "yaw", "vx", "wz", "stamp"])
            writer.writerows(node.odom)
        print(f"Saved {len(node.odom)} odometry samples to {args.csv}")

    n_fail = sum(1 for _, p in results if not p)
    print(f"\n{len(results) - n_fail}/{len(results)} phases passed")
    executor.shutdown()   # stop spinning before destroying the node
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
