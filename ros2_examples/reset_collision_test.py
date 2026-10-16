#!/usr/bin/env python3
"""
Check the episode interface used for reinforcement learning:

1. /reset_command (std_msgs/Bool true) -> /reset_response (true), WHILL back at its start pose.
2. Drive straight on until /collision_sensor reports a collision with an "Obstacle"-tagged object.
3. Reset again and check that the pose is back at the start.
4. Send a start pose (/awsim/episode/start_pose), reset, check the pose; restore the original one.
5. Move the goal (/awsim/episode/goal), check /awsim/goal_position; restore it.

    python3 ros2_examples/reset_collision_test.py [--speed 0.4] [--timeout 25]

Requires AWSIM to be playing (corridor scene) and autoware_vehicle_msgs.
"""

import argparse
import math
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from autoware_vehicle_msgs.msg import GearCommand

COMMAND_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)


class EpisodeTest(Node):
    def __init__(self):
        super().__init__("whill_reset_collision_test")
        self.cmd_pub = self.create_publisher(Twist, "/whill/controller/cmd_vel", COMMAND_QOS)
        self.gear_pub = self.create_publisher(GearCommand, "/control/command/gear_cmd", COMMAND_QOS)
        self.reset_pub = self.create_publisher(Bool, "/reset_command", RELIABLE_QOS)
        self.start_pub = self.create_publisher(Pose, "/awsim/episode/start_pose", RELIABLE_QOS)
        self.goal_pub = self.create_publisher(Point, "/awsim/episode/goal", RELIABLE_QOS)
        self.create_subscription(Bool, "/reset_response", self._on_reset_response, RELIABLE_QOS)
        self.create_subscription(Bool, "/collision_sensor", self._on_collision, RELIABLE_QOS)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, 10)
        self.create_subscription(Point, "/awsim/goal_position", self._on_goal, RELIABLE_QOS)
        self.reset_ok = False
        self.collision = False
        self.collision_time = None
        self.pose = None
        self.yaw = 0.0
        self.goal = None

    def _on_reset_response(self, msg):
        if msg.data:
            self.reset_ok = True

    def _on_collision(self, msg):
        if msg.data:
            self.collision = True
            self.collision_time = time.time()

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def send_start_pose(self, x, y, yaw):
        msg = Pose()
        msg.position.x, msg.position.y = float(x), float(y)
        msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.start_pub.publish(msg)

    def _on_goal(self, msg):
        self.goal = (msg.x, msg.y, msg.z)

    def drive(self, v, w=0.0):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def gear_drive(self):
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = GearCommand.DRIVE
        self.gear_pub.publish(msg)

    def reset(self, timeout=5.0):
        self.reset_ok = False
        self.drive(0.0)
        start = time.time()
        while time.time() - start < timeout:
            self.reset_pub.publish(Bool(data=True))
            t = time.time()
            while time.time() - t < 0.5:
                if self.reset_ok:
                    return True
                time.sleep(0.02)
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.4, help="Forward speed [m/s]")
    parser.add_argument("--timeout", type=float, default=25.0, help="Max. driving time [s]")
    args = parser.parse_args()

    rclpy.init()
    node = EpisodeTest()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    start = time.time()
    while node.pose is None and time.time() - start < 15.0:
        time.sleep(0.1)
    if node.pose is None:
        print("FAIL: no /ground_truth/odom")
        sys.exit(1)

    failures = 0

    ok = node.reset()
    time.sleep(1.0)
    start_pose = node.pose
    print(f"{'PASS' if ok else 'FAIL'}  reset #1: /reset_response received={ok}, start pose=({start_pose[0]:.3f}, {start_pose[1]:.3f})")
    failures += 0 if ok else 1
    print(f"INFO  goal position (/awsim/goal_position): {node.goal}")

    node.collision = False
    t0 = time.time()
    while time.time() - t0 < args.timeout and not node.collision:
        node.gear_drive()
        node.drive(args.speed)
        time.sleep(0.05)
    node.drive(0.0)
    travelled = math.dist(start_pose, node.pose)
    if node.collision:
        print(f"PASS  collision detected after {node.collision_time - t0:.1f} s, {travelled:.2f} m from the start")
    else:
        print(f"FAIL  no /collision_sensor message within {args.timeout:.0f} s ({travelled:.2f} m travelled)")
        failures += 1

    ok = node.reset()
    time.sleep(1.0)
    back = math.dist(start_pose, node.pose)
    passed = ok and back < 0.1
    print(f"{'PASS' if passed else 'FAIL'}  reset #2: response={ok}, distance to start pose={back:.3f} m (< 0.1 m)")
    failures += 0 if passed else 1

    # 4. Start pose
    orig = (node.pose[0], node.pose[1], node.yaw)
    target = (orig[0] + 0.3, orig[1] - 0.2, orig[2] + 0.5)
    node.send_start_pose(*target)
    time.sleep(0.3)
    ok = node.reset()
    time.sleep(1.0)
    err = math.dist(target[:2], node.pose)
    dyaw = abs(math.atan2(math.sin(node.yaw - target[2]), math.cos(node.yaw - target[2])))
    passed = ok and err < 0.05 and dyaw < math.radians(2)
    print(f"{'PASS' if passed else 'FAIL'}  start pose: commanded ({target[0]:.3f}, {target[1]:.3f}, {math.degrees(target[2]):.1f} deg) "
          f"-> odom ({node.pose[0]:.3f}, {node.pose[1]:.3f}, {math.degrees(node.yaw):.1f} deg), error {err:.3f} m / {math.degrees(dyaw):.2f} deg")
    failures += 0 if passed else 1
    node.send_start_pose(*orig)
    time.sleep(0.3)
    node.reset()
    time.sleep(1.0)

    # 5. Goal
    if node.goal is not None:
        orig_goal = node.goal
        new_goal = (orig_goal[0] + 0.5, orig_goal[1] - 0.5)
        node.goal_pub.publish(Point(x=new_goal[0], y=new_goal[1], z=0.0))
        t0 = time.time()
        while time.time() - t0 < 3.0 and math.dist(new_goal, node.goal[:2]) > 0.01:
            time.sleep(0.05)
        passed = math.dist(new_goal, node.goal[:2]) <= 0.01
        print(f"{'PASS' if passed else 'FAIL'}  goal moved to ({new_goal[0]:.3f}, {new_goal[1]:.3f}): "
              f"/awsim/goal_position = ({node.goal[0]:.3f}, {node.goal[1]:.3f})")
        failures += 0 if passed else 1
        node.goal_pub.publish(Point(x=orig_goal[0], y=orig_goal[1], z=0.0))
        time.sleep(1.5)
    else:
        print("FAIL  no /awsim/goal_position received")
        failures += 1

    executor.shutdown()   # stop spinning before destroying the node
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
