#!/usr/bin/env python3
"""
Drive the WHILL in AWSIM from the keyboard (terminal), over ROS 2.

    scripts/ros2_shell.sh python3 ros2_examples/teleop_keyboard.py

Keys:  w / s  : faster forward / backward (step 0.1 m/s)
       a / d  : turn left / right         (step 0.1 rad/s)
       space  : stop
       q      : quit (sends a stop command)

Sends gear DRIVE once, then geometry_msgs/Twist on /whill/controller/cmd_vel at 10 Hz
(RELIABLE + TRANSIENT_LOCAL, as AWSIM requires) and prints the speed from /ground_truth/odom.
The AWSIM window does not need the focus.
"""

import select
import sys
import termios
import threading
import time
import tty

import rclpy
import rclpy.executors
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from autoware_vehicle_msgs.msg import GearCommand

MAX_V = 1.667        # [m/s] _maxSpeed of the WHILL model
MAX_W = 1.0          # [rad/s]
STEP_V = 0.1
STEP_W = 0.1
QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 history=HistoryPolicy.KEEP_LAST, depth=1)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def main():
    if not sys.stdin.isatty():
        print("Run in an interactive terminal (scripts/ros2_shell.sh passes -it when it has a TTY).")
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("whill_teleop_keyboard")
    cmd_pub = node.create_publisher(Twist, "/whill/controller/cmd_vel", QOS)
    gear_pub = node.create_publisher(GearCommand, "/control/command/gear_cmd", QOS)
    odom = {"v": 0.0, "w": 0.0, "x": 0.0, "y": 0.0}

    def on_odom(msg):
        odom.update(v=msg.twist.twist.linear.x, w=msg.twist.twist.angular.z,
                    x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)
    node.create_subscription(Odometry, "/ground_truth/odom", on_odom, 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    gear = GearCommand()
    gear.stamp = node.get_clock().now().to_msg()
    gear.command = GearCommand.DRIVE
    gear_pub.publish(gear)

    print(__doc__.split("Sends")[0])
    v, w = 0.0, 0.0
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        last = 0.0
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == "w":
                    v = clamp(v + STEP_V, -MAX_V, MAX_V)
                elif key == "s":
                    v = clamp(v - STEP_V, -MAX_V, MAX_V)
                elif key == "a":
                    w = clamp(w + STEP_W, -MAX_W, MAX_W)
                elif key == "d":
                    w = clamp(w - STEP_W, -MAX_W, MAX_W)
                elif key == " ":
                    v, w = 0.0, 0.0
                elif key == "q":
                    break
            if time.time() - last >= 0.1:
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                cmd_pub.publish(msg)
                last = time.time()
                sys.stdout.write(f"\rcmd v={v:+.1f} m/s w={w:+.1f} rad/s | odom v={odom['v']:+.2f} "
                                 f"w={odom['w']:+.2f} pos=({odom['x']:.1f}, {odom['y']:.1f})   ")
                sys.stdout.flush()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        cmd_pub.publish(Twist())
        time.sleep(0.2)
        print("\nstopped")
        # stop the spin thread before destroying the node (else: "terminate called without an active exception")
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
