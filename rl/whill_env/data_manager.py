"""
Data management code (paper Fig. 2): the only module that talks to AWSIM.

Hybrid communication (paper Sec. III-C, Table III):

- Sensor data over ROS 2, each kept as "latest value" (KEEP_LAST, drop oldest):

  | Topic                              | Type                    | Rate  | QoS (subscriber)  | Stamp used | Queue |
  |------------------------------------|-------------------------|-------|-------------------|------------|-------|
  | /sensing/camera/front/image_raw    | sensor_msgs/Image       | 10 Hz | BEST_EFFORT       | yes        | 1     |
  | /ground_truth/odom                 | nav_msgs/Odometry       | 20 Hz | BEST_EFFORT       | no         | 1     |
  | /collision_sensor                  | std_msgs/Bool           | event | RELIABLE          | no         | 10    |
  | /whill/controller/cmd_vel (pub.)   | geometry_msgs/Twist     | 10 Hz | RELIABLE (+ TRANSIENT_LOCAL, required by AWSIM) | no | 1 |

  The vehicle LiDAR (/sensing/lidar/top/pointcloud_raw, 10 Hz, BEST_EFFORT, 1) is not part of the
  observation and is therefore not subscribed. cmd_vel is sent once per decision step, i.e. at the
  camera rate. Also used: the goal position and the obstacle poses (plots), the gear command.
- Episode reset (state management) over a synchronous TCP request ("reset_episode" of the
  OptimizerBridge of AWSIM, 127.0.0.1:50051): start pose and goal of the episode, reset, and the
  simulation time of the reset in the answer, to take the first camera frame after it.

It knows nothing about rewards, episodes or models; those are in env.py and agents/. To run the
same learning environment on another simulator or on the real vehicle, replace this class.
"""

import json
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .task import Episode

GEAR_DRIVE = 2   # autoware_vehicle_msgs/GearCommand.DRIVE


@dataclass
class Snapshot:
    """Latest values of all inputs, taken at one instant (the decision step)."""
    image: Optional[np.ndarray]          # RGB uint8, full camera resolution
    image_stamp: float                   # header stamp of the image [s] (simulation time)
    x: float                             # base_link in the map frame [m]
    y: float
    z: float
    yaw: float                           # [rad]
    linear_vel: float                    # [m/s], base_link frame
    angular_vel: float                   # [rad/s]
    goal: Optional[Tuple[float, float]]  # goal object (map x, y), from AWSIM
    obstacles: Tuple[Tuple[float, float, float], ...]  # randomized scene obstacles (x, y, yaw); for plots/checks only
    collision: bool                      # a collision was reported since the previous snapshot


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StateClient:
    """Synchronous TCP requests to the OptimizerBridge of AWSIM (4-byte big-endian length + JSON)."""

    def __init__(self, host: str, port: int, timeout: float):
        self._address = (host, port)
        self._timeout = timeout
        self._sock: Optional[socket.socket] = None

    def request(self, method: str, params: Dict[str, Any]) -> Dict[str, Any]:
        for attempt in range(2):   # one reconnect (AWSIM restarted, connection dropped)
            try:
                if self._sock is None:
                    self._sock = socket.create_connection(self._address, timeout=self._timeout)
                body = json.dumps({"method": method, "params_json": json.dumps(params)}).encode()
                self._sock.sendall(struct.pack(">I", len(body)) + body)
                size = struct.unpack(">I", self._recv(4))[0]
                return json.loads(self._recv(size).decode())
            except (OSError, ConnectionError, struct.error) as e:
                self.close()
                if attempt == 1:
                    raise ConnectionError(f"TCP request '{method}' to AWSIM {self._address} failed: {e}") from e
        raise AssertionError("unreachable")

    def _recv(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                raise ConnectionError("connection closed by AWSIM")
            data += chunk
        return data

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


class DataManager:

    def __init__(self, topics: Dict[str, str], awsim: Dict[str, Any], node_name: str = "whill_env"):
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(node_name)
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._state = StateClient(awsim["host"], int(awsim["port"]), float(awsim["timeout_sec"]))

        self._image: Optional[np.ndarray] = None
        self._image_stamp = 0.0
        self._odom: Optional[Odometry] = None
        self._collision = False
        self._goal: Optional[Tuple[float, float]] = None
        self._obstacles: Tuple[Tuple[float, float, float], ...] = ()
        self.reset_time = 0.0            # simulation time of the last reset (answer of AWSIM)

        def qos(reliability, depth, durability=DurabilityPolicy.VOLATILE):
            return QoSProfile(reliability=reliability, durability=durability,
                              history=HistoryPolicy.KEEP_LAST, depth=depth)
        best_effort_1 = qos(ReliabilityPolicy.BEST_EFFORT, 1)
        reliable_1 = qos(ReliabilityPolicy.RELIABLE, 1)
        # AWSIM subscribes cmd_vel / gear with TRANSIENT_LOCAL (a VOLATILE publisher does not match)
        command_qos = qos(ReliabilityPolicy.RELIABLE, 1, DurabilityPolicy.TRANSIENT_LOCAL)

        n = self._node
        n.create_subscription(Image, topics["camera"], self._on_image, best_effort_1)
        n.create_subscription(Odometry, topics["odom"], self._on_odom, best_effort_1)
        n.create_subscription(Bool, topics["collision"], self._on_collision, qos(ReliabilityPolicy.RELIABLE, 10))
        n.create_subscription(Point, topics["goal_position"], self._on_goal, reliable_1)
        n.create_subscription(PoseArray, topics["obstacles"], self._on_obstacles, reliable_1)

        from autoware_vehicle_msgs.msg import GearCommand
        self._GearCommand = GearCommand
        self._cmd_pub = n.create_publisher(Twist, topics["cmd_vel"], command_qos)
        self._gear_pub = n.create_publisher(GearCommand, topics["gear_cmd"], command_qos)

        self._running = True
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self.send_gear_drive()

    # ------------------------------------------------------------------ callbacks
    def _spin(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.01)

    def _on_image(self, msg: Image):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(f"image conversion: {e}")
            return
        with self._lock:
            self._image = image
            self._image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def _on_collision(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._collision = True

    def _on_goal(self, msg: Point):
        with self._lock:
            self._goal = (msg.x, msg.y)

    def _on_obstacles(self, msg: PoseArray):
        with self._lock:
            self._obstacles = tuple((p.position.x, p.position.y, _yaw(p.orientation)) for p in msg.poses)

    # ------------------------------------------------------------------ aggregate data
    def snapshot(self, consume_collision: bool = True) -> Snapshot:
        with self._lock:
            o = self._odom
            p = o.pose.pose if o is not None else None
            snap = Snapshot(
                image=self._image,
                image_stamp=self._image_stamp,
                x=p.position.x if p else 0.0,
                y=p.position.y if p else 0.0,
                z=p.position.z if p else 0.0,
                yaw=_yaw(p.orientation) if p else 0.0,
                linear_vel=o.twist.twist.linear.x if o else 0.0,
                angular_vel=o.twist.twist.angular.z if o else 0.0,
                goal=self._goal,
                obstacles=self._obstacles,
                collision=self._collision,
            )
            if consume_collision:
                self._collision = False
        return snap

    def wait_for_data(self, timeout: float) -> bool:
        """Until a camera image and an odometry message have been received."""
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._image is not None and self._odom is not None:
                    return True
            time.sleep(0.05)
        return False

    def wait_for_frame(self, after_stamp: float, timeout: float) -> bool:
        """Until a camera image stamped after `after_stamp` arrives (the decision step)."""
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._image_stamp > after_stamp + 1e-6:
                    return True
            time.sleep(0.001)
        return False

    # ------------------------------------------------------------------ send command
    def send_command(self, linear_vel: float, angular_vel: float):
        msg = Twist()
        msg.linear.x = float(linear_vel)
        msg.angular.z = float(angular_vel)
        self._cmd_pub.publish(msg)

    def send_gear_drive(self):
        msg = self._GearCommand()
        msg.command = GEAR_DRIVE
        self._gear_pub.publish(msg)

    # ------------------------------------------------------------------ reset the scenario
    def reset_scenario(self, episode: Episode, timeout: float) -> bool:
        """
        Synchronous reset over TCP: AWSIM sets the start pose / goal of the episode (on the ground),
        resets and answers with the simulation time of the reset (self.reset_time). Then waits until
        the odometry shows the start pose. Returns False if the request failed.
        """
        self.send_command(0.0, 0.0)
        params: Dict[str, Any] = {"has_start": episode.start is not None, "has_goal": episode.goal is not None}
        if episode.start is not None:
            params.update(start_x=episode.start.x, start_y=episode.start.y, start_yaw=episode.start.yaw,
                          has_start_z=episode.start.z is not None, start_z=episode.start.z or 0.0)
        if episode.goal is not None:
            params.update(goal_x=episode.goal.x, goal_y=episode.goal.y,
                          has_goal_z=episode.goal.z is not None, goal_z=episode.goal.z or 0.0)
        try:
            answer = self._state.request("reset_episode", params)
        except ConnectionError as e:
            self._node.get_logger().error(str(e))
            return False
        if not answer.get("success"):
            self._node.get_logger().error(f"reset_episode: {answer.get('message')}")
            return False
        self.reset_time = float(json.loads(answer.get("data_json") or "{}").get("sim_time", 0.0))

        # The reset is done in AWSIM; wait until the odometry (asynchronous, ROS 2) shows it.
        end = time.time() + timeout
        while time.time() < end:
            s = self.snapshot(consume_collision=False)
            if episode.start is None or math.hypot(s.x - episode.start.x, s.y - episode.start.y) < 0.5:
                break
            time.sleep(0.02)
        else:
            self._node.get_logger().warn("odometry does not show the start pose after the reset")
        time.sleep(0.2)   # let the vehicle settle on the ground
        with self._lock:
            self._collision = False   # contacts while being moved do not count
            if episode.goal is not None:
                self._goal = (episode.goal.x, episode.goal.y)
        self.send_gear_drive()
        return True

    def close(self):
        try:
            self.send_command(0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        self._state.close()
        self._running = False
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self._node.destroy_node()
