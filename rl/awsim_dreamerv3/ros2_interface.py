"""
ROS 2 Interface for AWSIM DreamerV3 Environment

Handles all ROS 2 communication:
- Subscribe to camera images
- Subscribe to odometry
- Subscribe to collision events
- Publish velocity commands
- Call reset service
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point, Pose
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import numpy as np
import threading
import time
from typing import Optional, Tuple

from . import config
import socket
import struct
import json
import math
from typing import Optional, Tuple, Any, Dict, List


def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to euler angles (roll, pitch, yaw) in degrees.
    """
    # Validation
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0:
        return 0.0, 0.0, 0.0
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TcpClient:
    """Simple TCP Client for AWSIM communication (Reset)"""
    
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        
    def connect(self) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            return True
        except Exception as e:
            # print(f"TCP Connect failed: {e}")
            return False
            
    def close(self):
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            
    def _send_request(self, method: str, params: Dict[str, Any]) -> Dict[str, Any]:
        if self.socket is None:
            if not self.connect():
                return {"success": False, "message": "Connection failed"}
                
        request = {
            "method": method,
            "params_json": json.dumps(params)
        }
        
        try:
            request_bytes = json.dumps(request).encode('utf-8')
            
            # Send length prefix (4 bytes big-endian)
            length_prefix = struct.pack('>I', len(request_bytes))
            self.socket.sendall(length_prefix + request_bytes)
            
            # Receive length prefix
            length_bytes = self._recv_exact(4)
            response_length = struct.unpack('>I', length_bytes)[0]
            
            # Receive response body
            response_bytes = self._recv_exact(response_length)
            response_json = response_bytes.decode('utf-8')
            
            return json.loads(response_json)
        except Exception as e:
            self.close()
            return {"success": False, "message": str(e)}
            
    def _recv_exact(self, n: int) -> bytes:
        data = b''
        while len(data) < n:
            chunk = self.socket.recv(n - len(data))
            if not chunk:
                raise ConnectionError("Connection closed")
            data += chunk
        return data

    def reset_simulation(self, position: Tuple[float, float, float], rotation: Tuple[float, float, float]) -> bool:
        params = {
            "position_x": position[0],
            "position_y": position[1],
            "position_z": position[2],
            "rotation_x": rotation[0],
            "rotation_y": rotation[1],
            "rotation_z": rotation[2],
        }
        response = self._send_request("reset_simulation", params)
        return response.get("success", False)


class ROS2Interface:
    """
    ROS 2 interface for AWSIM wheelchair environment.
    Provides synchronous access to sensor data and control.
    """
    
    def __init__(self, node_name: str = "awsim_dreamerv3_env"):
        """
        Initialize ROS 2 interface.
        
        Args:
            node_name: Name for the ROS 2 node
        """
        # Initialize ROS 2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        self._node = rclpy.create_node(node_name)
        self._bridge = CvBridge()
        
        # TCP Client for reliable reset
        self._tcp_client = TcpClient(config.AWSIM_HOST, config.AWSIM_PORT)
        
        # Data storage (thread-safe)
        self._lock = threading.Lock()
        self._latest_image: Optional[np.ndarray] = None
        self._latest_image_timestamp: float = 0.0
        self._latest_odom: Optional[Odometry] = None
        self._collision_detected: bool = False
        self._initial_position: Optional[np.ndarray] = None
        self._initial_orientation: Optional[Any] = None
        self._initial_frame_id: str = "map"  # Default to map
        self._initial_orientation: Optional[Any] = None
        self._initial_frame_id: str = "map"  # Default to map
        self._goal_position: Optional[Tuple[float, float, float]] = None
        self._initial_after_reset = False
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for reliable topics
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self._image_sub = self._node.create_subscription(
            Image,
            config.CAMERA_TOPIC,
            self._image_callback,
            sensor_qos
        )
        
        self._odom_sub = self._node.create_subscription(
            Odometry,
            config.ODOM_TOPIC,
            self._odom_callback,
            sensor_qos
        )
        
        self._collision_sub = self._node.create_subscription(
            Bool,
            config.COLLISION_TOPIC,
            self._collision_callback,
            reliable_qos
        )
        
        # Goal position subscriber (TRANSIENT_LOCAL to receive latched messages)
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._goal_position_sub = self._node.create_subscription(
            Point,
            config.GOAL_POSITION_TOPIC,
            self._goal_position_callback,
            goal_qos
        )
        
        # QoS for cmd_vel and gear_cmd (must match AWSIM subscriber)
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self._cmd_vel_pub = self._node.create_publisher(
            Twist,
            config.CMD_VEL_TOPIC,
            cmd_qos
        )
        
        # Gear command publisher (autoware_vehicle_msgs/GearCommand)
        from autoware_vehicle_msgs.msg import GearCommand
        self._gear_cmd_pub = self._node.create_publisher(
            GearCommand,
            config.GEAR_CMD_TOPIC,
            cmd_qos
        )
        self._GearCommand = GearCommand  # Store class reference
        
        # Reset command publisher and response subscriber (topic-based reset)
        self._reset_cmd_pub = self._node.create_publisher(
            Bool,
            config.RESET_COMMAND_TOPIC,
            reliable_qos
        )
        
        self._reset_response_received = False
        self._reset_response_sub = self._node.create_subscription(
            Bool,
            config.RESET_RESPONSE_TOPIC,
            self._reset_response_callback,
            reliable_qos
        )
        
        # Initial pose publisher (for robust reset)
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped,
            config.INITIAL_POSE_TOPIC,
            reliable_qos
        )
        
        # Episode settings (start pose / goal) for EnvironmentReset
        self._start_pose_pub = self._node.create_publisher(Pose, config.START_POSE_TOPIC, reliable_qos)
        self._goal_pub = self._node.create_publisher(Point, config.GOAL_TOPIC, reliable_qos)
        
        # Start spin thread
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._running = True
        self._spin_thread.start()
        
        # Set gear to Drive on initialization
        self._send_gear_command(drive=True)
        
        self._node.get_logger().info("ROS 2 Interface initialized")
    
    def _spin_loop(self):
        """Background thread for ROS 2 spinning."""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=config.ROS_SPIN_TIMEOUT)
    
    def _image_callback(self, msg: Image):
        """Process incoming camera images."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            with self._lock:
                self._latest_image = cv_image
                # Store timestamp in seconds
                self._latest_image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self._node.get_logger().error(f"Image conversion error: {e}")
    
    def _odom_callback(self, msg: Odometry):
        """Process incoming odometry."""
        with self._lock:
            self._latest_odom = msg
            
            # Capture initial position on first odometry
            if self._initial_position is None:
                self._initial_position = np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ])
                self._initial_orientation = msg.pose.pose.orientation
                self._initial_frame_id = msg.header.frame_id
                self._node.get_logger().info(
                    f"Initial pose captured: ({self._initial_position[0]:.3f}, {self._initial_position[1]:.3f}, {self._initial_position[2]:.3f})"
                )
    
    def _goal_position_callback(self, msg: Point):
        """Process incoming goal position."""
        with self._lock:
            self._goal_position = (msg.x, msg.y, msg.z)
            self._node.get_logger().info(
                f"Goal position received: ROS({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})"
            )
    
    def _collision_callback(self, msg: Bool):
        """Process collision events."""
        if msg.data:
            with self._lock:
                self._collision_detected = True
    
    def _reset_response_callback(self, msg: Bool):
        """Process reset response."""
        if msg.data:
            self._reset_response_received = True
    
    def get_observation(self) -> Tuple[Optional[np.ndarray], Optional[Odometry]]:
        """
        Get latest sensor data.
        
        Returns:
            Tuple of (image, odometry)
        """
        with self._lock:
            image = self._latest_image.copy() if self._latest_image is not None else None
            odom = self._latest_odom
        return image, odom
    
    def get_image(self) -> Optional[np.ndarray]:
        """Get latest camera image."""
        with self._lock:
            if self._latest_image is not None:
                return self._latest_image.copy()
        return None
    
    def get_latest_image_timestamp(self) -> float:
        """Get timestamp of latest camera image."""
        with self._lock:
            return self._latest_image_timestamp
    
    def get_forward_velocity(self) -> float:
        """Get current forward velocity from odometry."""
        with self._lock:
            if self._latest_odom is not None:
                # twist.linear.x is forward velocity in local frame
                return self._latest_odom.twist.twist.linear.x
        return 0.0
    
    def get_distance_from_start(self) -> float:
        """
        Get distance from start in forward direction.
        
        Coordinate System Mapping (Unity -> ROS2 via Ros2Utility):
        - Forward: Unity Z -> ROS X
        - Lateral: Unity -X -> ROS Y
        - Height:  Unity Y -> ROS Z
        
        Returns:
            Distance traveled in forward (ROS X) direction from initial position.
        """
        with self._lock:
            if self._latest_odom is None or self._initial_position is None:
                return 0.0
            
            # Use ROS X as forward direction (was Unity Z)
            current_forward = self._latest_odom.pose.pose.position.x
            return current_forward - self._initial_position[0]
    
    def get_distance_to_goal(self, goal_distance: float) -> float:
        """
        Get distance from current position to goal point on Forward-Lateral plane.
        
        Args:
            goal_distance: Distance to goal in Forward direction (ROS X)
            
        Returns:
            Distance to goal point (ignoring height/ROS Z)
        """
        with self._lock:
            if self._latest_odom is None:
                return float('inf')
            
            # Current position (ROS frame)
            current_pos = np.array([
                self._latest_odom.pose.pose.position.x,
                self._latest_odom.pose.pose.position.y
            ])
            
            # Goal position (ROS frame)
            # If goal position is available from topic, use it
            if self._goal_position is not None:
                target_pos = np.array([self._goal_position[0], self._goal_position[1]])
            else:
                # Fallback to fixed distance
                if self._initial_position is None:
                    return float('inf')
                target_pos = np.array([
                    self._initial_position[0] + goal_distance,
                    self._initial_position[1]
                ])
                
            return np.linalg.norm(target_pos - current_pos)
            
    def get_current_position(self) -> Tuple[float, float]:
        """
        Get current position (x, z).
        
        Returns:
            Tuple of (x, z) coordinates. Uses Odom frame (absolute).
        """
        with self._lock:
            if self._latest_odom is None:
                return 0.0, 0.0
            
            return (
                self._latest_odom.pose.pose.position.x,
                self._latest_odom.pose.pose.position.z
            )
    
    def get_position_3d(self) -> Tuple[float, float, float]:
        """
        Get current 3D position in ROS frame.
        
        Coordinate System Mapping (Unity -> ROS2):
        - Unity Z  -> ROS X (Forward)
        - Unity -X -> ROS Y (Lateral)
        - Unity Y  -> ROS Z (Height)
        
        Returns:
            Tuple of (x, y, z) in ROS frame.
        """
        with self._lock:
            if self._latest_odom is None:
                return 0.0, 0.0, 0.0
            return (
                self._latest_odom.pose.pose.position.x,
                self._latest_odom.pose.pose.position.y,
                self._latest_odom.pose.pose.position.z,
            )
    
    def get_goal_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get goal position in ROS frame (from Unity goal object).
        
        Returns:
            Tuple of (x, y, z) in ROS frame, or None if not yet received.
        """
        with self._lock:
            return self._goal_position
    
    def get_initial_position(self) -> Optional[np.ndarray]:
        """
        Get initial position [x, y, z] in ROS frame (captured on first odom).
        
        Returns:
            numpy array [x, y, z] or None if not yet captured.
        """
        with self._lock:
            if self._initial_position is not None:
                return self._initial_position.copy()
            return None
    
    def check_collision(self) -> bool:
        """
        Check if collision was detected since last check.
        Resets the collision flag after reading.
        
        Returns:
            True if collision occurred
        """
        with self._lock:
            collision = self._collision_detected
            self._collision_detected = False
        return collision
    
    def send_velocity_command(self, linear_vel: float, angular_vel: float):
        """
        Send velocity command to wheelchair.
        
        Args:
            linear_vel: Forward velocity in m/s
            angular_vel: Yaw rate in rad/s
        """
        msg = Twist()
        msg.linear.x = float(linear_vel)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_vel)
        
        self._cmd_vel_pub.publish(msg)
    
    def _send_gear_command(self, drive: bool = True):
        """
        Send gear command to wheelchair.
        
        Args:
            drive: True for Drive gear, False for Park
        """
        msg = self._GearCommand()
        # GearCommand constants: DRIVE=2, PARK=22 (from autoware_vehicle_msgs)
        msg.command = 2 if drive else 22  # DRIVE=2
        self._gear_cmd_pub.publish(msg)
        self._node.get_logger().info(f"Gear command sent: {'DRIVE' if drive else 'PARK'}")
    
    def set_episode(self, start_pose: Optional[Dict[str, float]] = None,
                    goal: Optional[Dict[str, float]] = None):
        """
        Send the start pose (applied by AWSIM at the next reset) and the goal (moved at once).

        Args:
            start_pose: {"x", "y", "yaw"} of base_link in the map frame
            goal: {"x", "y"} in the map frame
        """
        if start_pose is not None:
            msg = Pose()
            msg.position.x = float(start_pose["x"])
            msg.position.y = float(start_pose["y"])
            yaw = float(start_pose.get("yaw", 0.0))
            msg.orientation.z = math.sin(yaw / 2.0)
            msg.orientation.w = math.cos(yaw / 2.0)
            self._start_pose_pub.publish(msg)
        if goal is not None:
            msg = Point()
            msg.x = float(goal["x"])
            msg.y = float(goal["y"])
            self._goal_pub.publish(msg)
            with self._lock:
                # The goal topic of AWSIM catches up within one frame; use the commanded value meanwhile.
                old = self._goal_position
                self._goal_position = (msg.x, msg.y, old[2] if old else 0.0)
        if start_pose is not None:
            self._initial_after_reset = False   # re-capture the start pose after the next reset
        if start_pose is not None or goal is not None:
            time.sleep(0.2)   # let AWSIM process them before the reset command

    def reset(self, timeout: float = 5.0) -> bool:
        """
        Send reset command and wait for response.
        
        Args:
            timeout: Maximum time to wait for reset response
            
        Returns:
            True if reset was successful
        """
        # Clear response flag
        self._reset_response_received = False
        
        # ---------------------------------------------------------
        # 1. TCP Direct Reset (Most Reliable) -> DISABLED TO PREVENT DRIFT
        # ---------------------------------------------------------
        # We disabled TCP reset because sending explicit coordinates might cause drift
        # if the coordinate system or offset is slightly mismatched.
        # Unity's EnvironmentReset.cs handles reset to initial position correctly without args.
        # 
        # if self._initial_position is not None and self._initial_orientation is not None:
        #     # Convert quaternion to euler (degrees)
        #     rx, ry, rz = quaternion_to_euler(
        #         self._initial_orientation.x,
        #         self._initial_orientation.y,
        #         self._initial_orientation.z,
        #         self._initial_orientation.w
        #     )
        #     
        #     # Call TCP reset
        #     if self._tcp_client.reset_simulation(
        #         position=(float(self._initial_position[0]), float(self._initial_position[1]), float(self._initial_position[2])),
        #         rotation=(rx, ry, rz)
        #     ):
        #         self._node.get_logger().info("TCP Reset successful")
        #     else:
        #         self._node.get_logger().warn("TCP Reset failed, falling back to ROS topics")
        
        # ---------------------------------------------------------
        # 2. ROS Topic Reset (Backup -> NOW PRIMARY)
        # ---------------------------------------------------------
        # Publish reset command
        msg = Bool()
        msg.data = True
        self._reset_cmd_pub.publish(msg)
        
        # Also publish initial pose if available (Robust Reset)
        if self._initial_position is not None and self._initial_orientation is not None:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self._node.get_clock().now().to_msg()
            pose_msg.header.frame_id = self._initial_frame_id
            
            pose_msg.pose.pose.position.x = self._initial_position[0]
            pose_msg.pose.pose.position.y = self._initial_position[1]
            pose_msg.pose.pose.position.z = self._initial_position[2]
            pose_msg.pose.pose.orientation = self._initial_orientation
            
            # Publish multiple times to ensure delivery (Reliable Reset)
            for _ in range(5):
                self._initial_pose_pub.publish(pose_msg)
                time.sleep(0.05)
                
            self._node.get_logger().info(f"Initial pose published 5 times (frame: {self._initial_frame_id})")
        
        self._node.get_logger().info("Reset command sent")
        
        # Wait for response
        start_time = time.time()
        while not self._reset_response_received:
            if time.time() - start_time > timeout:
                self._node.get_logger().error("Reset response timed out")
                return False
            time.sleep(0.05)
        
        # Reset internal state
        with self._lock:
            self._collision_detected = False
            # KEEP INITIAL POSITION/ORIENTATION: Do not clear it on reset!
            # We want to reuse the very first position as the reset point always.
            # self._initial_position = None  <-- Removed
        
        # The "initial" pose was captured from the first odometry message, i.e. wherever the
        # WHILL happened to be before the first reset (e.g. where the previous run left it).
        # Re-capture it once, after the first reset, so that wait_for_reset_completion()
        # compares with the actual start pose. Also after a new start pose was sent.
        if not self._initial_after_reset:
            time.sleep(0.3)   # let the odometry of the reset pose arrive
            with self._lock:
                if self._latest_odom is not None:
                    p = self._latest_odom.pose.pose.position
                    self._initial_position = np.array([p.x, p.y, p.z])
                    self._initial_orientation = self._latest_odom.pose.pose.orientation
                    self._initial_after_reset = True
        
        # Re-send gear command after reset
        self._send_gear_command(drive=True)
        
        self._node.get_logger().info("Reset completed successfully")
        return True
    
    def wait_for_reset_completion(self, timeout: float = 5.0, tolerance: float = 2.0) -> bool:
        """
        Wait until the vehicle returns to the initial position.
        
        Args:
            timeout: Maximum time to wait in seconds
            tolerance: Maximum allowed distance from initial position in meters
            
        Returns:
            True if reset completion was confirmed, False if timed out
        """
        start_time = time.time()
        
        self._node.get_logger().info("Waiting for vehicle to return to start...")
        
        while time.time() - start_time < timeout:
            with self._lock:
                if self._latest_odom is not None and self._initial_position is not None:
                    current_pos = np.array([
                        self._latest_odom.pose.pose.position.x,
                        self._latest_odom.pose.pose.position.y,
                        self._latest_odom.pose.pose.position.z
                    ])
                    # Calculate distance to initial position
                    dist = np.linalg.norm(current_pos - self._initial_position)
                    
                    if dist < tolerance:
                        self._node.get_logger().info(f"Vehicle returned to start (dist: {dist:.3f}m)")
                        return True
            
            time.sleep(0.1)
            
        self._node.get_logger().warn(f"Reset wait timed out! Vehicle might not be at start.")
        return False
    
    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """
        Wait until sensor data is available.
        
        Args:
            timeout: Maximum time to wait
            
        Returns:
            True if data is available
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self._lock:
                if self._latest_image is not None and self._latest_odom is not None:
                    return True
            time.sleep(0.1)
        return False
    
    def destroy(self):
        """Clean up ROS 2 resources."""
        self._running = False
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self._node.destroy_node()
