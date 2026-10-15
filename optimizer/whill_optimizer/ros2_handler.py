
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from autoware_vehicle_msgs.msg import GearCommand, GearReport
import threading
import time
import math

class ROS2Handler(Node):
    """
    ROS2通信を担当するハンドラクラス
    whill_optimizer からの指示に従い、AWSIMに対してcmd_vel/gear_cmdを送信し、
    odomを受信する。
    """
    def __init__(self, odom_topic='/ground_truth/odom', cmd_vel_topic='/whill/controller/cmd_vel'):
        super().__init__('optimizer_ros2_handler')
        
        # QoS設定: TRANSIENT_LOCAL (AWSIMの要求に合わせる)
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos_profile)
        self.gear_pub = self.create_publisher(GearCommand, '/control/command/gear_cmd', qos_profile)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        
        # Gear report subscriber (for verification)
        self.gear_sub = self.create_subscription(
            GearReport, '/vehicle/status/gear_status', self.gear_status_callback, 10
        )
        
        self.lock = threading.Lock()
        self.odom_buffer = []  # 受信したOdomデータを蓄積
        self.latest_gear = None
        
        print(f"ROS2Handler initialized: cmd_vel={cmd_vel_topic}, odom={odom_topic}")
        
    def odom_callback(self, msg: Odometry):
        """Odom受信コールバック"""
        with self.lock:
            self.odom_buffer.append(msg)
            
    def gear_status_callback(self, msg: GearReport):
        """Gear受信コールバック"""
        self.latest_gear = msg.report
        
    def clear_odom_buffer(self):
        """Odomバッファをクリア"""
        with self.lock:
            self.odom_buffer = []
            
    def get_odom_buffer(self):
        """現在のOdomバッファを取得"""
        with self.lock:
            return list(self.odom_buffer)
            
    def set_gear_drive(self):
        """ギアをDriveに設定し、確認されるまで待機"""
        print("Setting gear to DRIVE via ROS2...")
        
        # まず現在の状態を確認
        if self.latest_gear == 2:
            print("Gear is already DRIVE.")
            return

        gear_msg = GearCommand()
        gear_msg.stamp = self.get_clock().now().to_msg()
        gear_msg.command = 2  # DRIVE
        
        start_time = time.time()
        timeout = 5.0
        
        while time.time() - start_time < timeout:
            # コマンド送信
            self.gear_pub.publish(gear_msg)
            
            # ステータス確認
            if self.latest_gear == 2:
                print("Gear successfully changed to DRIVE.")
                return
                
            time.sleep(0.1)
            
        print("Warning: Gear change timed out, but proceeding...")
        
    def publish_cmd_vel(self, linear_x, angular_z):
        """速度指令を送信"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        
    def stop_vehicle(self):
        """車両停止"""
        self.publish_cmd_vel(0.0, 0.0)

