#!/usr/bin/env python3
"""
replay_bag_on_awsim.py

実機のROS2 bagファイルをAWSIMシミュレーション上で再生するスクリプト

使い方:
    python3 replay_bag_on_awsim.py <rosbag_directory>
    
    例:
    python3 replay_bag_on_awsim.py whill_cmd_straight_20251201_182712/

前提条件:
 - AWSIMが既に起動していること
 - ROS2環境がセットアップされていること
 - rosbag2がインストールされていること (sudo apt install ros-humble-rosbag2)

動作:
 - rosbagから /whill/controller/cmd_vel (geometry_msgs/Twist) を読み込み
 - /control/command/gear_cmd でギアをドライブに設定
 - AWSIMの /whill/controller/cmd_vel にメッセージを再送信
 - /ground_truth/odom からオドメトリを記録
 - CSVとプロットを生成

出力:
 - replay_<bagname>_<timestamp>.csv - オドメトリデータ
 - replay_<bagname>_<timestamp>.png - 軌道プロット
"""

import argparse
import csv
import os
import sys
import threading
import numpy as np
import subprocess
import signal
from datetime import datetime
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from autoware_vehicle_msgs.msg import GearCommand, GearReport
    from std_msgs.msg import Float32MultiArray
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message
except Exception as e:
    print("ERROR: could not import required modules.")
    print(e)
    sys.exit(1)


class BagReplayer(Node):
    """rosbagを再生してAWSIMにメッセージを送信しながらオドメトリを記録"""
    def __init__(self, odom_topic='/ground_truth/odom', cmd_vel_topic='/whill/controller/cmd_vel'):
        super().__init__('bag_replayer')
        
        # QoS設定: TRANSIENT_LOCAL (AWSIMの要求に合わせる)
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Cmd Vel publisher (AWSIM宛)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos_profile)
        
        # Odometry subscriber (Ground Truth)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        
        # Wheel velocities subscriber (AWSIM sim data)
        self.wheel_vel_sub = self.create_subscription(
            Float32MultiArray, '/whill/wheel_velocities_sim', self.wheel_vel_callback, 10)
        
        self.lock = threading.Lock()
        self.odom_records = []
        
        # rosbag recording
        self.recording_process = None
        self.is_recording = False
        self.output_bag_dir = None
        
        # 最新のホイール速度データ（実測値と目標値）
        self.latest_wheel_data = {
            'wheel_l': 0.0,        # data[0]: 左後輪実測速度
            'wheel_r': 0.0,        # data[1]: 右後輪実測速度
            'wheel_l_target': 0.0, # data[2]: 左後輪目標速度
            'wheel_r_target': 0.0, # data[3]: 右後輪目標速度
        }
        
        # 最新のcmd_vel（指令値）
        self.latest_cmd_vel = {
            'cmd_v': 0.0,
            'cmd_w': 0.0,
        }
        
        print(f'Publisher ready: {cmd_vel_topic}')
        print(f'Subscriber ready: {odom_topic}')
        print(f'Subscriber ready: /whill/wheel_velocities_sim')

    def wheel_vel_callback(self, msg: Float32MultiArray):
        """ホイール速度コールバック"""
        if len(msg.data) >= 4:
            with self.lock:
                self.latest_wheel_data['wheel_l'] = msg.data[0]        # 左後輪実測速度
                self.latest_wheel_data['wheel_r'] = msg.data[1]        # 右後輪実測速度
                self.latest_wheel_data['wheel_l_target'] = msg.data[2] # 左後輪目標速度
                self.latest_wheel_data['wheel_r_target'] = msg.data[3] # 右後輪目標速度

    def odom_callback(self, msg: Odometry):
        """オドメトリコールバック"""
        stamp = msg.header.stamp
        ts = stamp.sec + stamp.nanosec * 1e-9
        
        q = msg.pose.pose.orientation
        
        # ヨー角を計算
        import math
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )
        
        # 速度
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        
        # 合成速度
        v = math.sqrt(vx**2 + vy**2)
        
        # 最新のホイール速度と指令値を取得
        with self.lock:
            wheel_l = self.latest_wheel_data['wheel_l']
            wheel_r = self.latest_wheel_data['wheel_r']
            wheel_l_target = self.latest_wheel_data['wheel_l_target']
            wheel_r_target = self.latest_wheel_data['wheel_r_target']
            cmd_v = self.latest_cmd_vel['cmd_v']
            cmd_w = self.latest_cmd_vel['cmd_w']
        
        rec = {
            'ts': ts,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw,
            'v': v,
            'w': wz,
            'wheel_l': wheel_l,
            'wheel_r': wheel_r,
            'cmd_v': cmd_v,
            'cmd_w': cmd_w,
            'wheel_l_target': wheel_l_target,
            'wheel_r_target': wheel_r_target,
            'qx': q.x,
            'qy': q.y,
            'qz': q.z,
            'qw': q.w,
        }
        
        with self.lock:
            self.odom_records.append(rec)
    
    def start_bag_recording(self, output_dir):
        """rosbag記録を開始"""
        if self.is_recording:
            self.get_logger().warn('Recording already in progress')
            return False
        
        self.output_bag_dir = output_dir
        
        # Topics to record from AWSIM
        topics_to_record = [
            '/ground_truth/odom',
            '/whill/wheel_velocities_sim',
            '/whill/controller/cmd_vel'
        ]
        
        # Build ros2 bag record command
        record_cmd = ['ros2', 'bag', 'record']
        record_cmd.extend(topics_to_record)
        record_cmd.extend(['-o', self.output_bag_dir])
        
        print(f'\nStarting bag recording: {" ".join(record_cmd)}')
        
        try:
            self.recording_process = subprocess.Popen(
                record_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.is_recording = True
            print(f'✓ Bag recording started: {self.output_bag_dir}')
            return True
            
        except Exception as e:
            print(f'ERROR: Failed to start bag recording: {e}')
            return False
    
    def stop_bag_recording(self):
        """rosbag記録を停止"""
        if not self.is_recording:
            return
        
        print('\nStopping bag recording...')
        
        if self.recording_process:
            # Send SIGINT to ros2 bag record
            self.recording_process.send_signal(signal.SIGINT)
            
            # Wait for process to finish
            try:
                self.recording_process.wait(timeout=5)
                print(f'✓ Bag recording stopped: {self.output_bag_dir}')
            except subprocess.TimeoutExpired:
                print('⚠ WARNING: Recording process did not stop, forcing termination')
                self.recording_process.kill()
            
            self.recording_process = None
        
        self.is_recording = False


def set_gear_to_drive(node):
    """ギアをドライブに設定"""
    print('Setting gear to DRIVE...')
    
    qos_profile = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE
    )
    
    gear_pub = node.create_publisher(GearCommand, '/control/command/gear_cmd', qos_profile)
    
    import time
    time.sleep(1.0)
    
    gear_msg = GearCommand()
    gear_msg.stamp.sec = 0
    gear_msg.stamp.nanosec = 0
    gear_msg.command = 2  # DRIVE
    
    for _ in range(10):
        gear_pub.publish(gear_msg)
        time.sleep(0.1)
    
    print('Gear command sent (DRIVE)')
    time.sleep(1.0)
    
    # Verify
    gear_status_received = {'value': None}
    
    def gear_status_callback(msg):
        gear_status_received['value'] = msg.report
    
    gear_sub = node.create_subscription(
        GearReport, '/vehicle/status/gear_status', gear_status_callback, 10)
    
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if gear_status_received['value'] == 2:
            print(f'✓ Gear successfully changed to DRIVE')
            break
    else:
        if gear_status_received['value'] is not None:
            print(f'⚠ WARNING: Gear status is {gear_status_received["value"]} (expected 2=DRIVE)')
    
    node.destroy_subscription(gear_sub)
    time.sleep(0.5)


def read_bag_messages(bag_path, topic_name='/whill/controller/cmd_vel'):
    """rosbagからメッセージを読み込む"""
    print(f'\nReading bag: {bag_path}')
    print(f'Target topic: {topic_name}')
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # トピックとメッセージタイプのマッピング
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f'ERROR: Topic {topic_name} not found in bag file!')
        print(f'Available topics: {list(type_map.keys())}')
        return []
    
    messages = []
    count = 0
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == topic_name:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            messages.append((timestamp, msg))
            count += 1
    
    print(f'Loaded {count} messages from {topic_name}')
    
    if count > 0:
        duration = (messages[-1][0] - messages[0][0]) / 1e9
        print(f'Duration: {duration:.2f} seconds')
        print(f'Frequency: {count / duration:.1f} Hz')
    
    return messages


def replay_messages(node, messages, rate_multiplier=1.0):
    """メッセージを再生"""
    if not messages:
        print('No messages to replay!')
        return
    
    print(f'\n=== Starting Replay ===')
    print(f'Rate multiplier: {rate_multiplier}x')
    print(f'Publishing to: /whill/controller/cmd_vel')
    
    import time
    
    # Clear existing records
    with node.lock:
        node.odom_records.clear()
    
    start_time = time.time()
    first_timestamp = messages[0][0]
    
    for i, (timestamp, msg) in enumerate(messages):
        # Calculate when this message should be published
        relative_time = (timestamp - first_timestamp) / 1e9  # nanoseconds to seconds
        target_time = start_time + (relative_time / rate_multiplier)
        
        # Wait until target time
        current_time = time.time()
        sleep_time = target_time - current_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        # Publish message
        node.cmd_vel_pub.publish(msg)
        
        # 指令値を保存
        with node.lock:
            node.latest_cmd_vel['cmd_v'] = msg.linear.x
            node.latest_cmd_vel['cmd_w'] = msg.angular.z
        
        # Allow ROS to process callbacks
        rclpy.spin_once(node, timeout_sec=0.001)
        
        # Progress report
        if i % 100 == 0 or i == len(messages) - 1:
            elapsed = time.time() - start_time
            progress = (i + 1) / len(messages) * 100
            print(f'  Progress: {progress:.1f}% ({i+1}/{len(messages)}) - Elapsed: {elapsed:.1f}s')
    
    print('\n=== Replay Complete ===')
    
    # Send zero velocity to stop
    print('Stopping vehicle...')
    zero_twist = Twist()
    for _ in range(10):
        node.cmd_vel_pub.publish(zero_twist)
        time.sleep(0.05)


def save_odom_csv(records, filename):
    """オドメトリデータをCSVに保存（車両ローカル座標系、新フォーマット）"""
    if not records:
        print('No odom records to save.')
        return
    
    x_offset = records[0]['x']
    y_offset = records[0]['y']
    
    q0 = records[0]
    import math
    initial_yaw = math.atan2(
        2.0 * (q0['qw'] * q0['qz'] + q0['qx'] * q0['qy']),
        1.0 - 2.0 * (q0['qy']**2 + q0['qz']**2)
    )
    
    print(f'\nApplying offset: x={x_offset:.3f}, y={y_offset:.3f}')
    print(f'Initial yaw: {math.degrees(initial_yaw):.2f}° → rotating to vehicle-centric frame')
    
    cos_yaw = math.cos(-initial_yaw)
    sin_yaw = math.sin(-initial_yaw)
    
    # 新フォーマット: time, x, y, yaw, v, w, wheel_l, wheel_r, qx, qy, qz, qw, wheel_l_target, wheel_r_target
    keys = ['time', 'x', 'y', 'yaw', 'v', 'w', 'wheel_l', 'wheel_r', 'qx', 'qy', 'qz', 'qw', 'wheel_l_target', 'wheel_r_target']
    
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in records:
            x_world = r['x'] - x_offset
            y_world = r['y'] - y_offset
            
            x_local = cos_yaw * x_world - sin_yaw * y_world
            y_local = sin_yaw * x_world + cos_yaw * y_world
            
            # ヨー角を相対角度に変換
            relative_yaw = r['yaw'] - initial_yaw
            
            # クォータニオンも回転
            qw_new = math.cos(relative_yaw / 2.0)
            qz_new = math.sin(relative_yaw / 2.0)
            
            r_adjusted = {
                'time': r['ts'],
                'x': x_local,
                'y': y_local,
                'yaw': relative_yaw,
                'v': r['v'],
                'w': r['w'],
                'wheel_l': r['wheel_l'],
                'wheel_r': r['wheel_r'],
                'qx': 0.0,
                'qy': 0.0,
                'qz': qz_new,
                'qw': qw_new,
                'wheel_l_target': r['wheel_l_target'],
                'wheel_r_target': r['wheel_r_target'],
            }
            writer.writerow(r_adjusted)
    
    print(f'Saved {len(records)} odom records to {filename}')
    print(f'  Format: time, x, y, yaw, v, w, wheel_l, wheel_r, qx, qy, qz, qw, wheel_l_target, wheel_r_target')


def plot_odom(csv_file, out_png):
    """オドメトリデータをプロット"""
    try:
        import pandas as pd
        import matplotlib.pyplot as plt
    except Exception as e:
        print('Plotting dependencies not found.')
        print(e)
        return

    df = pd.read_csv(csv_file)
    if df.empty:
        print('CSV is empty, skip plot.')
        return

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    
    t = (df['time'] - df['time'].iloc[0]).values
    
    # 1. XY軌道
    ax1 = axes[0, 0]
    ax1.plot(df['x'].values, df['y'].values, marker='o', markersize=2, linestyle='-', linewidth=1.5, label='trajectory')
    start_x, start_y = df['x'].iloc[0], df['y'].iloc[0]
    end_x, end_y = df['x'].iloc[-1], df['y'].iloc[-1]
    ax1.scatter([start_x], [start_y], color='green', s=100, marker='*', label='start', zorder=5)
    ax1.scatter([end_x], [end_y], color='red', s=100, marker='X', label='end', zorder=5)
    ax1.set_xlabel('X Position (m) - Forward', fontsize=11)
    ax1.set_ylabel('Y Position (m) - Left', fontsize=11)
    ax1.set_title('Replayed Trajectory (Vehicle-Centric)', fontsize=13, fontweight='bold')
    ax1.legend()
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # 2. X位置
    ax2 = axes[0, 1]
    ax2.plot(t, df['x'].values, linewidth=2, color='blue')
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('X Position (m)', fontsize=11)
    ax2.set_title('X Position (Forward)', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # 3. Y位置
    ax3 = axes[0, 2]
    ax3.plot(t, df['y'].values, linewidth=2, color='orange')
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Y Position (m)', fontsize=11)
    ax3.set_title('Y Position (Left)', fontsize=13, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # 4. 速度
    ax4 = axes[1, 0]
    ax4.plot(t, df['v'].values, linewidth=2, color='green', label='v (actual)')
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.set_ylabel('Speed (m/s)', fontsize=11)
    ax4.set_title('Linear Velocity', fontsize=13, fontweight='bold')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. 角速度
    ax5 = axes[1, 1]
    ax5.plot(t, df['w'].values, linewidth=2, color='purple', label='w (actual)')
    ax5.set_xlabel('Time (s)', fontsize=11)
    ax5.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    ax5.set_title('Angular Velocity', fontsize=13, fontweight='bold')
    ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # 6. ホイール速度（実測 vs 目標）
    ax6 = axes[1, 2]
    ax6.plot(t, df['wheel_l'].values, linewidth=2, color='blue', label='wheel_l (actual)')
    ax6.plot(t, df['wheel_r'].values, linewidth=2, color='red', label='wheel_r (actual)')
    ax6.plot(t, df['wheel_l_target'].values, linewidth=1.5, color='cyan', linestyle='--', label='wheel_l_target')
    ax6.plot(t, df['wheel_r_target'].values, linewidth=1.5, color='orange', linestyle='--', label='wheel_r_target')
    ax6.set_xlabel('Time (s)', fontsize=11)
    ax6.set_ylabel('Wheel Velocity (m/s)', fontsize=11)
    ax6.set_title('Wheel Velocities', fontsize=13, fontweight='bold')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f'Plot saved to {out_png}')


def main():
    parser = argparse.ArgumentParser(
        description='Replay ROS2 bag on AWSIM',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 replay_bag_on_awsim.py whill_cmd_straight_20251201_182712/
  python3 replay_bag_on_awsim.py whill_cmd_straight_20251201_182712/ --rate 0.5
  python3 replay_bag_on_awsim.py whill_cmd_straight_20251201_182712/ --save-bag
        """
    )
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('--topic', default='/whill/controller/cmd_vel',
                        help='Topic name to replay (default: /whill/controller/cmd_vel)')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='Playback rate multiplier (default: 1.0, 0.5=half speed, 2.0=double speed)')
    parser.add_argument('--odom-topic', default='/ground_truth/odom',
                        help='Odometry topic to record (default: /ground_truth/odom)')
    parser.add_argument('--save-bag', action='store_true',
                        help='Save AWSIM output as ROS2 bag file')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_path):
        print(f'ERROR: Bag path does not exist: {args.bag_path}')
        sys.exit(1)
    
    # Initialize ROS
    rclpy.init()
    node = BagReplayer(odom_topic=args.odom_topic, cmd_vel_topic=args.topic)
    
    try:
        # Set gear to DRIVE
        set_gear_to_drive(node)
        
        import time
        print('Waiting 2 seconds for system to stabilize...')
        time.sleep(2.0)
        
        # Extract pattern name for output files
        bag_name = Path(args.bag_path).stem
        parts = bag_name.split('_')
        if len(parts) >= 5 and parts[0] == 'whill' and parts[1] == 'cmd':
            pattern_parts = []
            for i in range(2, len(parts)):
                if len(parts[i]) == 8 and parts[i].isdigit():
                    break
                pattern_parts.append(parts[i])
            pattern_name = '_'.join(pattern_parts) if pattern_parts else 'unknown'
        else:
            pattern_name = 'unknown'
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Start bag recording if requested
        if args.save_bag:
            bag_output_dir = f'awsim_replay_{pattern_name}_{timestamp}'
            print(f'\n{"="*70}')
            print('=== ROS2 Bag Recording Enabled ===')
            print(f'{"="*70}')
            print(f'Output bag directory: {bag_output_dir}')
            node.start_bag_recording(bag_output_dir)
            time.sleep(1.0)  # Wait for recording to initialize
        
        # Read bag messages
        messages = read_bag_messages(args.bag_path, args.topic)
        
        if not messages:
            print('No messages found to replay!')
            if args.save_bag:
                node.stop_bag_recording()
            return
        
        # Replay messages
        replay_messages(node, messages, rate_multiplier=args.rate)
        
        # Wait a bit for final messages to be recorded
        if args.save_bag:
            print('\nWaiting for final messages to be recorded...')
            time.sleep(2.0)
            node.stop_bag_recording()
        
        # Get records
        with node.lock:
            odom_records = list(node.odom_records)
        
        if not odom_records:
            print('WARNING: No odometry data was recorded!')
            return
        
        # Generate CSV and plot filenames
        csv_file = f'replay_whill_cmd_{pattern_name}_{timestamp}.csv'
        png_file = f'replay_whill_cmd_{pattern_name}_{timestamp}.png'
        
        # Save data
        save_odom_csv(odom_records, csv_file)
        plot_odom(csv_file, png_file)
        
        print('\n' + '='*70)
        print('=== Replay Complete ===')
        print('='*70)
        print(f'CSV:  {csv_file}')
        print(f'Plot: {png_file}')
        if args.save_bag:
            print(f'Bag:  {bag_output_dir}/')
            print('\nBag information:')
            subprocess.run(['ros2', 'bag', 'info', bag_output_dir])
        print('='*70)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
