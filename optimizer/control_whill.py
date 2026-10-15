#!/usr/bin/env python3
"""
control_whill.py

Orchestrate WHILL odometry and wheel speed recording for a short experiment.

Requirements:
 - Run this script in a shell where ROS2 and relevant workspaces are sourced
   (see README.md).

Behavior:
 - Optionally start WHILL driver nodes (via ros2 run) in background
 - Call WHILL power-on service
 - Publish cmd_vel (geometry_msgs/Twist) or joy (sensor_msgs/Joy) for 'duration' seconds
 - Subscribe to odometry and wheel speed topics and record incoming messages with timestamps
 - Save recorded data to CSV with format: ts,x,y,yaw,vx,vy,wz,wheel_left,wheel_right,qx,qy,qz,qw
 - Produce a PNG plot of the trajectory

"""
import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from ros2_whill_interfaces.msg import WhillModelC
except Exception as e:
    print("ERROR: could not import rclpy or message types. Ensure you run this after sourcing ROS2 and your workspace.")
    print(e)
    sys.exit(1)

class DataRecorder(Node):
    """オドメトリとWHILL車輪速度を同時に記録"""
    def __init__(self, odom_topic, wheel_topic, wheel_radius=0.135):
        super().__init__('whill_data_recorder')
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        
        # WHILL WhillModelC subscriber for wheel speeds
        self.wheel_sub = self.create_subscription(
            WhillModelC, wheel_topic, self.wheel_callback, 10)
        
        self.lock = threading.Lock()
        self.odom_records = []
        self.latest_wheel_left = 0.0
        self.latest_wheel_right = 0.0
        self.latest_raw_left = 0.0
        self.latest_raw_right = 0.0
        self.wheel_msg_count = 0
        self.wheel_radius = wheel_radius  # 車輪半径 (m)
        
        self.get_logger().info(f'DataRecorder initialized (wheel_radius={wheel_radius}m)')

    def wheel_callback(self, msg: WhillModelC):
        """車輪速度を記録 (km/h → m/s に変換)"""
        with self.lock:
            # WHILL内部の wheel speed は km/h で送信される
            # 注意: WhillModelCでは左車輪速度が負の値で来るため、符号を反転
            # store raw speeds (km/h)
            self.latest_raw_left = msg.left_motor_speed
            self.latest_raw_right = msg.right_motor_speed

            # convert km/h to m/s (km/h / 3.6 = m/s)
            self.latest_wheel_left =  msg.left_motor_speed / 3.6
            self.latest_wheel_right = -(msg.right_motor_speed / 3.6) # 符号反転
            self.wheel_msg_count += 1
            # デバッグ: 最初の数回だけログ出力
            if self.wheel_msg_count <= 10:
                self.get_logger().info(
                    f'Wheel msg #{self.wheel_msg_count}: raw left={msg.left_motor_speed:.3f} km/h, raw right={msg.right_motor_speed:.3f} km/h -> left_m/s={self.latest_wheel_left:.3f}, right_m/s={self.latest_wheel_right:.3f}'
                )
    
    def odom_callback(self, msg: Odometry):
        # ROS時刻を使用（rosbagと同じタイムスタンプ）
        now_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # クォータニオンからyaw角を計算
        q = msg.pose.pose.orientation
        try:
            import math
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        except Exception:
            yaw = 0.0
        
        with self.lock:
            rec = {
                'ts': now_ros,                          # タイムスタンプ (ROS時刻、rosbagと同期)
                # 以下、Odometryメッセージから直接取得
                'x': msg.pose.pose.position.x,          # X座標 (odomから)
                'y': msg.pose.pose.position.y,          # Y座標 (odomから)
                'yaw': yaw,                             # Yaw角 (クォータニオンから計算)
                'vx': msg.twist.twist.linear.x,         # X方向速度 (odomから)
                'vy': msg.twist.twist.linear.y,         # Y方向速度 (odomから)
                'wz': msg.twist.twist.angular.z,        # 角速度 (odomから)
                # 以下、ModelCr2Stateメッセージから取得した車輪速度
                'wheel_left': self.latest_wheel_left,   # 左車輪速度 (m/s, stateから変換)
                'wheel_right': self.latest_wheel_right, # 右車輪速度 (m/s, stateから変換)
                # 生データ (km/h)
                'raw_left_motor_speed': self.latest_raw_left,
                'raw_right_motor_speed': self.latest_raw_right,
                # クォータニオン（参考用に保存）
                'qx': q.x,
                'qy': q.y,
                'qz': q.z,
                'qw': q.w,
            }
            self.odom_records.append(rec)


def start_process_in_bash(cmd: str, env: dict = None):
    """Start a process via bash -lc so sourcing can be used in command string.
    Returns subprocess.Popen object.
    """
    # Using preexec_fn=os.setsid to create new process group for later termination
    return subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, env=env)


def safe_chmod(path: str, mode: int = 0o666):
    try:
        os.chmod(path, mode)
        print(f'Set permissions on {path} to {oct(mode)}')
    except PermissionError:
        print(f'PermissionError: could not chmod {path}. You may need sudo. Skipping.')
    except FileNotFoundError:
        print(f'File {path} not found. Skipping chmod.')


def call_whill_power_on():
    # Use ros2 service call to set power. We rely on environment of the running shell.
    print('Calling WHILL power-on service...')
    try:
        subprocess.run(['ros2', 'service', 'call', '/whill/set_power_srv', 'ros2_whill_interfaces/srv/SetPower', '{p0: 1}'], check=True)

        print('Power on service called.')
    except subprocess.CalledProcessError as e:
        print('Failed to call WHILL power service. Is the WHILL node running?')
        print(e)


def publish_motion_and_record(node: DataRecorder, duration: float, speed: float, 
                              angular: float = 0.0, radius: float = 2.0, 
                              side_length: float = 2.0,
                              pattern: str = 'straight', pub_rate: float = 12.0):
    """
    WHILLに速度指令を送信してIMUデータを記録
    
    Args:
        node: DataRecorderノード
        duration: 走行時間(秒)
        speed: 基本速度(m/s)
        angular: 基本角速度(rad/s) - straightとturnで使用
        radius: 円の半径(m) - circleパターンで使用
        side_length: 正方形の1辺の長さ(m) - squareパターンで使用
        pattern: 軌道パターン ('straight', 'turn', 'circle', 's_curve', 'square')
        pub_rate: 指令送信レート(Hz)
    """
    # geometry_msgs/Twist メッセージを使用
    pub = node.create_publisher(Twist, '/whill/controller/cmd_vel', 10)
    twist_msg = Twist()
    zero_twist = Twist()
    print(f'Publishing cmd_vel commands at {pub_rate} Hz for {duration} seconds')

    start = time.time()
    rate_sleep = 1.0 / pub_rate
    
    # パターンごとの説明を表示
    pattern_descriptions = {
        'straight': f'直進 (speed={speed} m/s)',
        'turn': f'その場旋回 (angular={angular} rad/s)',
        'circle': f'円運動 (speed={speed} m/s, radius={radius} m)',
        's_curve': f'S字カーブ (speed={speed} m/s)',
        'square': f'正方形 (speed={speed} m/s, side={side_length} m)'
    }
    
    print(f'Pattern: {pattern_descriptions.get(pattern, pattern)}')
    
    # squareパターンの場合の準備
    if pattern == 'square':
        # 1辺を進む時間と90度旋回する時間を計算
        time_per_side = side_length / speed if speed > 0 else 0
        turn_angular_speed = 0.785  # 90度旋回の角速度 (rad/s) ≈ π/4
        turn_time = (np.pi / 2) / turn_angular_speed  # 90度 = π/2 rad
        total_cycle_time = 4 * (time_per_side + turn_time)  # 4辺 + 4回の旋回
        print(f'  1辺の走行時間: {time_per_side:.2f}s, 旋回時間: {turn_time:.2f}s')
        print(f'  1周の時間: {total_cycle_time:.2f}s')
        print(f'  推奨duration: {int(total_cycle_time) + 2}秒以上')
    
    while True:
        now = time.time()
        elapsed = now - start
        if elapsed >= duration:
            break
        
        # 進捗表示（1秒ごと）
        if int(elapsed) % 5 == 0 and elapsed > 0 and (elapsed - int(elapsed)) < rate_sleep:
            print(f'  Progress: {elapsed:.1f}s / {duration:.1f}s')
        
        # パターンに応じて速度指令を計算
        if pattern == 'straight':
            # 直進
            linear_x = speed
            angular_z = 0.0
            
        elif pattern == 'turn':
            # 旋回（その場回転）
            linear_x = 0.0  # その場回転なので前進速度は0
            angular_z = angular
            
        elif pattern == 'circle':
            # 円運動
            linear_x = speed
            # angular引数が指定されていればそれを使用、なければ半径から計算
            if angular != 0.0:
                angular_z = angular
            else:
                # v = r * ω より ω = v / r
                angular_z = (speed / radius) if radius > 0 else 0.0
            
        elif pattern == 's_curve':
            # S字カーブ（時間に応じて角速度を変化）
            linear_x = speed
            period = duration / 2.0
            # サイン波で左右に曲がる
            angular_z = 0.5 * np.sin(2.0 * np.pi * elapsed / period)
            
        elif pattern == 'square':
            # 正方形（1辺ずつ直進→その場で90度回転を4回繰り返す）
            time_per_side = side_length / speed if speed > 0 else 0
            turn_angular_speed = 0.785  # 90度旋回の角速度 (rad/s) ≈ π/4
            turn_time = (np.pi / 2) / turn_angular_speed  # 90度旋回時間
            segment_time = time_per_side + turn_time  # 1辺+1旋回の時間
            
            # 現在どのセグメントにいるか計算（0-3: 4辺）
            cycle_time = elapsed % (4 * segment_time)
            segment_index = int(cycle_time / segment_time)
            time_in_segment = cycle_time % segment_time
            
            if time_in_segment < time_per_side:
                # 直進フェーズ（1辺分）
                linear_x = speed
                angular_z = 0.0
            else:
                # 旋回フェーズ（その場で90度左旋回）
                linear_x = 0.0
                angular_z = turn_angular_speed
            
        else:
            # デフォルトは直進
            linear_x = speed
            angular_z = 0.0
        
        # Twistメッセージで送信
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        pub.publish(twist_msg)
        
        # sleep to maintain loop rate
        time.sleep(rate_sleep)
        
        # spin once to allow callbacks after sleep
        rclpy.spin_once(node, timeout_sec=0.0)

    # 停止コマンドを送信し、その後3秒間停止コマンドを送り続ける
    print('Sending stop command and recording deceleration for 3 seconds...')
    stop_duration = 3.0  # 停止後の記録時間
    stop_start = time.time()
    
    while True:
        now = time.time()
        stop_elapsed = now - stop_start
        if stop_elapsed >= stop_duration:
            break
        
        # 停止コマンドを継続的に送信
        pub.publish(zero_twist)
        
        # sleep to maintain loop rate
        time.sleep(rate_sleep)
        
        # spin once to allow callbacks after sleep
        rclpy.spin_once(node, timeout_sec=0.0)
    
    # 最終的な停止コマンド
    pub.publish(zero_twist)
    rclpy.spin_once(node, timeout_sec=0.1)
    print('Motion finished.')


def save_csv(records, filename='whill_odom_data.csv'):
    if not records:
        print('No records to save.')
        return
    # CSVヘッダー順序を指定
    keys = ['ts', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'wheel_left', 'wheel_right', 'raw_left_motor_speed', 'raw_right_motor_speed', 'qx', 'qy', 'qz', 'qw']
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in records:
            writer.writerow(r)
    print(f'Saved {len(records)} records to {filename}')


def plot_odom(csv_file='whill_odom_data.csv', out_png='whill_odom_plot.png'):
    try:
        import pandas as pd
        import matplotlib.pyplot as plt
    except Exception as e:
        print('Plotting dependencies not found. Install requirements.txt (matplotlib,pandas)')
        print(e)
        return

    df = pd.read_csv(csv_file)
    if df.empty:
        print('CSV is empty, skip plot.')
        return

    plt.figure(figsize=(6,6))
    plt.plot(df['x'], df['y'], marker='o', linestyle='-', label='trajectory')
    # mark start and end points
    start_x, start_y = df['x'].iloc[0], df['y'].iloc[0]
    end_x, end_y = df['x'].iloc[-1], df['y'].iloc[-1]
    plt.scatter([start_x], [start_y], color='green', s=80, marker='*', label='start')
    plt.scatter([end_x], [end_y], color='red', s=80, marker='X', label='end')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Filtered odometry trajectory')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    # annotate start/end
    plt.annotate('start', xy=(start_x, start_y), xytext=(5, 5), textcoords='offset points', color='green')
    plt.annotate('end', xy=(end_x, end_y), xytext=(5, -10), textcoords='offset points', color='red')
    plt.savefig(out_png)
    print(f'Odom plot saved to {out_png}')


def terminate_process(proc: subprocess.Popen, name: str):
    if not proc:
        return
    try:
        print(f'Terminating {name} (pid {proc.pid})')
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
    except Exception as e:
        print(f'Error terminating {name}: {e}')


def main():
    parser = argparse.ArgumentParser(
        description='WHILL control with odometry recording',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Trajectory Patterns:
  straight : 直進 (デフォルト)
  turn     : 旋回 (一定速度+一定角速度)
  circle   : 円運動 (指定した半径の円を描く)
  s_curve  : S字カーブ (左右に蛇行)
  square   : 正方形 (直進→90度旋回を4回繰り返す)

Examples:
  # 直進
  python3 control_whill.py --duration 5 --speed 0.3 --pattern straight
  
  # 左旋回
  python3 control_whill.py --duration 5 --speed 0.3 --angular 0.3 --pattern turn
  
  # 円運動（半径2m、デフォルト）
  python3 control_whill.py --duration 10 --speed 0.3 --pattern circle
  
  # 小さい円（半径1m）
  python3 control_whill.py --duration 10 --speed 0.3 --radius 1.0 --pattern circle
  
  # 大きい円（半径3m）
  python3 control_whill.py --duration 15 --speed 0.3 --radius 3.0 --pattern circle
  
  # S字カーブ
  python3 control_whill.py --duration 10 --speed 0.3 --pattern s_curve
  
  # 正方形（1辺2m、約40秒で1周）
  python3 control_whill.py --duration 40 --speed 0.3 --side-length 2.0 --pattern square
  
  # 小さい正方形（1辺1m、約20秒で1周）
  python3 control_whill.py --duration 20 --speed 0.3 --side-length 1.0 --pattern square
        """
    )
    parser.add_argument('--duration', type=float, default=5.0, help='duration (s) to drive WHILL')
    parser.add_argument('--speed', type=float, default=0.3, help='forward speed (m/s)')
    parser.add_argument('--angular', type=float, default=0.0, help='angular velocity (rad/s) for turn pattern')
    parser.add_argument('--radius', type=float, default=2.0, help='circle radius (m) for circle pattern')
    parser.add_argument('--side-length', type=float, default=2.0, help='square side length (m) for square pattern')
    parser.add_argument('--pattern', type=str, default='straight',
                        choices=['straight', 'turn', 'circle', 's_curve', 'square'],
                        help='trajectory pattern')
    parser.add_argument('--odom-topic', type=str, default='/whill/odom', help='Odometry topic')
    parser.add_argument('--wheel-topic', type=str, default='/whill/modelc_state', help='WHILL WhillModelC topic for wheel speeds')
    parser.add_argument('--wheel-radius', type=float, default=0.135, help='wheel radius (m) for converting rad/s to m/s')
    parser.add_argument('--port', type=str, default='/dev/whill_serial', help='WHILL serial port')

    parser.add_argument('--no-start-nodes', action='store_true', help='assume nodes already started')
    parser.add_argument('--start-nodes', action='store_true', help='start whill nodes automatically')
    parser.add_argument('--record-bag', action='store_true', help='record rosbag during experiment')
    parser.add_argument('--bag-topics', type=str, default='/whill/odom /whill/modelc_state /whill/controller/cmd_vel /whill/states/batttery_state', 
                        help='topics to record in rosbag (space-separated)')
    args = parser.parse_args()

    # optional chmod
    safe_chmod(args.port)

    whill_publisher_proc = None
    whill_odom_proc = None
    bag_proc = None

    if args.start_nodes and not args.no_start_nodes:
        # Start WHILL publisher node (now handles both odometry and velocity control)
        whill_driver_cmd = f'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; exec ros2 run ros2_whill whill_modelc_publisher --ros-args -p serialport:={args.port}'
        print('Starting WHILL publisher node...')
        whill_publisher_proc = start_process_in_bash(whill_driver_cmd)
        
        print('Waiting 5 seconds for node to initialize (power on and joystick release included)...')
        time.sleep(5)  # Publisher needs time to initialize, power on, and release joystick
        print('WHILL node ready.')

    # Initialize ROS client library (this process must be run in ROS2 env)
    rclpy.init()
    node = DataRecorder(args.odom_topic, args.wheel_topic, args.wheel_radius)

    # Start rosbag recording if requested
    if args.record_bag:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f'whill_cmd_{args.pattern}_{timestamp}'
        bag_cmd = f'source /opt/ros/humble/setup.bash && exec ros2 bag record -o {bag_name} {args.bag_topics}'
        print(f'Starting rosbag recording: {bag_name}')
        bag_proc = start_process_in_bash(bag_cmd)
        time.sleep(1)  # rosbagが起動するまで待つ


    try:
        # レコードをクリアして実験開始
        with node.lock:
            node.odom_records.clear()
        
        publish_motion_and_record(node, duration=args.duration, speed=args.speed, 
                                 angular=args.angular, radius=args.radius, 
                                 side_length=args.side_length,
                                 pattern=args.pattern, pub_rate=12.0)

        # gather records
        with node.lock:
            odom_records = list(node.odom_records)

        # Save CSV files
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Odometry data with wheel speeds
        odom_csv = f'whill_odom_{args.pattern}_{timestamp}.csv'
        save_csv(odom_records, odom_csv)
        
        # Plot trajectory
        png_name = f'whill_odom_plot_{args.pattern}_{timestamp}.png'
        plot_odom(odom_csv, png_name)
        
        print('\n' + '='*60)
        print('データ収集完了')
        print('='*60)
        print(f'Odometry: {len(odom_records)} samples → {odom_csv}')
        print(f'Plot:     {png_name}')
        print('='*60)

    except KeyboardInterrupt:
        print('\n\n[!] Ctrl+C detected - Emergency stop!')
        print('Sending emergency stop command (velocity=0, angular=0)...')
        
        # 緊急停止: 速度0、角速度0のコマンドを送信
        try:
            emergency_pub = node.create_publisher(Twist, '/whill/controller/cmd_vel', 10)
            zero_twist = Twist()
            zero_twist.linear.x = 0.0
            zero_twist.angular.z = 0.0
            
            # 停止コマンドを複数回送信して確実に停止
            for _ in range(10):
                emergency_pub.publish(zero_twist)
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.05)
            
            print('Emergency stop command sent.')
        except Exception as e:
            print(f'Error during emergency stop: {e}')
        
        # 収集済みのデータを保存
        with node.lock:
            odom_records = list(node.odom_records)
        
        if len(odom_records) > 0:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            odom_csv = f'whill_odom_{args.pattern}_interrupted_{timestamp}.csv'
            save_csv(odom_records, odom_csv)
            print(f'Partial data saved to {odom_csv}')

    finally:
        # cleanup
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

        # terminate rosbag first to ensure data is saved
        if bag_proc:
            print('Stopping rosbag recording...')
            terminate_process(bag_proc, 'ros2 bag record')
            time.sleep(2)  # rosbagが安全に終了するまで待つ

        # terminate started processes
        if whill_publisher_proc:
            terminate_process(whill_publisher_proc, 'whill_modelc_publisher')
        if whill_odom_proc:
            terminate_process(whill_odom_proc, 'whill_odom_from_modelcr2')

    print('Experiment complete.')


if __name__ == '__main__':
    main()
