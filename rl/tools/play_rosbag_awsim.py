#!/usr/bin/env python3
"""
AWSIMでrosbagを再生するスクリプト

AWSIMではギアをDRIVEにしないと車椅子が動かないため、
先にギアコマンドを送信してからrosbagを再生する。

使い方:
  python3 scripts/play_rosbag_awsim.py /path/to/rosbag_dir
  python3 scripts/play_rosbag_awsim.py /path/to/rosbag_dir --rate 0.5
"""

import sys
import time
import subprocess
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def send_gear_drive():
    """AWSIMにギアコマンド(DRIVE)を送信する。"""
    rclpy.init()
    node = Node('rosbag_gear_sender')

    from autoware_vehicle_msgs.msg import GearCommand

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    pub = node.create_publisher(GearCommand, '/control/command/gear_cmd', qos)

    # ギアコマンドを数回送信して確実に届くようにする
    print("[1/2] ギアをDRIVEに設定中...")
    for i in range(10):
        msg = GearCommand()
        msg.command = 2  # DRIVE=2
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("  ✓ ギアコマンド送信完了")

    node.destroy_node()
    rclpy.shutdown()


def play_rosbag(bag_path: str, rate: float = 1.0):
    """rosbagを再生する。"""
    print(f"\n[2/2] rosbag再生開始 (rate={rate}x)...")
    print(f"  ファイル: {bag_path}")
    print(f"  Ctrl+C で停止\n")

    cmd = ['ros2', 'bag', 'play', bag_path]
    if rate != 1.0:
        cmd.extend(['--rate', str(rate)])

    try:
        proc = subprocess.run(cmd)
        if proc.returncode == 0:
            print("\n✓ rosbag再生完了")
        else:
            print(f"\n✗ rosbag再生失敗 (code={proc.returncode})")
    except KeyboardInterrupt:
        print("\n\n中断しました")


def main():
    parser = argparse.ArgumentParser(
        description='AWSIMでrosbagを再生 (ギアDRIVE自動設定)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python3 scripts/play_rosbag_awsim.py logs/real_whill_rosbags/inference_20260217_113826
  python3 scripts/play_rosbag_awsim.py logs/real_whill_rosbags/inference_20260217_113826 --rate 0.5
        """
    )
    parser.add_argument('bag_path', type=str, help='rosbagディレクトリのパス')
    parser.add_argument('--rate', type=float, default=1.0, help='再生速度 (default: 1.0)')
    args = parser.parse_args()

    bag_path = Path(args.bag_path).expanduser()
    if not bag_path.exists():
        print(f"エラー: {bag_path} が見つかりません")
        sys.exit(1)

    print("=" * 60)
    print("AWSIM rosbag Replay")
    print("=" * 60)

    # Step 1: ギアをDRIVEに
    send_gear_drive()

    # 少し待ってからrosbag再生
    time.sleep(0.5)

    # Step 2: rosbag再生
    play_rosbag(str(bag_path), rate=args.rate)


if __name__ == "__main__":
    main()
