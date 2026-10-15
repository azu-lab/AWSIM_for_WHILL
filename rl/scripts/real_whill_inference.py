#!/usr/bin/env python3
"""
Real WHILL Inference Node with DreamerV3

実機WHILLに対してAWSIMで学習したDreamerV3モデルを使ってリアルタイム推論を行い、
Twistメッセージで速度指令を送信するROS 2ノード。

構成:
  実機PC (カメラ配信) → [ROS 2 DDS] → 推論PC (このスクリプト) → [cmd_vel] → 実機PC

使い方:
  python scripts/real_whill_inference.py \
    --logdir /path/to/dreamerv3_logs/obstacle_avoidance \
    --checkpoint 20000
"""

import os
import sys
import argparse
import csv
import math
import signal
import time
import threading
from pathlib import Path
from typing import Optional

import numpy as np
import cv2
import subprocess
from datetime import datetime

# Add paths
# rl/ (awsim_dreamerv3) と third_party/dreamerv3 を import できるようにする
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'rl'))
sys.path.insert(0, str(REPO_ROOT / 'third_party' / 'dreamerv3'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class RealWhillInferenceNode(Node):
    """
    実機WHILLへのDreamerV3リアルタイム推論ノード。

    カメラ画像を受信し、学習済みDreamerV3モデルで推論し、
    Twistメッセージとして速度指令を送信する。
    """

    def __init__(
        self,
        agent,
        agent_state,
        camera_topic: str,
        cmd_vel_topic: str,
        max_linear_vel: float,
        max_angular_vel: float,
        control_hz: float,
        image_size: tuple = (64, 64),
        duration: Optional[float] = None,
        record: bool = False,
        record_dir: str = '',
        odom_topic: str = '/whill/odom',
        save_trajectory: bool = True,
    ):
        super().__init__('real_whill_inference')

        self._agent = agent
        self._agent_state = agent_state
        self._max_linear_vel = max_linear_vel
        self._max_angular_vel = max_angular_vel
        self._control_hz = control_hz
        self._image_size = image_size
        self._bridge = CvBridge()

        # State
        self._latest_image: Optional[np.ndarray] = None
        self._is_first_step = True
        self._step_count = 0
        self._running = True

        # Readiness check state
        self._phase = 'STANDBY'  # STANDBY -> READY -> RUNNING
        self._camera_ready = False
        self._subscriber_ready = False
        self._ready_since: Optional[float] = None
        self._countdown_seconds = 3.0  # 準備完了後のカウントダウン
        self._running_start_time: Optional[float] = None
        self._duration: Optional[float] = None  # 推論の最大実行時間（秒）
        self._record_enabled = record
        self._record_dir = record_dir
        self._bag_process: Optional[subprocess.Popen] = None
        self._cmd_vel_topic = cmd_vel_topic
        self._save_trajectory = save_trajectory

        # Trajectory recording
        self._traj_lock = threading.Lock()
        self._traj_records = []  # list of dicts
        self._latest_cmd_linear = 0.0
        self._latest_cmd_angular = 0.0
        self._timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')

        # QoS for camera (BEST_EFFORT for sensor data)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS for cmd_vel (RELIABLE)
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: カメラ画像
        self._image_sub = self.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            sensor_qos
        )

        # Subscriber: オドメトリ（軌道記録用）
        if self._save_trajectory:
            self._odom_sub = self.create_subscription(
                Odometry,
                odom_topic,
                self._odom_callback,
                10
            )
            self.get_logger().info(f"  Odom topic:     {odom_topic} (trajectory recording ON)")

        # Publisher: 速度指令
        self._cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            cmd_qos
        )

        # スタンバイチェックタイマー (1Hz)
        self._standby_timer = self.create_timer(1.0, self._standby_check)

        # 制御タイマー (初期は無効化)
        timer_period = 1.0 / control_hz
        self._control_timer = self.create_timer(timer_period, self._control_loop)
        self._control_timer.cancel()  # まだ開始しない

        self.get_logger().info(f"=== Real WHILL Inference Node ===")
        self.get_logger().info(f"  Camera topic:   {camera_topic}")
        self.get_logger().info(f"  Cmd vel topic:  {cmd_vel_topic}")
        self.get_logger().info(f"  Max linear vel: {max_linear_vel} m/s")
        self.get_logger().info(f"  Max angular vel: {max_angular_vel} rad/s")
        self.get_logger().info(f"  Control freq:   {control_hz} Hz")
        self.get_logger().info(f"  Image size:     {image_size}")
        self._duration = duration
        self.get_logger().info(f"  Duration:       {duration}s" if duration else f"  Duration:       unlimited")
        self.get_logger().info(f"")
        self.get_logger().info(f"  [STANDBY] 実機の準備完了を待機中...")
        self.get_logger().info(f"    - カメラ画像の受信待ち")
        self.get_logger().info(f"    - cmd_velサブスクライバーの接続待ち")

    def _image_callback(self, msg: Image):
        """カメラ画像コールバック。"""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self._latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def _odom_callback(self, msg: Odometry):
        """オドメトリコールバック: 位置・姿勢・速度を記録。"""
        # ROS時刻を使用
        now_ros = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # クォータニオンからyaw角を計算
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        with self._traj_lock:
            rec = {
                'ts': now_ros,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': yaw,
                'vx': msg.twist.twist.linear.x,
                'vy': msg.twist.twist.linear.y,
                'wz': msg.twist.twist.angular.z,
                'cmd_linear': self._latest_cmd_linear,
                'cmd_angular': self._latest_cmd_angular,
            }
            self._traj_records.append(rec)

    def _standby_check(self):
        """スタンバイフェーズ: 実機側の準備が整ったか1秒ごとにチェック。"""
        if self._phase == 'RUNNING':
            return

        # 条件1: カメラ画像を受信しているか
        self._camera_ready = self._latest_image is not None

        # 条件2: cmd_velにサブスクライバーがいるか (実機がcmd_velを受信する準備ができている)
        self._subscriber_ready = self._cmd_vel_pub.get_subscription_count() > 0

        # 状態表示
        cam_icon = '✓' if self._camera_ready else '✗'
        sub_icon = '✓' if self._subscriber_ready else '✗'

        if self._camera_ready and self._subscriber_ready:
            # 両方準備完了
            if self._phase == 'STANDBY':
                self._phase = 'READY'
                self._ready_since = time.time()
                self.get_logger().info(
                    f"  [READY] 実機の準備が完了しました！"
                    f" {self._countdown_seconds:.0f}秒後に推論を開始します..."
                )
                # rosbag録画を開始（推論3秒前）
                self._start_recording()

            if self._phase == 'READY':
                elapsed = time.time() - self._ready_since
                remaining = self._countdown_seconds - elapsed
                if remaining > 0:
                    self.get_logger().info(f"  [READY] 推論開始まで {remaining:.0f}秒...")
                else:
                    # カウントダウン完了 → 推論開始
                    self._phase = 'RUNNING'
                    self._running_start_time = time.time()
                    self._standby_timer.cancel()
                    self._control_timer.reset()  # 制御タイマーを有効化
                    self.get_logger().info(f"")
                    self.get_logger().info(f"  ====================================")
                    self.get_logger().info(f"  [RUNNING] 推論を開始しました！")
                    if self._duration:
                        self.get_logger().info(f"  {self._duration:.0f}秒後に自動停止します")
                    self.get_logger().info(f"  ====================================")
                    self.get_logger().info(f"")
        else:
            # まだ準備できていない → STANDBYに戻す
            if self._phase == 'READY':
                self._phase = 'STANDBY'
                self._ready_since = None
                self.get_logger().warn("  [STANDBY] 接続が途切れました。再待機中...")

            self.get_logger().info(
                f"  [STANDBY] カメラ: {cam_icon}  cmd_velサブスクライバー: {sub_icon}"
            )

    def _control_loop(self):
        """制御ループ: 推論してcmd_velを送信。"""
        if not self._running:
            return

        # 時間制限チェック
        if self._duration and self._running_start_time:
            elapsed = time.time() - self._running_start_time
            if elapsed >= self._duration:
                self.get_logger().info(f"")
                self.get_logger().info(f"  ====================================")
                self.get_logger().info(f"  [FINISHED] {self._duration:.0f}秒経過 - 自動停止")
                self.get_logger().info(f"  ====================================")
                self.stop()
                # ノードをシャットダウン
                raise SystemExit(0)

        # 画像がまだ来ていない場合 (通常はここに来ないが安全のため)
        if self._latest_image is None:
            return

        # ----- 画像の前処理 -----
        image = self._latest_image.copy()

        # 64x64にリサイズ
        h, w = image.shape[:2]
        if (h, w) != self._image_size:
            image = cv2.resize(
                image,
                (self._image_size[1], self._image_size[0]),
                interpolation=cv2.INTER_AREA
            )
            if self._step_count == 0:
                self.get_logger().info(
                    f"Image resized: ({w}, {h}) -> ({self._image_size[1]}, {self._image_size[0]})"
                )
        elif self._step_count == 0:
            self.get_logger().info(f"Image already {self._image_size}, no resize needed")

        image = np.asarray(image, dtype=np.uint8)

        # ----- 観測データの構築 -----
        # DreamerV3が期待するobs_batchを構築
        # 学習時と同じキーを含める (is_collision, is_success等)
        obs_batch = {
            'image': image[None],                                    # (1, 64, 64, 3)
            'is_first': np.array([self._is_first_step]),             # (1,)
            'is_last': np.array([False]),                            # (1,)
            'is_terminal': np.array([False]),                        # (1,)
            'reward': np.array([0.0], dtype=np.float32),             # (1,)
            'is_collision': np.array([0.0], dtype=np.float32),       # (1,)
            'is_success': np.array([0.0], dtype=np.float32),         # (1,)
        }

        # ----- 推論 -----
        try:
            self._agent_state, action, _ = self._agent.policy(
                self._agent_state, obs_batch, mode='eval'
            )
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            self._send_stop()
            return

        # 最初のステップの後はis_firstをFalseに
        self._is_first_step = False

        # ----- アクションのスケーリング -----
        # action['action'] shape: (1, 2)  → [linear_vel_norm, angular_vel_norm]
        act = action['action'][0]  # Remove batch dim → (2,)
        linear_vel = float(act[0]) * self._max_linear_vel
        angular_vel = float(act[1]) * self._max_angular_vel

        # 前進のみにクランプ (安全のためバックなし)
        linear_vel = np.clip(linear_vel, 0.0, self._max_linear_vel)
        angular_vel = np.clip(angular_vel, -self._max_angular_vel, self._max_angular_vel)

        # ----- Twistメッセージの送信 -----
        msg = Twist()
        msg.linear.x = linear_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_vel
        self._cmd_vel_pub.publish(msg)

        # 送信したcmd_velを記録用に保持
        self._latest_cmd_linear = linear_vel
        self._latest_cmd_angular = angular_vel

        # ログ (10ステップごと)
        self._step_count += 1
        if self._step_count % 10 == 0:
            self.get_logger().info(
                f"Step {self._step_count}: "
                f"linear={linear_vel:.3f} m/s, angular={angular_vel:.3f} rad/s "
                f"(raw: [{act[0]:.3f}, {act[1]:.3f}])"
            )

    def _send_stop(self):
        """速度0を送信して停止。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._cmd_vel_pub.publish(msg)

    def stop(self):
        """推論を停止し、速度0を送信。軌道データを保存。"""
        self._running = False
        self._send_stop()
        self._stop_recording()
        self._save_trajectory_data()
        self.get_logger().info("Stopped: velocity set to 0")

    def _save_trajectory_data(self):
        """軌道データをCSVとPNGに保存。"""
        if not self._save_trajectory:
            return

        with self._traj_lock:
            records = list(self._traj_records)

        if not records:
            self.get_logger().warn("軌道データがありません。オドメトリが受信されていない可能性があります。")
            return

        # 保存先ディレクトリ
        save_dir = self._record_dir if self._record_dir else '.'
        os.makedirs(save_dir, exist_ok=True)

        ts = self._timestamp_str
        csv_path = os.path.join(save_dir, f'inference_trajectory_{ts}.csv')
        png_path = os.path.join(save_dir, f'inference_trajectory_{ts}.png')

        # --- CSV保存 ---
        keys = ['ts', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'cmd_linear', 'cmd_angular']
        try:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                for r in records:
                    writer.writerow(r)
            self.get_logger().info(f"  [TRAJECTORY] CSV保存: {csv_path} ({len(records)} samples)")
        except Exception as e:
            self.get_logger().error(f"  [TRAJECTORY] CSV保存エラー: {e}")
            return

        # --- PNGプロット ---
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            xs = [r['x'] for r in records]
            ys = [r['y'] for r in records]

            plt.figure(figsize=(8, 8))
            plt.plot(xs, ys, marker='o', markersize=2, linestyle='-', linewidth=1, label='trajectory')

            # start/end マーカー
            plt.scatter([xs[0]], [ys[0]], color='green', s=100, marker='*', zorder=5, label='start')
            plt.scatter([xs[-1]], [ys[-1]], color='red', s=100, marker='X', zorder=5, label='end')
            plt.annotate('start', xy=(xs[0], ys[0]), xytext=(5, 5),
                         textcoords='offset points', color='green', fontsize=10)
            plt.annotate('end', xy=(xs[-1], ys[-1]), xytext=(5, -10),
                         textcoords='offset points', color='red', fontsize=10)

            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title(f'Inference Trajectory ({ts})')
            plt.legend()
            plt.axis('equal')
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(png_path, dpi=150)
            plt.close()
            self.get_logger().info(f"  [TRAJECTORY] PNG保存: {png_path}")
        except ImportError:
            self.get_logger().warn("  [TRAJECTORY] matplotlib未インストール。PNGはスキップされました。")
        except Exception as e:
            self.get_logger().error(f"  [TRAJECTORY] PNG保存エラー: {e}")
    
    def _start_recording(self):
        """rosbag録画を開始する。"""
        if not self._record_enabled or self._bag_process is not None:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f"inference_{timestamp}"
        
        if self._record_dir:
            bag_path = f"{self._record_dir}/{bag_name}"
        else:
            bag_path = bag_name
        
        cmd = [
            'ros2', 'bag', 'record',
            self._cmd_vel_topic,
            '-o', bag_path,
        ]
        
        try:
            self._bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"  [RECORD] rosbag録画開始: {bag_path}")
        except Exception as e:
            self.get_logger().error(f"  [RECORD] 録画開始失敗: {e}")
            self._bag_process = None
    
    def _stop_recording(self):
        """rosbag録画を停止する。"""
        if self._bag_process is None:
            return
        
        try:
            self._bag_process.send_signal(signal.SIGINT)
            self._bag_process.wait(timeout=5)
            self.get_logger().info("  [RECORD] rosbag録画を保存しました")
        except subprocess.TimeoutExpired:
            self._bag_process.kill()
            self.get_logger().warn("  [RECORD] 録画プロセスを強制終了しました")
        except Exception as e:
            self.get_logger().error(f"  [RECORD] 録画停止エラー: {e}")
        finally:
            self._bag_process = None


def load_agent(logdir: Path, checkpoint_step: int):
    """
    DreamerV3 Agentをロードする。

    Args:
        logdir: 学習ログディレクトリ
        checkpoint_step: チェックポイントのステップ番号 (0 = finalを使用)

    Returns:
        (agent, initial_state) のタプル
    """
    import elements
    import ruamel.yaml as yaml
    import pickle
    from dreamerv3.agent import Agent

    # 設定ファイルのロード
    config_path = logdir / 'config.yaml'
    if config_path.exists():
        print(f"Loading config from {config_path}")
        config = elements.Config.load(config_path)
    else:
        print("Config not found in logdir, loading defaults...")
        dreamerv3_folder = REPO_ROOT / 'third_party' / 'dreamerv3' / 'dreamerv3'
        defaults_config = yaml.YAML(typ='safe').load(
            (dreamerv3_folder / 'configs.yaml').read_text()
        )
        config = elements.Config(defaults_config['defaults'])
        config = config.update({
            'logdir': str(logdir),
            'batch_size': 1,
            'batch_length': 64,
        })

    # 観測・行動空間の定義 (学習時と一致させる)
    obs_space = {
        'image': elements.Space(np.uint8, (64, 64, 3), 0, 255),
        'is_first': elements.Space(bool, ()),
        'is_last': elements.Space(bool, ()),
        'is_terminal': elements.Space(bool, ()),
        'reward': elements.Space(np.float32, ()),
        'is_collision': elements.Space(np.float32, ()),
        'is_success': elements.Space(np.float32, ()),
    }

    act_space = {
        'action': elements.Space(np.float32, (2,), -1.0, 1.0),
    }

    # Agent構築
    print("Creating DreamerV3 agent...")
    from awsim_dreamerv3.jax_compat import adjust_jax_config
    agent_config = elements.Config({
        **config.agent,
        'logdir': config.logdir,
        'seed': config.seed,
        'jax': adjust_jax_config(config.jax, None),
        'batch_size': 1,
        'batch_length': config.batch_length,
        'replay_context': config.replay_context,
        'report_length': config.report_length,
        'replica': 0,
        'replicas': 1,
    })
    agent = Agent(obs_space, act_space, agent_config)

    # チェックポイントのロード
    checkpoint_dir = logdir / 'agent'
    if checkpoint_step == 0:
        checkpoint_path = checkpoint_dir / 'checkpoint_final.pkl'
    else:
        checkpoint_path = checkpoint_dir / f'checkpoint_{checkpoint_step}.pkl'

    if not checkpoint_path.exists():
        # 利用可能なチェックポイントを一覧表示
        available = sorted(checkpoint_dir.glob('checkpoint_*.pkl'))
        available_str = ', '.join([p.stem for p in available])
        raise FileNotFoundError(
            f"Checkpoint not found: {checkpoint_path}\n"
            f"Available checkpoints: {available_str}"
        )

    print(f"Loading checkpoint: {checkpoint_path}")
    with open(str(checkpoint_path), 'rb') as f:
        data = pickle.load(f)
        agent.load(data)

    print("Agent loaded successfully!")

    # 初期状態を生成
    state = agent.init_policy(batch_size=1)

    return agent, state


def main():
    parser = argparse.ArgumentParser(
        description='DreamerV3 Real WHILL Inference Node',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # デフォルト設定で実行 (checkpoint_20000)
  python scripts/real_whill_inference.py

  # チェックポイントを指定して実行
  python scripts/real_whill_inference.py --checkpoint 15000

  # finalチェックポイントを使用
  python scripts/real_whill_inference.py --checkpoint 0

  # カスタム設定
  python scripts/real_whill_inference.py \\
    --logdir /path/to/logs \\
    --checkpoint 20000 \\
    --max-linear-vel 0.2 \\
    --control-hz 5
        """
    )
    parser.add_argument(
        '--logdir', type=Path,
        default=str(REPO_ROOT / 'logs' / 'dreamer' / 'obstacle_avoidance'),
        help='学習ログディレクトリ'
    )
    parser.add_argument(
        '--checkpoint', type=int, default=20000,
        help='チェックポイントのステップ番号 (0=final)'
    )
    parser.add_argument(
        '--camera-topic', type=str, default='/camera/image_cropped',
        help='カメラ画像トピック'
    )
    parser.add_argument(
        '--cmd-vel-topic', type=str, default='/whill/controller/cmd_vel',
        help='速度指令トピック'
    )
    parser.add_argument(
        '--max-linear-vel', type=float, default=0.3,
        help='最大線速度 (m/s)'
    )
    parser.add_argument(
        '--max-angular-vel', type=float, default=0.6,
        help='最大角速度 (rad/s)'
    )
    parser.add_argument(
        '--control-hz', type=float, default=10.0,
        help='制御周波数 (Hz)'
    )
    parser.add_argument(
        '--duration', type=float, default=20.0,
        help='推論の最大実行時間 (秒, 0=無制限)'
    )
    parser.add_argument(
        '--record', action='store_true',
        help='推論中のcmd_velをrosbagに保存'
    )
    parser.add_argument(
        '--record-dir', type=str,
        default=str(REPO_ROOT / 'logs' / 'real_whill_rosbags'),
        help='rosbag保存先ディレクトリ'
    )
    parser.add_argument(
        '--odom-topic', type=str, default='/whill/odom',
        help='オドメトリトピック (軌道記録用)'
    )
    parser.add_argument(
        '--save-trajectory', action='store_true', default=True,
        help='軌道データ(CSV/PNG)を保存する (デフォルト: ON)'
    )
    parser.add_argument(
        '--no-save-trajectory', dest='save_trajectory', action='store_false',
        help='軌道データの保存を無効化'
    )
    args = parser.parse_args()

    print("=" * 60)
    print("DreamerV3 Real WHILL Inference")
    print("=" * 60)
    print(f"  Log directory:  {args.logdir}")
    print(f"  Checkpoint:     {args.checkpoint}")
    print(f"  Camera topic:   {args.camera_topic}")
    print(f"  Cmd vel topic:  {args.cmd_vel_topic}")
    print(f"  Max linear vel: {args.max_linear_vel} m/s")
    print(f"  Max angular vel:{args.max_angular_vel} rad/s")
    print(f"  Control freq:   {args.control_hz} Hz")
    print(f"  Duration:       {args.duration}s" if args.duration else f"  Duration:       unlimited")
    print(f"  Record:         {'ON → ' + args.record_dir if args.record else 'OFF'}")
    print(f"  Trajectory:     {'ON → ' + args.record_dir if args.save_trajectory else 'OFF'}")
    print(f"  Odom topic:     {args.odom_topic}")
    print("=" * 60)

    # ----- DreamerV3 Agent のロード -----
    print("\n[1/2] Loading DreamerV3 agent...")
    agent, agent_state = load_agent(args.logdir, args.checkpoint)

    # ----- ROS 2 ノードの起動 -----
    print("\n[2/2] Starting ROS 2 inference node...")
    rclpy.init()

    node = RealWhillInferenceNode(
        agent=agent,
        agent_state=agent_state,
        camera_topic=args.camera_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        max_linear_vel=args.max_linear_vel,
        max_angular_vel=args.max_angular_vel,
        control_hz=args.control_hz,
        duration=args.duration if args.duration > 0 else None,
        record=args.record,
        record_dir=args.record_dir,
        odom_topic=args.odom_topic,
        save_trajectory=args.save_trajectory,
    )

    print("\n" + "=" * 60)
    print("STANDBY: 実機の準備完了を待機中...")
    print("  カメラ画像の受信と cmd_vel サブスクライバーの接続を待っています。")
    print("  実機側でカメラと cmd_vel 受信ノードを起動してください。")
    print("  準備完了後、自動で推論を開始します。")
    print("  Ctrl+C で停止。")
    print("=" * 60)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        try:
            node.stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
