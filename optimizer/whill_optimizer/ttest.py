"""
t検定によるパラメータ検証モジュール

初期パラメータと最適化パラメータでそれぞれn回シミュレーションを実行し、
RMSEを比較してt検定で有意差を確認する。

使用方法:
    python3 -m whill_optimizer ttest
"""

import yaml
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from pathlib import Path
from datetime import datetime
import time
import threading
import sys
import os

# 統計
from scipy.stats import ttest_ind, t, sem

# PDF生成
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.font_manager as fm

# ROS2
import rclpy

# 既存モジュール
from .grpc_client import GrpcClient, VehicleParameters
from .ros2_handler import ROS2Handler
from .rosbag_loader import RosbagLoader, OdomData, CmdVelData

from . import compare_bags


@dataclass
class TrialResult:
    """1試行の結果"""
    trial_number: int
    rmse_trajectory: float
    rmse_velocity: float
    rmse_angular_velocity: float
    rmse_yaw: float
    # 軌道データ（プロット用）
    trajectory_x: np.ndarray
    trajectory_y: np.ndarray
    timestamps: np.ndarray
    # Yaw角データ（プロット用）
    sim_yaw: np.ndarray = field(default_factory=lambda: np.array([]))
    real_yaw: np.ndarray = field(default_factory=lambda: np.array([]))


@dataclass
class GroupResults:
    """パラメータグループ（初期 or 最適化）の全試行結果"""
    name: str
    trials: List[TrialResult] = field(default_factory=list)
    
    def get_rmse_arrays(self) -> Dict[str, np.ndarray]:
        """各指標のRMSE配列を取得"""
        return {
            'trajectory': np.array([t.rmse_trajectory for t in self.trials]),
            'velocity': np.array([t.rmse_velocity for t in self.trials]),
            'angular_velocity': np.array([t.rmse_angular_velocity for t in self.trials]),
            'yaw': np.array([t.rmse_yaw for t in self.trials]),
        }
    
    def get_median_trial(self) -> TrialResult:
        """軌道RMSEの中央値となる試行を返す"""
        rmses = [t.rmse_trajectory for t in self.trials]
        sorted_indices = np.argsort(rmses)
        median_idx = sorted_indices[len(sorted_indices) // 2]
        return self.trials[median_idx]


class TTestRunner:
    """t検定実行クラス"""
    
    def __init__(self, grpc_host: str = "localhost", grpc_port: int = 50051):
        self.grpc_host = grpc_host
        self.grpc_port = grpc_port
        
        # gRPCクライアント
        self.client: Optional[GrpcClient] = None
        
        # ROS2ハンドラ
        self.ros2: Optional[ROS2Handler] = None
        self.ros2_thread: Optional[threading.Thread] = None
        
        # 実機データ
        self.real_odom: List[Dict] = []
        self.real_cmd_vel: List[Dict] = []
        self.real_duration: float = 0.0
        
        # スレッド制御用フラグ
        self._running: bool = False
    
    def _init_ros2(self):
        """ROS2初期化"""
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros2 = ROS2Handler()
        self._running = True
        self.ros2_thread = threading.Thread(target=self._ros2_spin, daemon=True)
        self.ros2_thread.start()
        print("ROS2 initialized")
    
    def _ros2_spin(self):
        """ROS2スピンループ"""
        while self._running and rclpy.ok():
            rclpy.spin_once(self.ros2, timeout_sec=0.1)
    
    def _shutdown_ros2(self):
        """ROS2シャットダウン"""
        # スレッドを停止
        self._running = False
        if self.ros2_thread:
            self.ros2_thread.join(timeout=2.0)
        
        # ノードを破棄
        if self.ros2:
            self.ros2.destroy_node()
        
        # rclpyシャットダウン
        if rclpy.ok():
            rclpy.shutdown()
    
    def load_yaml_params(self, yaml_path: str) -> VehicleParameters:
        """YAMLファイルからパラメータを読み込む"""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        params = VehicleParameters()
        vehicle_params = data.get('vehicle_params', data)
        
        for key, value in vehicle_params.items():
            if hasattr(params, key):
                setattr(params, key, float(value))
        
        return params
    
    def load_real_data(self, bag_path: str):
        """実機rosbagデータを読み込む"""
        print(f"Loading real data from: {bag_path}")
        
        # RosbagLoaderを使用
        loader = RosbagLoader(bag_path)
        odom_data, cmd_vel_data = loader.load()
        
        # OdomDataをcompare_bags形式（辞書リスト）に変換
        self.real_odom = self._convert_odom_data_to_list(odom_data)
        
        # CmdVelDataをリスト形式に変換
        self.real_cmd_vel = []
        for i in range(len(cmd_vel_data.timestamps)):
            self.real_cmd_vel.append({
                'ts': cmd_vel_data.timestamps[i],
                'linear_x': cmd_vel_data.linear_x[i],
                'angular_z': cmd_vel_data.angular_z[i]
            })
        
        # Duration計算
        self.real_duration = odom_data.timestamps[-1] - odom_data.timestamps[0]
        
        print(f"  Loaded {len(self.real_odom)} odom msgs, {len(self.real_cmd_vel)} cmd_vel msgs")
        print(f"  Duration: {self.real_duration:.2f}s")
    
    def _convert_odom_data_to_list(self, odom_data: OdomData) -> List[Dict]:
        """OdomDataを辞書リスト形式に変換"""
        records = []
        for i in range(len(odom_data.timestamps)):
            records.append({
                'ts': odom_data.timestamps[i],
                'x': odom_data.positions_x[i],
                'y': odom_data.positions_y[i],
                'z': odom_data.positions_z[i],
                'qx': odom_data.orientations[i, 0],
                'qy': odom_data.orientations[i, 1],
                'qz': odom_data.orientations[i, 2],
                'qw': odom_data.orientations[i, 3],
                'vx': odom_data.linear_velocities[i, 0],
                'vy': odom_data.linear_velocities[i, 1],
                'vz': odom_data.linear_velocities[i, 2],
                'wx': odom_data.angular_velocities[i, 0],
                'wy': odom_data.angular_velocities[i, 1],
                'wz': odom_data.angular_velocities[i, 2],
            })
        return records
    
    def run_single_trial(self, params: VehicleParameters, trial_num: int) -> TrialResult:
        """1回のシミュレーション試行を実行"""
        
        # 1. ギアをDriveに設定
        self.ros2.set_gear_drive()
        
        # 2. システム安定化待機
        time.sleep(2.0)
        
        # 3. バッファクリア（記録開始相当）
        self.ros2.clear_odom_buffer()
        
        # 4. 記録安定化待機
        time.sleep(1.0)
        
        # 5. cmd_vel再生
        start_time = time.time()
        cmd_idx = 0
        
        while cmd_idx < len(self.real_cmd_vel):
            elapsed = time.time() - start_time
            cmd = self.real_cmd_vel[cmd_idx]
            
            if elapsed >= cmd['ts']:
                self.ros2.publish_cmd_vel(cmd['linear_x'], cmd['angular_z'])
                cmd_idx += 1
            else:
                time.sleep(0.001)
        
        # 6. 停止コマンド
        self.ros2.publish_cmd_vel(0.0, 0.0)
        time.sleep(0.5)
        
        # 7. Odomデータ取得
        sim_odoms = self.ros2.get_odom_buffer()
        
        if len(sim_odoms) == 0:
            raise RuntimeError("No odom data collected")
        
        # 8. RMSE計算（compare_bags.pyのロジックを使用）
        # sim_odoms を compare_bags 形式に変換
        sim_records = self._convert_to_compare_bags_format(sim_odoms)
        
        # 実機データも変換
        real_records = self._convert_odom_list_to_records(self.real_odom)
        
        # 車両座標系に変換
        real_records = compare_bags.transform_to_vehicle_frame(real_records)
        sim_records = compare_bags.transform_to_vehicle_frame(sim_records)
        
        # 時間同期
        real_records, sim_records = compare_bags.synchronize_time(real_records, sim_records)
        
        # RMSE計算
        rmse_dict = compare_bags.compute_rmse_metrics(real_records, sim_records)
        
        if rmse_dict is None:
            raise RuntimeError("Failed to compute RMSE")
        
        # 軌道データ抽出（プロット用）
        traj_x = np.array([r['x'] for r in sim_records])
        traj_y = np.array([r['y'] for r in sim_records])
        timestamps = np.array([r['ts'] for r in sim_records])
        
        # Yaw角データ抽出
        sim_yaw = np.array([r['yaw'] for r in sim_records])
        
        # real_yawをsim時間軸に補間（配列長を揃えるため）
        real_times = np.array([r['ts'] for r in real_records])
        real_yaw_raw = np.array([r['yaw'] for r in real_records])
        real_yaw = np.interp(timestamps, real_times, real_yaw_raw)
        
        return TrialResult(
            trial_number=trial_num,
            rmse_trajectory=rmse_dict['trajectory'],
            rmse_velocity=rmse_dict['linear_velocity'],
            rmse_angular_velocity=rmse_dict['angular_velocity'],
            rmse_yaw=rmse_dict['yaw_rad'],
            trajectory_x=traj_x,
            trajectory_y=traj_y,
            timestamps=timestamps,
            sim_yaw=sim_yaw,
            real_yaw=real_yaw
        )
    
    def _convert_to_compare_bags_format(self, odom_msgs) -> List[Dict]:
        """ROS2 Odometryメッセージをcompare_bags形式に変換"""
        records = []
        
        for msg in odom_msgs:
            # msg は nav_msgs/Odometry
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            twist = msg.twist.twist
            
            # クォータニオンからyaw取得
            import math
            siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
            cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # 速度
            vx = twist.linear.x
            vy = twist.linear.y
            v = math.sqrt(vx**2 + vy**2)
            w = twist.angular.z
            
            records.append({
                'ts': ts,
                'x': pos.x,
                'y': pos.y,
                'z': pos.z,
                'yaw': yaw,
                'vx': vx,
                'vy': vy,
                'v': v,
                'w': w
            })
        
        return records
    
    def _convert_odom_list_to_records(self, odom_list: List[Dict]) -> List[Dict]:
        """rosbag_loaderのodomリストをcompare_bags形式に変換"""
        records = []
        
        for odom in odom_list:
            import math
            
            # クォータニオンからyaw
            qx, qy, qz, qw = odom['qx'], odom['qy'], odom['qz'], odom['qw']
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            vx = odom['vx']
            vy = odom['vy']
            v = math.sqrt(vx**2 + vy**2)
            
            records.append({
                'ts': odom['ts'],
                'x': odom['x'],
                'y': odom['y'],
                'z': odom['z'],
                'yaw': yaw,
                'vx': vx,
                'vy': vy,
                'v': v,
                'w': odom['wz']
            })
        
        return records
    
    def run_trials(self, params: VehicleParameters, n_trials: int, group_name: str) -> GroupResults:
        """n回の試行を実行"""
        results = GroupResults(name=group_name)
        
        print(f"\n--- {group_name}パラメータで実行中 ---")
        
        for i in range(n_trials):
            print(f"  [{i+1}/{n_trials}] ", end="", flush=True)
            
            try:
                # gRPCでパラメータ適用 & リセット
                self.client.run_scenario(
                    params=params,
                    cmd_vel_timestamps=[],
                    cmd_vel_linear_x=[],
                    cmd_vel_angular_z=[],
                    duration=self.real_duration + 5.0,
                    time_scale=1.0,
                    odom_sample_interval=0.033
                )
                
                # 試行実行
                result = self.run_single_trial(params, i)
                results.trials.append(result)
                
                print(f"RMSE: {result.rmse_trajectory:.4f}m")
                
            except Exception as e:
                print(f"Error: {e}")
        
        return results
    
    def compute_statistics(self, initial: GroupResults, optimized: GroupResults) -> Dict:
        """t検定を実行して統計結果を返す"""
        metrics = ['trajectory', 'velocity', 'angular_velocity', 'yaw']
        metric_names = {
            'trajectory': '軌道RMSE',
            'velocity': '速度RMSE',
            'angular_velocity': '角速度RMSE',
            'yaw': '角度RMSE'
        }
        metric_units = {
            'trajectory': 'm',
            'velocity': 'm/s',
            'angular_velocity': 'rad/s',
            'yaw': 'rad'
        }
        
        init_arrays = initial.get_rmse_arrays()
        opt_arrays = optimized.get_rmse_arrays()
        
        stats = {}
        
        for metric in metrics:
            init_data = init_arrays[metric]
            opt_data = opt_arrays[metric]
            
            # Welch's t-test
            t_stat, p_value = ttest_ind(init_data, opt_data, equal_var=False)
            
            # 95%信頼区間の計算 (Confidence Interval)
            # 初期パラメータ
            init_sem = sem(init_data)  # 標準誤差
            init_ci = t.interval(0.95, len(init_data)-1, loc=np.mean(init_data), scale=init_sem)
            
            # 最適化パラメータ
            opt_sem = sem(opt_data)
            opt_ci = t.interval(0.95, len(opt_data)-1, loc=np.mean(opt_data), scale=opt_sem)
            
            stats[metric] = {
                'name': metric_names[metric],
                'unit': metric_units[metric],
                'initial_mean': np.mean(init_data),
                'initial_std': np.std(init_data),
                'initial_ci': init_ci,  # (lower, upper)
                'optimized_mean': np.mean(opt_data),
                'optimized_std': np.std(opt_data),
                'optimized_ci': opt_ci, # (lower, upper)
                't_statistic': t_stat,
                'p_value': p_value,
                'significant': p_value < 0.05
            }
        
        return stats
    
    def generate_pdf_report(self, initial: GroupResults, optimized: GroupResults, 
                           stats: Dict, real_trajectory: Tuple[np.ndarray, np.ndarray],
                           output_path: str):
        """PDFレポートを生成"""
        
        # 日本語フォント設定
        plt.rcParams['font.family'] = ['DejaVu Sans', 'sans-serif']
        
        with PdfPages(output_path) as pdf:
            # ページ1: 軌道比較図
            fig, ax = plt.subplots(figsize=(10, 8))
            
            real_x, real_y = real_trajectory
            init_median = initial.get_median_trial()
            opt_median = optimized.get_median_trial()
            
            ax.plot(real_x, real_y, 'b-', linewidth=2, label='Real (Ground Truth)')
            ax.plot(init_median.trajectory_x, init_median.trajectory_y, 
                   'r--', linewidth=1.5, label=f'Initial (Median, RMSE={init_median.rmse_trajectory:.3f}m)')
            ax.plot(opt_median.trajectory_x, opt_median.trajectory_y, 
                   'g-.', linewidth=1.5, label=f'Optimized (Median, RMSE={opt_median.rmse_trajectory:.3f}m)')
            
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_title('Trajectory Comparison')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
            
            # ページ2: Yaw角比較図（時間 vs 角度[rad]）
            fig, ax = plt.subplots(figsize=(12, 6))
            
            # タイムスタンプを0始まりに正規化
            init_ts = init_median.timestamps - init_median.timestamps[0] if len(init_median.timestamps) > 0 else init_median.timestamps
            opt_ts = opt_median.timestamps - opt_median.timestamps[0] if len(opt_median.timestamps) > 0 else opt_median.timestamps
            
            # Real（実機）のyawは中央値試行のreal_yawを使用
            if len(init_median.real_yaw) > 0:
                ax.plot(init_ts, init_median.real_yaw, 'b-', linewidth=2, label='Real (Ground Truth)')
            
            if len(init_median.sim_yaw) > 0:
                ax.plot(init_ts, init_median.sim_yaw, 'r--', linewidth=1.5, 
                       label=f'Initial (Median, RMSE={init_median.rmse_yaw:.4f}rad)')
            
            if len(opt_median.sim_yaw) > 0:
                ax.plot(opt_ts, opt_median.sim_yaw, 'g-.', linewidth=1.5, 
                       label=f'Optimized (Median, RMSE={opt_median.rmse_yaw:.4f}rad)')
            
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Yaw Angle [rad]')
            ax.set_title('Yaw Angle Comparison (Median Trials)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
            
            # ページ3: 統計表
            fig, ax = plt.subplots(figsize=(12, 6))
            ax.axis('off')
            
            # テーブルデータ作成
            headers = ['Metric', 'Initial (Mean±SD)', 'Optimized (Mean±SD)', 'p-value', 'Significant']
            table_data = []
            
            for metric, s in stats.items():
                row = [
                    s['name'],
                    f"{s['initial_mean']:.3f} (95%CI: {s['initial_ci'][0]:.3f}-{s['initial_ci'][1]:.3f})",
                    f"{s['optimized_mean']:.3f} (95%CI: {s['optimized_ci'][0]:.3f}-{s['optimized_ci'][1]:.3f})",
                    f"{s['p_value']:.4f}",
                    "Yes" if s['significant'] else "No"
                ]
                table_data.append(row)
            
            table = ax.table(
                cellText=table_data,
                colLabels=headers,
                loc='center',
                cellLoc='center'
            )
            table.auto_set_font_size(False)
            table.set_fontsize(10)
            table.scale(1.2, 1.5)
            
            ax.set_title('T-Test Results (Welch\'s t-test, α=0.05)', fontsize=14, pad=20)
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
        
        print(f"PDF saved: {output_path}")
    
    def save_trajectory_json(self, initial: GroupResults, optimized: GroupResults,
                             real_trajectory: Tuple[np.ndarray, np.ndarray],
                             stats: Dict, output_path: str):
        """軌跡データをJSONに保存（後から再プロット可能）"""
        import json
        
        init_median = initial.get_median_trial()
        opt_median = optimized.get_median_trial()
        real_x, real_y = real_trajectory
        
        # タイムスタンプを0始まりに正規化
        init_ts = (init_median.timestamps - init_median.timestamps[0]).tolist() if len(init_median.timestamps) > 0 else []
        opt_ts = (opt_median.timestamps - opt_median.timestamps[0]).tolist() if len(opt_median.timestamps) > 0 else []
        
        data = {
            "real": {
                "x": real_x.tolist(),
                "y": real_y.tolist(),
                "label": "Real (Ground Truth)"
            },
            "initial": {
                "x": init_median.trajectory_x.tolist(),
                "y": init_median.trajectory_y.tolist(),
                "rmse": init_median.rmse_trajectory,
                "trial_number": init_median.trial_number,
                "label": f"Initial (Median, RMSE={init_median.rmse_trajectory:.3f}m)"
            },
            "optimized": {
                "x": opt_median.trajectory_x.tolist(),
                "y": opt_median.trajectory_y.tolist(),
                "rmse": opt_median.rmse_trajectory,
                "trial_number": opt_median.trial_number,
                "label": f"Optimized (Median, RMSE={opt_median.rmse_trajectory:.3f}m)"
            },
            # Yaw角データ（時間 vs yaw[rad]）
            "yaw_data": {
                "timestamps_initial": init_ts,
                "timestamps_optimized": opt_ts,
                "real_yaw": init_median.real_yaw.tolist() if len(init_median.real_yaw) > 0 else [],
                "initial_sim_yaw": init_median.sim_yaw.tolist() if len(init_median.sim_yaw) > 0 else [],
                "optimized_sim_yaw": opt_median.sim_yaw.tolist() if len(opt_median.sim_yaw) > 0 else [],
                "initial_rmse_yaw": float(init_median.rmse_yaw),
                "optimized_rmse_yaw": float(opt_median.rmse_yaw)
            },
            "stats": {
                metric: {
                    "name": s["name"],
                    "unit": s["unit"],
                    "initial_mean": float(s["initial_mean"]),
                    "initial_std": float(s["initial_std"]),
                    "initial_ci": [float(x) for x in s["initial_ci"]],
                    "optimized_mean": float(s["optimized_mean"]),
                    "optimized_std": float(s["optimized_std"]),
                    "optimized_ci": [float(x) for x in s["optimized_ci"]],
                    "p_value": float(s["p_value"]),
                    "significant": bool(s["significant"])
                }
                for metric, s in stats.items()
            }
        }
        
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Trajectory JSON saved: {output_path}")
    
    def generate_csv_log(self, initial: GroupResults, optimized: GroupResults, output_path: str):
        """CSVログを生成"""
        import csv
        
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Group', 'Trial', 'Trajectory_RMSE', 'Velocity_RMSE', 
                           'AngularVelocity_RMSE', 'Yaw_RMSE'])
            
            for trial in initial.trials:
                writer.writerow([
                    'Initial', trial.trial_number,
                    trial.rmse_trajectory, trial.rmse_velocity,
                    trial.rmse_angular_velocity, trial.rmse_yaw
                ])
            
            for trial in optimized.trials:
                writer.writerow([
                    'Optimized', trial.trial_number,
                    trial.rmse_trajectory, trial.rmse_velocity,
                    trial.rmse_angular_velocity, trial.rmse_yaw
                ])
        
        print(f"CSV saved: {output_path}")
    
    def run(self, real_bag_path: str, initial_yaml: str, optimized_yaml: str, n_trials: int = 10):
        """t検定を実行"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            # 初期化
            print("Initializing...")
            self._init_ros2()
            
            self.client = GrpcClient(self.grpc_host, self.grpc_port)
            if not self.client.connect():
                raise RuntimeError("Failed to connect to AWSIM")
            
            # 実機データ読み込み
            self.load_real_data(real_bag_path)
            
            # パラメータ読み込み
            initial_params = self.load_yaml_params(initial_yaml)
            optimized_params = self.load_yaml_params(optimized_yaml)
            
            print(f"\nInitial params loaded from: {initial_yaml}")
            print(f"Optimized params loaded from: {optimized_yaml}")
            
            # 試行実行
            initial_results = self.run_trials(initial_params, n_trials, "初期")
            optimized_results = self.run_trials(optimized_params, n_trials, "最適化")
            
            # 統計計算
            print("\n--- 統計計算中 ---")
            stats = self.compute_statistics(initial_results, optimized_results)
            
            # 結果表示
            print("\n=== 結果 ===")
            print(f"{'指標':<15} {'初期 (平均 & 95%信頼区間)':<35} {'最適化 (平均 & 95%信頼区間)':<35} {'p値':<10} {'有意差'}")
            print("-" * 110)
            
            for metric, s in stats.items():
                sig = "✓ あり" if s['significant'] else "- なし"
                init_str = f"{s['initial_mean']:.4f} [{s['initial_ci'][0]:.4f}-{s['initial_ci'][1]:.4f}]"
                opt_str = f"{s['optimized_mean']:.4f} [{s['optimized_ci'][0]:.4f}-{s['optimized_ci'][1]:.4f}]"
                
                print(f"{s['name']:<12} {init_str:<35} {opt_str:<35} {s['p_value']:.4f}     {sig}")
            
            # 実機軌道データ取得
            real_records = self._convert_odom_list_to_records(self.real_odom)
            real_records = compare_bags.transform_to_vehicle_frame(real_records)
            real_x = np.array([r['x'] for r in real_records])
            real_y = np.array([r['y'] for r in real_records])
            
            # PDF/CSV/JSON生成
            pdf_path = f"ttest_report_{timestamp}.pdf"
            csv_path = f"ttest_results_{timestamp}.csv"
            json_path = f"ttest_trajectory_{timestamp}.json"
            
            self.generate_pdf_report(initial_results, optimized_results, stats,
                                    (real_x, real_y), pdf_path)
            self.generate_csv_log(initial_results, optimized_results, csv_path)
            self.save_trajectory_json(initial_results, optimized_results, 
                                     (real_x, real_y), stats, json_path)
            
            print(f"\n✅ 完了")
            print(f"PDFレポート: {pdf_path}")
            print(f"CSVログ: {csv_path}")
            print(f"軌跡データ: {json_path}  ← plot-trajectory コマンドで再プロット可能")
            
        finally:
            self._shutdown_ros2()


def run_ttest():
    """対話形式でt検定を実行"""
    print("=== AWSIM パラメータ検証 (t検定) ===\n")
    
    # 入力取得
    real_bag = input("実機rosbagのパスを入力してください: ").strip()
    initial_yaml = input("初期パラメータYAMLのパスを入力してください: ").strip()
    optimized_yaml = input("最適化パラメータYAMLのパスを入力してください: ").strip()
    
    n_trials_str = input("試行回数を入力してください [デフォルト: 10]: ").strip()
    n_trials = int(n_trials_str) if n_trials_str else 10
    
    # 入力検証
    for path in [real_bag, initial_yaml, optimized_yaml]:
        if not Path(path).exists():
            print(f"Error: Path not found: {path}")
            return
    
    # 実行
    runner = TTestRunner()
    runner.run(real_bag, initial_yaml, optimized_yaml, n_trials)


def run_ttest_triple():
    """対話形式で3グループt検定を実行"""
    print("=== AWSIM パラメータ検証 (3グループ比較) ===\n")
    
    # 入力取得
    real_bag = input("実機rosbagのパスを入力してください: ").strip()
    initial_yaml = input("初期パラメータYAMLのパスを入力してください: ").strip()
    optimized1_yaml = input("最適化パラメータ1 YAMLのパスを入力してください: ").strip()
    optimized2_yaml = input("最適化パラメータ2 YAMLのパスを入力してください: ").strip()
    
    n_trials_str = input("試行回数を入力してください [デフォルト: 10]: ").strip()
    n_trials = int(n_trials_str) if n_trials_str else 10
    
    # 入力検証
    for path in [real_bag, initial_yaml, optimized1_yaml, optimized2_yaml]:
        if not Path(path).exists():
            print(f"Error: Path not found: {path}")
            return
    
    # 実行
    runner = TTestRunnerTriple()
    runner.run(real_bag, initial_yaml, optimized1_yaml, optimized2_yaml, n_trials)


class TTestRunnerTriple(TTestRunner):
    """3グループ比較用のt検定実行クラス"""
    
    def compute_statistics_triple(self, initial: GroupResults, opt1: GroupResults, opt2: GroupResults) -> Dict:
        """3グループ間のt検定を実行して統計結果を返す"""
        metrics = ['trajectory', 'velocity', 'angular_velocity', 'yaw']
        metric_names = {
            'trajectory': '軌道RMSE',
            'velocity': '速度RMSE',
            'angular_velocity': '角速度RMSE',
            'yaw': '角度RMSE'
        }
        metric_units = {
            'trajectory': 'm',
            'velocity': 'm/s',
            'angular_velocity': 'rad/s',
            'yaw': 'rad'
        }
        
        init_arrays = initial.get_rmse_arrays()
        opt1_arrays = opt1.get_rmse_arrays()
        opt2_arrays = opt2.get_rmse_arrays()
        
        stats = {}
        
        for metric in metrics:
            init_data = init_arrays[metric]
            opt1_data = opt1_arrays[metric]
            opt2_data = opt2_arrays[metric]
            
            # 各ペアについてWelch's t-test
            t_init_opt1, p_init_opt1 = ttest_ind(init_data, opt1_data, equal_var=False)
            t_init_opt2, p_init_opt2 = ttest_ind(init_data, opt2_data, equal_var=False)
            t_opt1_opt2, p_opt1_opt2 = ttest_ind(opt1_data, opt2_data, equal_var=False)
            
            # 信頼区間
            init_sem = sem(init_data)
            init_ci = t.interval(0.95, len(init_data)-1, loc=np.mean(init_data), scale=init_sem)
            
            opt1_sem = sem(opt1_data)
            opt1_ci = t.interval(0.95, len(opt1_data)-1, loc=np.mean(opt1_data), scale=opt1_sem)
            
            opt2_sem = sem(opt2_data)
            opt2_ci = t.interval(0.95, len(opt2_data)-1, loc=np.mean(opt2_data), scale=opt2_sem)
            
            stats[metric] = {
                'name': metric_names[metric],
                'unit': metric_units[metric],
                'initial_mean': np.mean(init_data),
                'initial_std': np.std(init_data),
                'initial_ci': init_ci,
                'opt1_mean': np.mean(opt1_data),
                'opt1_std': np.std(opt1_data),
                'opt1_ci': opt1_ci,
                'opt2_mean': np.mean(opt2_data),
                'opt2_std': np.std(opt2_data),
                'opt2_ci': opt2_ci,
                # ペアワイズ比較
                'p_init_opt1': p_init_opt1,
                'p_init_opt2': p_init_opt2,
                'p_opt1_opt2': p_opt1_opt2,
                'sig_init_opt1': p_init_opt1 < 0.05,
                'sig_init_opt2': p_init_opt2 < 0.05,
                'sig_opt1_opt2': p_opt1_opt2 < 0.05,
            }
        
        return stats
    
    def generate_pdf_report_triple(self, initial: GroupResults, opt1: GroupResults, opt2: GroupResults,
                                    stats: Dict, real_trajectory: Tuple[np.ndarray, np.ndarray],
                                    output_path: str, opt1_name: str = "Opt1", opt2_name: str = "Opt2"):
        """3グループ比較用PDFレポートを生成"""
        plt.rcParams['font.family'] = ['DejaVu Sans', 'sans-serif']
        
        with PdfPages(output_path) as pdf:
            # ページ1: 軌道比較図
            fig, ax = plt.subplots(figsize=(12, 8))
            
            real_x, real_y = real_trajectory
            init_median = initial.get_median_trial()
            opt1_median = opt1.get_median_trial()
            opt2_median = opt2.get_median_trial()
            
            ax.plot(real_x, real_y, 'b-', linewidth=2, label='Real (Ground Truth)')
            ax.plot(init_median.trajectory_x, init_median.trajectory_y, 
                   'r--', linewidth=1.5, label=f'Initial (RMSE={init_median.rmse_trajectory:.3f}m)')
            ax.plot(opt1_median.trajectory_x, opt1_median.trajectory_y, 
                   'g-.', linewidth=1.5, label=f'{opt1_name} (RMSE={opt1_median.rmse_trajectory:.3f}m)')
            ax.plot(opt2_median.trajectory_x, opt2_median.trajectory_y, 
                   'm:', linewidth=1.5, label=f'{opt2_name} (RMSE={opt2_median.rmse_trajectory:.3f}m)')
            
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_title('Trajectory Comparison (3 Groups)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
            
            # ページ2: Yaw角比較図（時間 vs 角度[rad]）
            fig, ax = plt.subplots(figsize=(12, 6))
            
            # タイムスタンプを0始まりに正規化
            init_ts = init_median.timestamps - init_median.timestamps[0] if len(init_median.timestamps) > 0 else init_median.timestamps
            opt1_ts = opt1_median.timestamps - opt1_median.timestamps[0] if len(opt1_median.timestamps) > 0 else opt1_median.timestamps
            opt2_ts = opt2_median.timestamps - opt2_median.timestamps[0] if len(opt2_median.timestamps) > 0 else opt2_median.timestamps
            
            # Real（実機）のyaw
            if len(init_median.real_yaw) > 0:
                ax.plot(init_ts, init_median.real_yaw, 'b-', linewidth=2, label='Real (Ground Truth)')
            
            if len(init_median.sim_yaw) > 0:
                ax.plot(init_ts, init_median.sim_yaw, 'r--', linewidth=1.5, 
                       label=f'Initial (RMSE={init_median.rmse_yaw:.4f}rad)')
            
            if len(opt1_median.sim_yaw) > 0:
                ax.plot(opt1_ts, opt1_median.sim_yaw, 'g-.', linewidth=1.5, 
                       label=f'{opt1_name} (RMSE={opt1_median.rmse_yaw:.4f}rad)')
            
            if len(opt2_median.sim_yaw) > 0:
                ax.plot(opt2_ts, opt2_median.sim_yaw, 'm:', linewidth=1.5, 
                       label=f'{opt2_name} (RMSE={opt2_median.rmse_yaw:.4f}rad)')
            
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('Yaw Angle [rad]')
            ax.set_title('Yaw Angle Comparison (Median Trials)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
            
            # ページ3: 統計表
            fig, ax = plt.subplots(figsize=(16, 8))
            ax.axis('off')
            
            headers = ['Metric', 'Initial', opt1_name, opt2_name, 
                      'p(I vs O1)', 'p(I vs O2)', 'p(O1 vs O2)']
            table_data = []
            
            for metric, s in stats.items():
                row = [
                    s['name'],
                    f"{s['initial_mean']:.3f}±{s['initial_std']:.3f}",
                    f"{s['opt1_mean']:.3f}±{s['opt1_std']:.3f}",
                    f"{s['opt2_mean']:.3f}±{s['opt2_std']:.3f}",
                    f"{s['p_init_opt1']:.4f}" + ("*" if s['sig_init_opt1'] else ""),
                    f"{s['p_init_opt2']:.4f}" + ("*" if s['sig_init_opt2'] else ""),
                    f"{s['p_opt1_opt2']:.4f}" + ("*" if s['sig_opt1_opt2'] else ""),
                ]
                table_data.append(row)
            
            table = ax.table(
                cellText=table_data,
                colLabels=headers,
                loc='center',
                cellLoc='center'
            )
            table.auto_set_font_size(False)
            table.set_fontsize(9)
            table.scale(1.2, 1.5)
            
            ax.set_title('T-Test Results (Welch\'s t-test, *p<0.05)', fontsize=14, pad=20)
            
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
        
        print(f"PDF saved: {output_path}")
    
    def save_trajectory_json_triple(self, initial: GroupResults, opt1: GroupResults, opt2: GroupResults,
                                      real_trajectory: Tuple[np.ndarray, np.ndarray],
                                      stats: Dict, output_path: str,
                                      opt1_name: str = "Opt1", opt2_name: str = "Opt2"):
        """3グループ用軌跡データをJSONに保存"""
        import json
        
        init_median = initial.get_median_trial()
        opt1_median = opt1.get_median_trial()
        opt2_median = opt2.get_median_trial()
        real_x, real_y = real_trajectory
        
        # タイムスタンプを0始まりに正規化
        init_ts = (init_median.timestamps - init_median.timestamps[0]).tolist() if len(init_median.timestamps) > 0 else []
        opt1_ts = (opt1_median.timestamps - opt1_median.timestamps[0]).tolist() if len(opt1_median.timestamps) > 0 else []
        opt2_ts = (opt2_median.timestamps - opt2_median.timestamps[0]).tolist() if len(opt2_median.timestamps) > 0 else []
        
        data = {
            "real": {
                "x": real_x.tolist(),
                "y": real_y.tolist(),
                "label": "Real (Ground Truth)"
            },
            "initial": {
                "x": init_median.trajectory_x.tolist(),
                "y": init_median.trajectory_y.tolist(),
                "rmse": init_median.rmse_trajectory,
                "trial_number": init_median.trial_number,
                "label": f"Initial (RMSE={init_median.rmse_trajectory:.3f}m)"
            },
            opt1_name: {
                "x": opt1_median.trajectory_x.tolist(),
                "y": opt1_median.trajectory_y.tolist(),
                "rmse": opt1_median.rmse_trajectory,
                "trial_number": opt1_median.trial_number,
                "label": f"{opt1_name} (RMSE={opt1_median.rmse_trajectory:.3f}m)"
            },
            opt2_name: {
                "x": opt2_median.trajectory_x.tolist(),
                "y": opt2_median.trajectory_y.tolist(),
                "rmse": opt2_median.rmse_trajectory,
                "trial_number": opt2_median.trial_number,
                "label": f"{opt2_name} (RMSE={opt2_median.rmse_trajectory:.3f}m)"
            },
            # Yaw角データ（時間 vs yaw[rad]）
            "yaw_data": {
                "timestamps_initial": init_ts,
                "timestamps_opt1": opt1_ts,
                "timestamps_opt2": opt2_ts,
                "real_yaw": init_median.real_yaw.tolist() if len(init_median.real_yaw) > 0 else [],
                "initial_sim_yaw": init_median.sim_yaw.tolist() if len(init_median.sim_yaw) > 0 else [],
                "opt1_sim_yaw": opt1_median.sim_yaw.tolist() if len(opt1_median.sim_yaw) > 0 else [],
                "opt2_sim_yaw": opt2_median.sim_yaw.tolist() if len(opt2_median.sim_yaw) > 0 else [],
                "initial_rmse_yaw": float(init_median.rmse_yaw),
                "opt1_rmse_yaw": float(opt1_median.rmse_yaw),
                "opt2_rmse_yaw": float(opt2_median.rmse_yaw)
            },
            "stats": {
                metric: {
                    "name": s["name"],
                    "unit": s["unit"],
                    "initial_mean": float(s["initial_mean"]),
                    "initial_std": float(s["initial_std"]),
                    "opt1_mean": float(s["opt1_mean"]),
                    "opt1_std": float(s["opt1_std"]),
                    "opt2_mean": float(s["opt2_mean"]),
                    "opt2_std": float(s["opt2_std"]),
                    "p_init_opt1": float(s["p_init_opt1"]),
                    "p_init_opt2": float(s["p_init_opt2"]),
                    "p_opt1_opt2": float(s["p_opt1_opt2"]),
                    "sig_init_opt1": bool(s["sig_init_opt1"]),
                    "sig_init_opt2": bool(s["sig_init_opt2"]),
                    "sig_opt1_opt2": bool(s["sig_opt1_opt2"]),
                }
                for metric, s in stats.items()
            }
        }
        
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Trajectory JSON saved: {output_path}")
    
    def generate_csv_log_triple(self, initial: GroupResults, opt1: GroupResults, opt2: GroupResults,
                                 output_path: str, opt1_name: str = "Opt1", opt2_name: str = "Opt2"):
        """3グループ用CSVログを生成"""
        import csv
        
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Group', 'Trial', 'Trajectory_RMSE', 'Velocity_RMSE', 
                           'AngularVelocity_RMSE', 'Yaw_RMSE'])
            
            for trial in initial.trials:
                writer.writerow([
                    'Initial', trial.trial_number,
                    trial.rmse_trajectory, trial.rmse_velocity,
                    trial.rmse_angular_velocity, trial.rmse_yaw
                ])
            
            for trial in opt1.trials:
                writer.writerow([
                    opt1_name, trial.trial_number,
                    trial.rmse_trajectory, trial.rmse_velocity,
                    trial.rmse_angular_velocity, trial.rmse_yaw
                ])
            
            for trial in opt2.trials:
                writer.writerow([
                    opt2_name, trial.trial_number,
                    trial.rmse_trajectory, trial.rmse_velocity,
                    trial.rmse_angular_velocity, trial.rmse_yaw
                ])
        
        print(f"CSV saved: {output_path}")
    
    def run(self, real_bag_path: str, initial_yaml: str, opt1_yaml: str, opt2_yaml: str, n_trials: int = 10):
        """3グループt検定を実行"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # YAMLファイル名からグループ名を抽出
        opt1_name = Path(opt1_yaml).stem
        opt2_name = Path(opt2_yaml).stem
        
        try:
            # 初期化
            print("Initializing...")
            self._init_ros2()
            
            self.client = GrpcClient(self.grpc_host, self.grpc_port)
            if not self.client.connect():
                raise RuntimeError("Failed to connect to AWSIM")
            
            # 実機データ読み込み
            self.load_real_data(real_bag_path)
            
            # パラメータ読み込み
            initial_params = self.load_yaml_params(initial_yaml)
            opt1_params = self.load_yaml_params(opt1_yaml)
            opt2_params = self.load_yaml_params(opt2_yaml)
            
            print(f"\nInitial params loaded from: {initial_yaml}")
            print(f"Opt1 params loaded from: {opt1_yaml}")
            print(f"Opt2 params loaded from: {opt2_yaml}")
            
            # 試行実行
            initial_results = self.run_trials(initial_params, n_trials, "初期")
            opt1_results = self.run_trials(opt1_params, n_trials, opt1_name)
            opt2_results = self.run_trials(opt2_params, n_trials, opt2_name)
            
            # 統計計算
            print("\n--- 統計計算中 ---")
            stats = self.compute_statistics_triple(initial_results, opt1_results, opt2_results)
            
            # 結果表示
            print("\n=== 結果 ===")
            print(f"{'指標':<15} {'初期':<20} {opt1_name:<20} {opt2_name:<20} {'p(I-O1)':<10} {'p(I-O2)':<10} {'p(O1-O2)':<10}")
            print("-" * 115)
            
            for metric, s in stats.items():
                init_str = f"{s['initial_mean']:.4f}"
                opt1_str = f"{s['opt1_mean']:.4f}"
                opt2_str = f"{s['opt2_mean']:.4f}"
                sig1 = "✓" if s['sig_init_opt1'] else "-"
                sig2 = "✓" if s['sig_init_opt2'] else "-"
                sig12 = "✓" if s['sig_opt1_opt2'] else "-"
                
                print(f"{s['name']:<12} {init_str:<20} {opt1_str:<20} {opt2_str:<20} "
                      f"{s['p_init_opt1']:.4f}{sig1:<5} {s['p_init_opt2']:.4f}{sig2:<5} {s['p_opt1_opt2']:.4f}{sig12}")
            
            # 実機軌道データ取得
            real_records = self._convert_odom_list_to_records(self.real_odom)
            real_records = compare_bags.transform_to_vehicle_frame(real_records)
            real_x = np.array([r['x'] for r in real_records])
            real_y = np.array([r['y'] for r in real_records])
            
            # CSV/JSON/PDF生成（エラーが出ても他のファイルは生成されるように個別処理）
            pdf_path = f"ttest3_report_{timestamp}.pdf"
            csv_path = f"ttest3_results_{timestamp}.csv"
            json_path = f"ttest3_trajectory_{timestamp}.json"
            
            # CSV（最初に生成 - 最も軽量）
            try:
                self.generate_csv_log_triple(initial_results, opt1_results, opt2_results,
                                             csv_path, opt1_name, opt2_name)
            except Exception as e:
                print(f"Warning: CSV generation failed: {e}")
            
            # JSON（軌跡データ）
            try:
                self.save_trajectory_json_triple(initial_results, opt1_results, opt2_results,
                                                 (real_x, real_y), stats, json_path, opt1_name, opt2_name)
            except Exception as e:
                print(f"Warning: JSON generation failed: {e}")
            
            # PDF（最後 - 最も重い処理）
            try:
                self.generate_pdf_report_triple(initial_results, opt1_results, opt2_results, stats,
                                               (real_x, real_y), pdf_path, opt1_name, opt2_name)
            except Exception as e:
                print(f"Warning: PDF generation failed: {e}")
            
            print(f"\n✅ 完了")
            print(f"CSVログ: {csv_path}")
            print(f"軌跡データ: {json_path}")
            print(f"PDFレポート: {pdf_path}")
            
        finally:
            self._shutdown_ros2()


if __name__ == "__main__":
    run_ttest()
