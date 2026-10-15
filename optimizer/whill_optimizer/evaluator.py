import sys
import os
import numpy as np
import math
from typing import List, Dict, Any

from . import compare_bags

from dataclasses import dataclass

@dataclass
class EvaluationResult:
    """評価結果"""
    rmse_xy: float
    rmse_yaw: float
    rmse_v: float
    rmse_w: float
    eval_duration: float
    
    # 可視化用データ
    sim_positions_x: np.ndarray
    sim_positions_y: np.ndarray
    real_positions_x: np.ndarray
    real_positions_y: np.ndarray
    timestamps: np.ndarray
    details: Dict[str, float] = None
    
    # Raw simulation data (non-interpolated, but transformed to vehicle frame)
    sim_raw_x: np.ndarray = None
    sim_raw_y: np.ndarray = None
    sim_raw_timestamps: np.ndarray = None


class Evaluator:
    """軌跡評価クラス (compare_bags.py のロジックを直接利用)"""
    
    # 動き始め検出の閾値
    MOTION_VELOCITY_THRESHOLD = 0.05  # m/s
    MOTION_POSITION_THRESHOLD = 0.01  # m
    
    def __init__(self):
        pass
    
    def _detect_motion_start(self, records: List[Dict], threshold: float = None) -> int:
        """
        軌跡データから動き始めのインデックスを検出
        
        Args:
            records: 軌跡レコードのリスト (ts, x, y, v, ...)
            threshold: 動き始め判定の位置変化閾値 (m)
            
        Returns:
            動き始めのインデックス (見つからない場合は0)
        """
        if threshold is None:
            threshold = self.MOTION_POSITION_THRESHOLD
            
        if len(records) < 2:
            return 0
        
        # 位置変化量で検出
        for i in range(1, len(records)):
            dx = records[i]['x'] - records[0]['x']
            dy = records[i]['y'] - records[0]['y']
            dist = math.sqrt(dx**2 + dy**2)
            if dist > threshold:
                return i
        
        return 0
    
    def _align_motion_start(self, real_records: List[Dict], sim_records: List[Dict]) -> tuple:
        """
        実機とシミュレーションの動き始めタイミングを揃える
        
        両方の軌跡で動き始めたタイミングを検出し、
        そのタイミングを新しいt=0として両方のタイムスタンプをシフトする。
        動き始め前のデータはトリミングする。
        
        Args:
            real_records: 実機軌跡レコード
            sim_records: シミュレーション軌跡レコード
            
        Returns:
            (aligned_real, aligned_sim): 動き始めを揃えた軌跡レコード
        """
        # 動き始めインデックスを検出
        real_start_idx = self._detect_motion_start(real_records)
        sim_start_idx = self._detect_motion_start(sim_records)
        
        # 動き始め時刻を取得
        real_start_time = real_records[real_start_idx]['ts']
        sim_start_time = sim_records[sim_start_idx]['ts']
        
        print(f"Motion start alignment:")
        print(f"  Real: starts at idx={real_start_idx}, t={real_start_time:.3f}s")
        print(f"  Sim:  starts at idx={sim_start_idx}, t={sim_start_time:.3f}s")
        
        # 動き始め以降のレコードのみを使用し、タイムスタンプをシフト
        aligned_real = []
        for r in real_records[real_start_idx:]:
            new_r = r.copy()
            new_r['ts'] = r['ts'] - real_start_time
            aligned_real.append(new_r)
        
        aligned_sim = []
        for r in sim_records[sim_start_idx:]:
            new_r = r.copy()
            new_r['ts'] = r['ts'] - sim_start_time
            aligned_sim.append(new_r)
        
        print(f"  Aligned: real={len(aligned_real)} samples, sim={len(aligned_sim)} samples")
        
        return aligned_real, aligned_sim
        
    def _extract_odom_data(self, odom_msgs) -> List[Dict]:
        """ROS2 Odometryメッセージリストをcompare_bags互換の辞書リストに変換"""
        records = []
        for msg in odom_msgs:
            # タイムスタンプ
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 位置
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # クォータニオン
            q = msg.pose.pose.orientation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            
            # ヨー角計算
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy**2 + qz**2)
            )
            
            # 速度
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            v = math.sqrt(vx**2 + vy**2)
            
            records.append({
                'ts': ts,
                'x': x,
                'y': y,
                'yaw': yaw,
                'v': v,
                'vx': vx,
                'vy': vy,
                'w': wz,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
            })
            
        return records

    def _extract_scenario_data(self, scenario_odom) -> List[Dict]:
        """ScenarioData.odom をcompare_bags互換の辞書リストに変換"""
        records = []
        timestamps = scenario_odom.timestamps
        x = scenario_odom.positions_x
        y = scenario_odom.positions_y
        
        qx = scenario_odom.orientations[:, 0]
        qy = scenario_odom.orientations[:, 1]
        qz = scenario_odom.orientations[:, 2]
        qw = scenario_odom.orientations[:, 3]
        
        vx = scenario_odom.linear_velocities[:, 0]
        vy = scenario_odom.linear_velocities[:, 1]  # 修正: vyを正しく取得
        wz = scenario_odom.angular_velocities[:, 2]
        
        for i in range(len(timestamps)):
            _qx, _qy, _qz, _qw = qx[i], qy[i], qz[i], qw[i]
            _yaw = math.atan2(
                2.0 * (_qw * _qz + _qx * _qy),
                1.0 - 2.0 * (_qy**2 + _qz**2)
            )
            # 修正: sqrt(vx^2 + vy^2) で計算 (compare_bags.py と同様)
            _v = math.sqrt(vx[i]**2 + vy[i]**2)
            
            records.append({
                'ts': timestamps[i],
                'x': x[i],
                'y': y[i],
                'yaw': _yaw,
                'v': _v,
                'vx': vx[i],
                'vy': vy[i],  # 修正: 実際のvy値を使用
                'w': wz[i],
                'qx': _qx, 'qy': _qy, 'qz': _qz, 'qw': _qw
            })
            
        return records

    def evaluate(self, sim_odom_msgs, real_odom_data) -> EvaluationResult:
        """シミュレーション結果と実機データを比較評価 (compare_bags.py利用)"""
        if not sim_odom_msgs:
            raise ValueError("No simulation data")
            
        # データの抽出
        sim_records = self._extract_odom_data(sim_odom_msgs)
        real_records = self._extract_scenario_data(real_odom_data)
        
        if not sim_records or not real_records:
            raise ValueError("Empty records")
            
        # 1. 車両座標系への変換 (compare_bags.py と同じ順序で先に行う)
        real_records = compare_bags.transform_to_vehicle_frame(real_records)
        sim_records = compare_bags.transform_to_vehicle_frame(sim_records)
        
        # 2. タイムスタンプ同期 (開始時刻を0に)
        real_records, sim_records = compare_bags.synchronize_time(real_records, sim_records)
        
        # 3. RMSE計算 (compare_bagsのロジックを使用)
        # compare_bags.py と完全に同じ処理 - 動き始め整合は行わない
        # (optimizer.py のタイミングを replay_bag_on_awsim.py と一致させているため)
        rmse_dict = compare_bags.compute_rmse_metrics(real_records, sim_records)
        
        if rmse_dict is None:
             raise ValueError("Failed to compute RMSE")

        # 可視化用に補間済みデータを再取得する必要がある
        # compare_bags.compute_rmse_metrics は計算結果の数値しか返さないため、
        # ここで可視化用データ生成のために同じロジック(の一部)を走らせるか、
        # あるいは補完についてはシンプルにやるか。
        # ここでは dashboard 表示用に、実機データと、それに合わせたSimデータを生成する。
        
        # 共通期間
        min_duration = rmse_dict['duration']
        real_filtered = [r for r in real_records if r['ts'] <= min_duration]
        sim_filtered = [r for r in sim_records if r['ts'] <= min_duration]
        
        base_times = np.array([r['ts'] for r in real_filtered])
        sim_times = np.array([r['ts'] for r in sim_filtered])
        sim_x = np.array([r['x'] for r in sim_filtered])
        sim_y = np.array([r['y'] for r in sim_filtered])
        
        # ダッシュボード表示用には実機時間軸に合わせた補間データを作成
        sim_x_interp = np.interp(base_times, sim_times, sim_x)
        sim_y_interp = np.interp(base_times, sim_times, sim_y)
        
        real_x = np.array([r['x'] for r in real_filtered])
        real_y = np.array([r['y'] for r in real_filtered])

        return EvaluationResult(
            rmse_xy=rmse_dict['trajectory'],
            rmse_yaw=rmse_dict['yaw_rad'],
            rmse_v=rmse_dict['linear_velocity'],
            rmse_w=rmse_dict['angular_velocity'],
            eval_duration=min_duration,
            sim_positions_x=sim_x_interp,
            sim_positions_y=sim_y_interp,
            real_positions_x=real_x,
            real_positions_y=real_y,
            timestamps=base_times,
            details={
                "rmse_xy": rmse_dict['trajectory'],
                "rmse_yaw_deg": rmse_dict['yaw_deg'],
                "rmse_v": rmse_dict['linear_velocity'],
                "rmse_w": rmse_dict['angular_velocity']
            },
            sim_raw_x=sim_x,
            sim_raw_y=sim_y,
            sim_raw_timestamps=sim_times
        )
