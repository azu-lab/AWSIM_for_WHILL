"""
パラメータ一括評価モジュール

全パラメータファイルを全シナリオで評価し、
軌道RMSE/走行距離の精度を算出してCSV・PDFで出力する。

使用方法:
    python3 -m whill_optimizer batch-eval --param-dir ../parameter
"""

import sys
import os
import time
import csv
import json
import yaml
import numpy as np
import threading
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

import rclpy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from .grpc_client import GrpcClient, VehicleParameters
from .ros2_handler import ROS2Handler
from .rosbag_loader import load_all_scenarios, ScenarioData
from .evaluator import Evaluator

from . import compare_bags
from .paths import DEFAULT_ROSBAG_DIR


@dataclass
class EvalResult:
    """1つのパラメータ×シナリオの評価結果"""
    param_name: str
    scenario_name: str
    rmse_trajectory: float
    rmse_yaw_rad: float
    rmse_yaw_deg: float
    rmse_linear_velocity: float
    rmse_angular_velocity: float
    travel_distance: float  # 実機走行距離
    accuracy: float         # 軌道RMSE / 走行距離
    duration: float
    # プロット用データ
    sim_x: np.ndarray = None
    sim_y: np.ndarray = None
    real_x: np.ndarray = None
    real_y: np.ndarray = None
    success: bool = True
    error_msg: str = ""


class BatchEvaluator:
    """パラメータ一括評価クラス"""
    
    def __init__(self, grpc_host: str = "localhost", grpc_port: int = 50051,
                 rosbag_path: str = DEFAULT_ROSBAG_DIR):
        self.grpc_host = grpc_host
        self.grpc_port = grpc_port
        self.rosbag_path = rosbag_path
        
        self.client: Optional[GrpcClient] = None
        self.ros2: Optional[ROS2Handler] = None
        self.evaluator = Evaluator()
        self.real_scenarios: Dict[str, ScenarioData] = {}
        self.results: List[EvalResult] = []
    
    def _init_ros2(self):
        """ROS2初期化"""
        if not rclpy.ok():
            rclpy.init()
        self.ros2 = ROS2Handler()
        self.ros2_thread = threading.Thread(target=self._ros2_spin, daemon=True)
        self.ros2_thread.start()
        time.sleep(1.0)
    
    def _ros2_spin(self):
        """ROS2スピンループ"""
        while rclpy.ok():
            rclpy.spin_once(self.ros2, timeout_sec=0.01)
    
    def _shutdown_ros2(self):
        """ROS2シャットダウン"""
        if self.ros2:
            try:
                self.ros2.stop_vehicle()
                self.ros2.destroy_node()
            except Exception:
                pass
        if self.client:
            try:
                self.client.disconnect()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    
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
    
    def load_all_params(self, param_dir: str) -> Dict[str, Tuple[str, VehicleParameters]]:
        """パラメータディレクトリから全YAMLを読み込む"""
        param_dir = Path(param_dir)
        params = {}
        
        for yaml_file in sorted(param_dir.glob("*.yaml")):
            try:
                name = yaml_file.stem
                vp = self.load_yaml_params(str(yaml_file))
                params[name] = (str(yaml_file), vp)
                print(f"  Loaded: {name}")
            except Exception as e:
                print(f"  Warning: Failed to load {yaml_file.name}: {e}")
        
        return params
    
    def load_real_data(self):
        """実機rosbagデータを読み込む"""
        print(f"\nLoading real scenarios from: {self.rosbag_path}")
        self.real_scenarios = load_all_scenarios(self.rosbag_path)
        print(f"Loaded {len(self.real_scenarios)} scenarios: {list(self.real_scenarios.keys())}")
    
    @staticmethod
    def compute_travel_distance(records: List[Dict]) -> float:
        """軌跡レコードから走行距離を計算（累積距離）"""
        if len(records) < 2:
            return 0.0
        
        total_dist = 0.0
        for i in range(1, len(records)):
            dx = records[i]['x'] - records[i-1]['x']
            dy = records[i]['y'] - records[i-1]['y']
            total_dist += np.sqrt(dx**2 + dy**2)
        
        return total_dist
    
    def run_scenario(self, real_data: ScenarioData):
        """
        ROS2経由でシナリオを実行
        optimizer.py の run_ros2_scenario と同じフロー
        """
        print(f"  Starting ROS2 scenario execution ({real_data.duration:.1f}s)...")
        
        # 1. ギアをDriveに入れる
        self.ros2.set_gear_drive()
        
        # 2. システム安定化待機
        time.sleep(2.0)
        
        # 3. バッファをクリア = 記録開始
        self.ros2.clear_odom_buffer()
        
        # 4. 記録安定化待機
        time.sleep(1.0)
        
        # 5. 開始時刻記録
        start_time = time.time()
        
        timestamps = real_data.cmd_vel.timestamps
        linear_x = real_data.cmd_vel.linear_x
        angular_z = real_data.cmd_vel.angular_z
        
        for i in range(len(timestamps)):
            target_time = start_time + timestamps[i]
            current_time = time.time()
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            self.ros2.publish_cmd_vel(linear_x[i], angular_z[i])
        
        # シナリオ時間いっぱいまで待機
        remaining_time = (start_time + real_data.duration) - time.time()
        if remaining_time > 0:
            time.sleep(remaining_time)
        
        # 停止
        self.ros2.stop_vehicle()
    
    def run_single_eval(self, param_name: str, params: VehicleParameters,
                        scenario_name: str, real_data: ScenarioData) -> EvalResult:
        """1つのパラメータ×シナリオを評価"""
        print(f"\n  [{param_name}] x [{scenario_name}]")
        
        try:
            # Odomバッファクリア
            self.ros2.clear_odom_buffer()
            
            # Unityリセット & パラメータ適用
            self.client.run_scenario(
                params=params,
                cmd_vel_timestamps=[],
                cmd_vel_linear_x=[],
                cmd_vel_angular_z=[],
                duration=real_data.duration + 5.0,
                time_scale=1.0,
                odom_sample_interval=0.02,
            )
            
            # ROS2経由で制御実行
            self.run_scenario(real_data)
            
            # Odom取得
            sim_odoms = self.ros2.get_odom_buffer()
            
            if not sim_odoms:
                return EvalResult(
                    param_name=param_name, scenario_name=scenario_name,
                    rmse_trajectory=float('inf'), rmse_yaw_rad=0, rmse_yaw_deg=0,
                    rmse_linear_velocity=0, rmse_angular_velocity=0,
                    travel_distance=0, accuracy=float('inf'), duration=0,
                    success=False, error_msg="No odom data collected"
                )
            
            # 評価 (Evaluator.evaluate)
            result = self.evaluator.evaluate(sim_odoms, real_data.odom)
            
            # 実機走行距離を計算
            real_records = self.evaluator._extract_scenario_data(real_data.odom)
            real_records = compare_bags.transform_to_vehicle_frame(real_records)
            travel_dist = self.compute_travel_distance(real_records)
            
            # 精度 = RMSE / 走行距離
            accuracy = result.rmse_xy / travel_dist if travel_dist > 0 else float('inf')
            
            print(f"    RMSE={result.rmse_xy:.4f}m, 走行距離={travel_dist:.3f}m, "
                  f"精度={accuracy:.6f}")
            
            return EvalResult(
                param_name=param_name,
                scenario_name=scenario_name,
                rmse_trajectory=result.rmse_xy,
                rmse_yaw_rad=result.rmse_yaw,
                rmse_yaw_deg=result.details.get('rmse_yaw_deg', 0),
                rmse_linear_velocity=result.rmse_v,
                rmse_angular_velocity=result.rmse_w,
                travel_distance=travel_dist,
                accuracy=accuracy,
                duration=result.eval_duration,
                sim_x=result.sim_positions_x,
                sim_y=result.sim_positions_y,
                real_x=result.real_positions_x,
                real_y=result.real_positions_y,
                success=True
            )
            
        except Exception as e:
            print(f"    ❌ Error: {e}")
            return EvalResult(
                param_name=param_name, scenario_name=scenario_name,
                rmse_trajectory=float('inf'), rmse_yaw_rad=0, rmse_yaw_deg=0,
                rmse_linear_velocity=0, rmse_angular_velocity=0,
                travel_distance=0, accuracy=float('inf'), duration=0,
                success=False, error_msg=str(e)
            )
    
    def save_csv(self, output_path: str):
        """結果をCSVに保存"""
        with open(output_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # ヘッダー
            writer.writerow([
                'param_file', 'scenario',
                'rmse_trajectory_m', 'travel_distance_m', 'accuracy',
                'rmse_yaw_rad', 'rmse_yaw_deg',
                'rmse_linear_velocity', 'rmse_angular_velocity',
                'duration_s', 'success', 'error'
            ])
            
            # データ行
            for r in self.results:
                writer.writerow([
                    r.param_name, r.scenario_name,
                    f"{r.rmse_trajectory:.6f}", f"{r.travel_distance:.4f}",
                    f"{r.accuracy:.8f}",
                    f"{r.rmse_yaw_rad:.6f}", f"{r.rmse_yaw_deg:.4f}",
                    f"{r.rmse_linear_velocity:.6f}", f"{r.rmse_angular_velocity:.6f}",
                    f"{r.duration:.2f}", r.success, r.error_msg
                ])
        
        print(f"\nCSV saved: {output_path}")
        
        # サマリーテーブルも出力
        self._print_summary_table()
    
    def _print_summary_table(self):
        """サマリーテーブルをコンソールに表示"""
        # パラメータ名とシナリオ名を収集
        param_names = sorted(set(r.param_name for r in self.results))
        scenario_names = sorted(set(r.scenario_name for r in self.results))
        
        # 結果を辞書に格納
        result_map = {}
        for r in self.results:
            result_map[(r.param_name, r.scenario_name)] = r
        
        print("\n" + "=" * 120)
        print("BATCH EVALUATION SUMMARY - 精度 (軌道RMSE / 走行距離)")
        print("=" * 120)
        
        # ヘッダー
        header = f"{'Parameter':<25}"
        for s in scenario_names:
            header += f" | {s:<14}"
        header += f" | {'Average':<14}"
        print(header)
        print("-" * 120)
        
        # 各パラメータの行
        for p in param_names:
            row = f"{p:<25}"
            accuracies = []
            for s in scenario_names:
                r = result_map.get((p, s))
                if r and r.success:
                    row += f" | {r.accuracy:<14.6f}"
                    accuracies.append(r.accuracy)
                else:
                    row += f" | {'FAIL':<14}"
            
            avg = np.mean(accuracies) if accuracies else float('inf')
            row += f" | {avg:<14.6f}"
            print(row)
        
        print("=" * 120)
    
    def save_pdf(self, output_path: str):
        """軌道グラフをPDFに保存"""
        plt.rcParams['font.family'] = ['DejaVu Sans', 'sans-serif']
        
        # 成功した結果のみプロット
        success_results = [r for r in self.results if r.success and r.sim_x is not None]
        
        if not success_results:
            print("No successful results to plot.")
            return
        
        # パラメータ名とシナリオ名を収集
        param_names = sorted(set(r.param_name for r in success_results))
        scenario_names = sorted(set(r.scenario_name for r in success_results))
        
        result_map = {}
        for r in success_results:
            result_map[(r.param_name, r.scenario_name)] = r
        
        with PdfPages(output_path) as pdf:
            # ページ1: サマリーテーブル
            fig, ax = plt.subplots(figsize=(16, max(4, len(param_names) * 0.5 + 2)))
            ax.axis('off')
            
            headers = ['Parameter'] + scenario_names + ['Average']
            table_data = []
            for p in param_names:
                row = [p]
                accuracies = []
                for s in scenario_names:
                    r = result_map.get((p, s))
                    if r:
                        row.append(f"{r.accuracy:.6f}")
                        accuracies.append(r.accuracy)
                    else:
                        row.append("N/A")
                avg = np.mean(accuracies) if accuracies else float('inf')
                row.append(f"{avg:.6f}")
                table_data.append(row)
            
            table = ax.table(
                cellText=table_data,
                colLabels=headers,
                loc='center',
                cellLoc='center'
            )
            table.auto_set_font_size(False)
            table.set_fontsize(8)
            table.scale(1.0, 1.4)
            
            # ヘッダーの色
            for j in range(len(headers)):
                table[0, j].set_facecolor('#4472C4')
                table[0, j].set_text_props(color='white', fontweight='bold')
            
            # Average列のハイライト
            avg_col = len(headers) - 1
            for i in range(1, len(table_data) + 1):
                table[i, avg_col].set_facecolor('#D9E2F3')
            
            ax.set_title('Accuracy Summary (RMSE / Travel Distance)', fontsize=14, pad=20)
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
            
            # シナリオごとにページを作成
            for scenario in scenario_names:
                scenario_results = [r for r in success_results if r.scenario_name == scenario]
                if not scenario_results:
                    continue
                
                n_params = len(scenario_results)
                cols = min(3, n_params)
                rows = (n_params + cols - 1) // cols
                
                fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5 * rows))
                fig.suptitle(f'Scenario: {scenario}', fontsize=16, y=1.02)
                
                if n_params == 1:
                    axes = np.array([axes])
                axes = np.atleast_2d(axes)
                
                for idx, r in enumerate(sorted(scenario_results, key=lambda x: x.param_name)):
                    row_i = idx // cols
                    col_i = idx % cols
                    ax = axes[row_i, col_i]
                    
                    ax.plot(r.real_x, r.real_y, 'b-', linewidth=2, label='Real')
                    ax.plot(r.sim_x, r.sim_y, 'r--', linewidth=1.5, label='Sim')
                    
                    ax.set_xlabel('X [m]')
                    ax.set_ylabel('Y [m]')
                    ax.set_title(f'{r.param_name}\n'
                                f'RMSE={r.rmse_trajectory:.4f}m, '
                                f'Acc={r.accuracy:.6f}',
                                fontsize=9)
                    ax.legend(fontsize=8)
                    ax.grid(True, alpha=0.3)
                    ax.set_aspect('equal')
                
                # 余ったaxesを非表示
                for idx in range(n_params, rows * cols):
                    row_i = idx // cols
                    col_i = idx % cols
                    axes[row_i, col_i].set_visible(False)
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close(fig)
        
        print(f"PDF saved: {output_path}")
    
    def save_json(self, output_path: str):
        """軌道データをJSONに保存（再プロット・追加分析用）"""
        data = {
            "timestamp": datetime.now().isoformat(),
            "evaluations": []
        }
        
        for r in self.results:
            entry = {
                "param_name": r.param_name,
                "scenario_name": r.scenario_name,
                "metrics": {
                    "rmse_trajectory": r.rmse_trajectory,
                    "rmse_yaw_rad": r.rmse_yaw_rad,
                    "rmse_yaw_deg": r.rmse_yaw_deg,
                    "rmse_linear_velocity": r.rmse_linear_velocity,
                    "rmse_angular_velocity": r.rmse_angular_velocity,
                    "travel_distance": r.travel_distance,
                    "accuracy": r.accuracy,
                    "duration": r.duration,
                },
                "success": r.success,
                "error_msg": r.error_msg,
                "trajectory": None
            }
            
            if r.success and r.sim_x is not None:
                entry["trajectory"] = {
                    "sim_x": r.sim_x.tolist(),
                    "sim_y": r.sim_y.tolist(),
                    "real_x": r.real_x.tolist(),
                    "real_y": r.real_y.tolist(),
                }
            
            data["evaluations"].append(entry)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        
        print(f"JSON saved: {output_path}")
    
    def run(self, param_dir: str, output_prefix: str = "batch_eval"):
        """全パラメータ×全シナリオの評価を実行"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            # 初期化
            print("=" * 60)
            print("パラメータ一括評価 (Batch Evaluation)")
            print("=" * 60)
            
            print("\nInitializing ROS2...")
            self._init_ros2()
            
            print("Connecting to AWSIM...")
            self.client = GrpcClient(self.grpc_host, self.grpc_port)
            if not self.client.connect():
                raise RuntimeError("Failed to connect to AWSIM")
            
            # データ読み込み
            self.load_real_data()
            
            print(f"\nLoading parameters from: {param_dir}")
            all_params = self.load_all_params(param_dir)
            print(f"Loaded {len(all_params)} parameter files")
            
            scenario_names = sorted(self.real_scenarios.keys())
            total_evals = len(all_params) * len(scenario_names)
            print(f"\nTotal evaluations: {len(all_params)} params × {len(scenario_names)} scenarios = {total_evals}")
            print(f"Scenarios: {scenario_names}")
            
            # 評価ループ
            eval_count = 0
            for param_name, (yaml_path, params) in sorted(all_params.items()):
                print(f"\n{'='*60}")
                print(f"Parameter: {param_name} ({yaml_path})")
                print(f"{'='*60}")
                
                for scenario_name in scenario_names:
                    eval_count += 1
                    print(f"\n--- [{eval_count}/{total_evals}] ---")
                    
                    real_data = self.real_scenarios[scenario_name]
                    result = self.run_single_eval(param_name, params, scenario_name, real_data)
                    self.results.append(result)
            
            # 結果出力
            csv_path = f"{output_prefix}_{timestamp}.csv"
            pdf_path = f"{output_prefix}_{timestamp}.pdf"
            json_path = f"{output_prefix}_{timestamp}.json"
            
            self.save_csv(csv_path)
            self.save_pdf(pdf_path)
            self.save_json(json_path)
            
            print(f"\n✅ 完了")
            print(f"CSV: {csv_path}")
            print(f"PDF: {pdf_path}")
            print(f"JSON: {json_path}  ← 軌道座標データ（再プロット用）")
            
        finally:
            self._shutdown_ros2()


def run_batch_eval(param_dir: str = "../parameter",
                   rosbag_path: str = DEFAULT_ROSBAG_DIR,
                   grpc_host: str = "localhost",
                   grpc_port: int = 50051,
                   output_prefix: str = "batch_eval"):
    """バッチ評価を実行するエントリーポイント"""
    evaluator = BatchEvaluator(
        grpc_host=grpc_host,
        grpc_port=grpc_port,
        rosbag_path=rosbag_path
    )
    evaluator.run(param_dir, output_prefix)
