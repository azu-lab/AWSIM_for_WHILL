"""
Optunaによる最適化コアモジュール

パラメータ空間の定義、目的関数、最適化ループを実装します。
"""

import optuna
from optuna.trial import Trial
import numpy as np
from typing import Dict, List, Optional, Callable, Any, Tuple
from dataclasses import dataclass, field
import json
import time
import csv
import math
from pathlib import Path
from datetime import datetime

from .grpc_client import GrpcClient, VehicleParameters
from .paths import DEFAULT_ROSBAG_DIR
from .rosbag_loader import load_all_scenarios, ScenarioData
from .evaluator import Evaluator
from .ros2_handler import ROS2Handler
import threading
import rclpy


class EarlyStoppedException(Exception):
    """累積スコアが閾値を超えた場合に発生する例外"""
    def __init__(self, score: float, scenario_scores: dict):
        self.score = score
        self.scenario_scores = scenario_scores
        super().__init__(f"Early stopped: cumulative score {score:.3f} > threshold")


@dataclass
class ParameterRange:
    """パラメータの探索範囲
    
    enabled=False にすると、そのパラメータはデフォルト値で固定され探索されない
    constrained=False にすると、low/high を無視して無制限（-1e9〜1e9）で探索
    """
    name: str
    low: float
    high: float
    default: float
    log_scale: bool = False      # 対数スケールで探索
    enabled: bool = True         # False にすると探索せずデフォルト値を使用
    constrained: bool = True     # False にすると無制限で探索
    
    # 無制限探索時の範囲
    UNCONSTRAINED_LOW: float = -1e9
    UNCONSTRAINED_HIGH: float = 1e9
    
    def suggest(self, trial: Trial) -> float:
        """Optunaで値を提案"""
        if not self.enabled:
            return self.default
        
        # 探索範囲を決定
        if self.constrained:
            low, high = self.low, self.high
        else:
            low, high = self.UNCONSTRAINED_LOW, self.UNCONSTRAINED_HIGH
        
        if self.log_scale:
            # 対数スケールの場合、無制限でも正の範囲にする必要がある
            low = max(low, 1e-9)
            return trial.suggest_float(self.name, low, high, log=True)
        else:
            return trial.suggest_float(self.name, low, high)


@dataclass
class OptimizerConfig:
    """最適化設定"""
    # 探索するシナリオ (Noneの場合は全シナリオ)
    scenarios: Optional[List[str]] = None
    
    # Optuna設定
    n_trials: int = 100
    timeout: Optional[float] = None  # 秒
    
    # シミュレーション設定
    time_scale: float = 5.0
    odom_sample_interval: float = 0.02
    
    # gRPC設定
    grpc_host: str = "localhost"
    grpc_port: int = 50051
    
    # デバッグモード (CSV出力)
    debug_mode: bool = False
    
    # データパス
    rosbag_base_path: str = DEFAULT_ROSBAG_DIR
    
    # 探索空間 (YAML)。None なら DEFAULT_PARAMETER_RANGES を使う
    search_space_path: Optional[str] = None

    # 結果保存
    study_name: str = "awsim_optimization"
    storage_path: str = "sqlite:///optuna_study.db"
    results_json_path: str = "optimization_results.json"  # 結果JSONファイルパス
    
    # コールバック
    on_trial_complete: Optional[Callable[[int, float, Dict], None]] = None


# デフォルトのパラメータ探索範囲
# enabled=False にすると探索せずデフォルト値で固定される
DEFAULT_PARAMETER_RANGES = [
    # 物理パラメータ
    ParameterRange("skidding_cancel_rate", 0.0, 50.0, 1.0, enabled=True),
    ParameterRange("front_wheel_skidding_rate", 0.0, 10.0, 0.1, enabled=True),
    ParameterRange("forward_slip_multiplier", 0.5, 1.0, 1.0, enabled=True),
    
    # PIDゲイン
    ParameterRange("wheel_kp", 0.0, 30.0, 10.0, enabled=True),
    ParameterRange("wheel_ki", 0.0, 1.0, 0.0, enabled=True),
    ParameterRange("wheel_kd", 0.0, 2.0, 0.0, enabled=True),
    ParameterRange("wheel_integral_clamp", 0.1, 3.0, 0.5, enabled=True),
    
    # 抵抗力
    ParameterRange("air_drag", 0.1, 1.5, 0.2, enabled=True),
    ParameterRange("rolling_resistance", 0.0, 0.05, 0.02, enabled=True),
    ParameterRange("frontal_area", 0.4, 1.2, 0.8, enabled=True),
    
    # 重心オフセット (車両原点は地面にある)
    # X軸: 左右 (正=右向き)
    # Y軸: 上下 (正=上向き) ※地面が0なので、車椅子の重心は0.2〜0.8m程度
    # Z軸: 前後 (正=車両前方)
    ParameterRange("com_offset_x", -0.2, 0.2, 0.0, enabled=True),   # 左右: ±20cm
    ParameterRange("com_offset_y", 0.1, 0.6, 0.3, enabled=True),    # 上下: 地面から20〜80cm (デフォルト50cm)
    ParameterRange("com_offset_z", -0.3, 0.3, 0.0, enabled=True),   # 前後: ±30cm
]


def load_parameter_ranges(path: str) -> List[ParameterRange]:
    """探索空間を YAML から読み込む (書式は config/optimizer/search_space.yaml)"""
    import yaml
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    ranges = []
    for name, spec in data["parameters"].items():
        ranges.append(ParameterRange(
            name=name,
            low=float(spec["low"]),
            high=float(spec["high"]),
            default=float(spec["default"]),
            log_scale=bool(spec.get("log", False)),
            enabled=bool(spec.get("enabled", True)),
        ))
    return ranges


class AWSimOptimizer:
    """AWSIM パラメータ最適化クラス"""
    
    def __init__(self, config: OptimizerConfig, 
                 parameter_ranges: Optional[List[ParameterRange]] = None):
        """
        Args:
            config: 最適化設定
            parameter_ranges: パラメータ探索範囲のリスト (Noneでデフォルト使用)
        """
        self.config = config
        if parameter_ranges is None and config.search_space_path:
            parameter_ranges = load_parameter_ranges(config.search_space_path)
        self.parameter_ranges = parameter_ranges or DEFAULT_PARAMETER_RANGES
        
        # gRPCクライアント
        self.client = GrpcClient(config.grpc_host, config.grpc_port)
        
        # ROS2ハンドラ (初期化とスピン開始)
        rclpy.init(args=None)
        self.ros2 = ROS2Handler()
        self.ros2_thread = threading.Thread(target=self.ros2_spin, daemon=True)
        self.ros2_thread.start()
        
        # 評価器
        self.evaluator = Evaluator()
        
        # 実機データ
        self.real_scenarios: Dict[str, ScenarioData] = {}
        
        # 結果保存用
        self.trial_history: List[Dict] = []
        self.best_params: Optional[Dict] = None
        self.best_score: float = float('inf')
        
        # 軌跡データ保存用
        self.initial_trajectories: Dict[str, Dict] = {}  # 初期パラメータの軌跡
        self.best_trajectories: Dict[str, Dict] = {}     # ベストパラメータの軌跡
        self._initial_trial_done: bool = False
        
        # 前回学習結果があれば読み込む
        self._load_previous_results()
    
    def _load_previous_results(self):
        """前回学習結果を復元"""
        res_file = Path(self.config.results_json_path)
        
        # DBファイルが存在しない場合は、JSONがあっても読み込まない (完全リセットのため)
        # config.storage_path = "sqlite:///optuna_study.db"
        db_path = "optuna_study.db"
        if self.config.storage_path.startswith("sqlite:///"):
             db_path = self.config.storage_path.replace("sqlite:///", "")
        
        if not Path(db_path).exists():
            print("DB file not found. Starting fresh optimization (ignoring previous json results).")
            return

        if not res_file.exists():
            return
            
        try:
            print(f"Loading previous results from {res_file}...")
            with open(res_file, 'r') as f:
                data = json.load(f)
                
            # 状態復元
            if "best_score" in data:
                self.best_score = data["best_score"]
            if "best_params" in data:
                self.best_params = data["best_params"]
            if "trial_history" in data:
                self.trial_history = data["trial_history"]
            
            # 軌跡データ復元
            if "trajectories" in data:
                trajectories = data["trajectories"]
                if "initial" in trajectories and trajectories["initial"]:
                    self.initial_trajectories = trajectories["initial"]
                    self._initial_trial_done = True
                    print(f"  Restored initial trajectories")
                if "best" in trajectories and trajectories["best"]:
                    self.best_trajectories = trajectories["best"]
                    print(f"  Restored best trajectories")
            
            print(f"  Restored {len(self.trial_history)} trials, best score: {self.best_score:.4f}")
            
        except Exception as e:
            print(f"Warning: Failed to load previous results: {e}")

    def _restore_state_from_db(self, study: optuna.Study):
        """DBから学習状態（履歴、ベスト値、重要軌跡）を完全復元する"""
        try:
            trials = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE]
            if not trials:
                return

            print(f"Restoring state from DB ({len(trials)} trials)...")

            # 1. 履歴の復元
            # JSONがない場合でも、DBから最低限の情報（スコア、パラメータ）を復元して、
            # ダッシュボードの試行回数表示などを整合させる。
            self.trial_history = []
            for t in trials:
                self.trial_history.append({
                    "trial_number": t.number,
                    "score": t.value,
                    "scenario_scores": {}, # 詳細スコアはDBにないので空
                    "params": t.params
                })

            # 2. ベストスコアの復元
            if study.best_trial:
                self.best_score = study.best_value
                self.best_params = study.best_params
                print(f"  Restored global best score: {self.best_score:.4f} (Trial {study.best_trial.number})")

            # 3. 軌跡データの復元（再計算）
            # Trial 0 (初期軌道) と Best Trial (ベスト軌道) を再計算して保存する
            
            # --- 初期軌道 (Trial 0) ---
            trial0 = next((t for t in trials if t.number == 0), None)
            if trial0:
                # 既にデータがあり、かつ Trial 0 のものであれば再計算スキップ
                is_valid = False
                if self.initial_trajectories:
                    first_val = next(iter(self.initial_trajectories.values()))
                    if first_val.get("trial_number") == 0:
                        is_valid = True
                        print("  Initial trajectories already loaded and valid. Skipping re-calc.")
                
                if not is_valid:
                    print("  Re-calculating Trial 0 (Initial)...")
                    params0 = self._create_params_from_dict(trial0.params)
                    _, _, traj0 = self._evaluate_params(params0, 0)
                    self.initial_trajectories = traj0
            
            # --- ベスト軌道 ---
            best_trial = study.best_trial
            if best_trial:
                # Trial 0と同じなら再計算不要（計算済みデータを使用）
                if trial0 and best_trial.number == 0:
                    print(f"  Best trial is Trial 0. Reusing initial trajectories.")
                    self.best_trajectories = self.initial_trajectories.copy()
                else:
                    # 既にデータがあり、かつ Best Trial のものであれば再計算スキップ
                    is_valid = False
                    if self.best_trajectories:
                        first_val = next(iter(self.best_trajectories.values()))
                        if first_val.get("trial_number") == best_trial.number:
                            is_valid = True
                            print(f"  Best trajectories already loaded (Trial {best_trial.number}). Skipping re-calc.")
                    
                    if not is_valid:
                        print(f"  Re-calculating Best Trial {best_trial.number}...")
                        params_best = self._create_params_from_dict(best_trial.params)
                        _, _, traj_best = self._evaluate_params(params_best, best_trial.number)
                        self.best_trajectories = traj_best
            
            # 復元結果をJSONに保存して、ダッシュボードが即座に反映できるようにする
            self._save_results_realtime()
            print("  State restoration complete.")
                
        except Exception as e:
            print(f"Warning: Failed to restore state from DB: {e}") 
            
            print(f"  Restored {len(self.trial_history)} trials, best score: {self.best_score:.4f}")
            
        except Exception as e:
            print(f"Warning: Failed to load previous results: {e}")

    def load_real_data(self) -> bool:
        """実機データを読み込み"""
        print("Loading real data from rosbag2...")
        if self.config.scenarios:
            print(f"  Filtering scenarios: {self.config.scenarios}")
        self.real_scenarios = load_all_scenarios(
            self.config.rosbag_base_path, 
            scenario_filter=self.config.scenarios
        )
        
        if len(self.real_scenarios) == 0:
            print("Error: No scenarios loaded!")
            return False
        
        print(f"Loaded {len(self.real_scenarios)} scenarios")
        return True
    
    def connect(self) -> bool:
        """gRPCサーバーに接続"""
        print(f"Connecting to AWSIM at {self.config.grpc_host}:{self.config.grpc_port}...")
        if self.client.connect():
            print("Connected to AWSIM")
            return True
        else:
            print("Failed to connect to AWSIM")
            return False
    
    def disconnect(self):
        """接続を切断"""
        self.client.disconnect()
        # ROS2シャットダウンはグローバルに行われるため、ここでは明示的に呼び出さない方が安全な場合もあるが
        # スクリプト終了時ならOK
        try:
            rclpy.shutdown()
        except:
            pass
        # Wait for the spin thread to return before the node is torn down; leaving it running
        # made the process crash (exit 139) at interpreter exit.
        self.ros2_thread.join(timeout=2.0)
        try:
            self.ros2.destroy_node()
        except Exception:
            pass
            
    def ros2_spin(self):
        """ROS2ノードをスピンさせる"""
        try:
            rclpy.spin(self.ros2)
        except:
            pass
    
    def _debug_save_odom_csv(self, scenario_name: str, sim_odoms, real_odom_data, trial_number: int):
        """
        デバッグ用: odomデータをcompare_bags.py互換のCSV形式で保存
        
        Args:
            scenario_name: シナリオ名
            sim_odoms: シミュレーションodomメッセージリスト
            real_odom_data: 実機odomデータ (OdomData)
            trial_number: トライアル番号
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = Path(f"debug_odom_{scenario_name}_{timestamp}_trial{trial_number}")
        output_dir.mkdir(exist_ok=True)
        
        # シミュレーションodomをCSV出力
        sim_csv = output_dir / "sim_odom.csv"
        with open(sim_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'yaw', 'vx', 'vy', 'v', 'w', 'qx', 'qy', 'qz', 'qw'])
            
            for msg in sim_odoms:
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                v = math.sqrt(vx**2 + vy**2)
                wz = msg.twist.twist.angular.z
                writer.writerow([ts, x, y, yaw, vx, vy, v, wz, q.x, q.y, q.z, q.w])
        
        # 実機odomをCSV出力
        real_csv = output_dir / "real_odom.csv"
        with open(real_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'yaw', 'vx', 'vy', 'v', 'w', 'qx', 'qy', 'qz', 'qw'])
            
            timestamps = real_odom_data.timestamps
            x = real_odom_data.positions_x
            y = real_odom_data.positions_y
            qx = real_odom_data.orientations[:, 0]
            qy = real_odom_data.orientations[:, 1]
            qz = real_odom_data.orientations[:, 2]
            qw = real_odom_data.orientations[:, 3]
            vx_arr = real_odom_data.linear_velocities[:, 0]
            vy_arr = real_odom_data.linear_velocities[:, 1]
            wz_arr = real_odom_data.angular_velocities[:, 2]
            
            for i in range(len(timestamps)):
                yaw = math.atan2(2.0 * (qw[i] * qz[i] + qx[i] * qy[i]), 1.0 - 2.0 * (qy[i]**2 + qz[i]**2))
                v = math.sqrt(vx_arr[i]**2 + vy_arr[i]**2)
                writer.writerow([timestamps[i], x[i], y[i], yaw, vx_arr[i], vy_arr[i], v, wz_arr[i], qx[i], qy[i], qz[i], qw[i]])
        
        print(f"DEBUG: Saved odom data to {output_dir}/")
        print(f"  - sim_odom.csv: {len(sim_odoms)} records")
        print(f"  - real_odom.csv: {len(timestamps)} records")
    
    def _create_params_from_trial(self, trial: Trial) -> VehicleParameters:
        """Trialからパラメータを生成"""
        params = VehicleParameters()
        
        for pr in self.parameter_ranges:
            value = pr.suggest(trial)
            setattr(params, pr.name, value)
        
        return params
    
    def _create_params_from_dict(self, param_dict: Dict[str, float]) -> VehicleParameters:
        """辞書からパラメータを生成"""
        params = VehicleParameters()
        
        for name, value in param_dict.items():
            if hasattr(params, name):
                setattr(params, name, value)
        
        return params
    
    def _evaluate_params(self, params: VehicleParameters, trial_number: int) -> Tuple[float, Dict[str, float], Dict[str, Dict]]:
        """パラメータを評価してスコアと軌跡データを返す
        
        Raises:
            EarlyStoppedException: 累積スコアが閾値を超えた場合
        """
        SCENARIO_FAILURE_PENALTY = 10.0
        EARLY_STOP_THRESHOLD = 3.0  # 累積スコアの早期打ち切り閾値
        
        # 使用するシナリオを決定
        scenarios_to_run = self.config.scenarios
        if scenarios_to_run is None:
            scenarios_to_run = list(self.real_scenarios.keys())
        
        total_score = 0.0
        scenario_scores = {}
        current_trajectories = {}
        
        for scenario_name in scenarios_to_run:
            if scenario_name not in self.real_scenarios:
                continue
            
            real_data = self.real_scenarios[scenario_name]
            
            # シミュレーション実行
            try:
                # ROS2バッファをクリア
                self.ros2.clear_odom_buffer()
                
                # Unityリセット & パラメータ適用
                self.client.run_scenario(
                    params=params,
                    cmd_vel_timestamps=[],
                    cmd_vel_linear_x=[],
                    cmd_vel_angular_z=[],
                    duration=real_data.duration + 5.0, # マージン
                    time_scale=1.0, # 等倍速必須
                    odom_sample_interval=self.config.odom_sample_interval,
                )
                
                # ROS2経由で制御実行
                self.run_ros2_scenario(real_data)
                
                # Odom取得
                sim_odoms = self.ros2.get_odom_buffer()
                
            except Exception as e:
                print(f"Simulation execution failed: {e}")
                sim_odoms = None
            
            if sim_odoms is None or len(sim_odoms) == 0:
                print(f"Scenario '{scenario_name}' failed")
                scenario_scores[scenario_name] = SCENARIO_FAILURE_PENALTY
                total_score += SCENARIO_FAILURE_PENALTY
                
                # 早期打ち切り判定
                if total_score > EARLY_STOP_THRESHOLD:
                    print(f"  ⚡ Early stopped after '{scenario_name}': cumulative {total_score:.3f} > {EARLY_STOP_THRESHOLD}")
                    raise EarlyStoppedException(total_score, scenario_scores)
                    
                continue
            
            # 評価
            try:
                result = self.evaluator.evaluate(sim_odoms, real_data.odom)
                scenario_scores[scenario_name] = result.rmse_xy
                total_score += result.rmse_xy
                
                # 軌跡データを一時保存
                trajectory_data = {
                    "sim_x": result.sim_positions_x.tolist(),
                    "sim_y": result.sim_positions_y.tolist(),
                    "sim_raw_x": result.sim_raw_x.tolist() if result.sim_raw_x is not None else [],
                    "sim_raw_y": result.sim_raw_y.tolist() if result.sim_raw_y is not None else [],
                    "sim_raw_timestamps": result.sim_raw_timestamps.tolist() if result.sim_raw_timestamps is not None else [],
                    "real_x": result.real_positions_x.tolist(),
                    "real_y": result.real_positions_y.tolist(),
                    "timestamps": result.timestamps.tolist(),
                    "rmse": result.rmse_xy,
                    "trial_number": trial_number
                }
                current_trajectories[scenario_name] = trajectory_data
                
                # デバッグモード
                if self.config.debug_mode:
                    self._debug_save_odom_csv(scenario_name, sim_odoms, real_data.odom, trial_number)
                
                # 早期打ち切り判定
                if total_score > EARLY_STOP_THRESHOLD:
                    print(f"  ⚡ Early stopped after '{scenario_name}': cumulative {total_score:.3f} > {EARLY_STOP_THRESHOLD}")
                    raise EarlyStoppedException(total_score, scenario_scores)
                
            except EarlyStoppedException:
                # 早期打ち切りは再スロー
                raise
            except Exception as e:
                print(f"Evaluation failed for '{scenario_name}': {e}")
                scenario_scores[scenario_name] = SCENARIO_FAILURE_PENALTY
                total_score += SCENARIO_FAILURE_PENALTY
                
                # 早期打ち切り判定
                if total_score > EARLY_STOP_THRESHOLD:
                    print(f"  ⚡ Early stopped after '{scenario_name}': cumulative {total_score:.3f} > {EARLY_STOP_THRESHOLD}")
                    raise EarlyStoppedException(total_score, scenario_scores)
                
        return total_score, scenario_scores, current_trajectories

    def objective(self, trial: Trial) -> float:
        """Optuna目的関数"""
        # パラメータを生成
        params = self._create_params_from_trial(trial)
        
        # 評価実行
        try:
            total_score, scenario_scores, current_trajectories = self._evaluate_params(params, trial.number)
        except EarlyStoppedException as e:
            # 早期打ち切りの場合は Pruned として記録
            print(f"Trial {trial.number}: PRUNED (score={e.score:.3f})")
            raise optuna.TrialPruned(f"Early stopped: {e.score:.3f}")
        
        # 初期軌跡として保存 (まだ無い場合)
        for name, data in current_trajectories.items():
            if name not in self.initial_trajectories:
                self.initial_trajectories[name] = data.copy()
        
        # 結果を記録
        trial_result = {
            "trial_number": trial.number,
            "score": total_score,
            "scenario_scores": scenario_scores,
            "params": {pr.name: getattr(params, pr.name) for pr in self.parameter_ranges}
        }
        self.trial_history.append(trial_result)
        
        # ベスト更新
        if total_score < self.best_score:
            self.best_score = total_score
            self.best_params = trial_result["params"].copy()
            # ベスト軌跡データを更新
            self.best_trajectories = current_trajectories.copy()
        
        # コールバック
        if self.config.on_trial_complete:
            self.config.on_trial_complete(trial.number, total_score, trial_result)
        
        # リアルタイム保存
        self._save_results_realtime()
        
        print(f"Trial {trial.number}: score={total_score:.4f} (best={self.best_score:.4f})")
        
        return total_score
    
    def _save_results_realtime(self):
        """リアルタイム更新用に結果を保存"""
        try:
            results = {
                "best_score": self.best_score,
                "best_params": self.best_params,
                "trial_history": self.trial_history,
                "trajectories": {
                    "initial": self.initial_trajectories,
                    "best": self.best_trajectories
                }
            }
            with open(self.config.results_json_path, 'w') as f:
                json.dump(results, f, indent=2)
        except Exception:
            pass  # リアルタイム保存は失敗しても続行
    
    def optimize(self) -> optuna.Study:
        """
        最適化を実行
        
        Returns:
            Optunaのstudyオブジェクト
        """
        # Studyを作成または読み込み
        study = optuna.create_study(
            study_name=self.config.study_name,
            storage=self.config.storage_path,
            direction="minimize",
            load_if_exists=True,
        )
        
        print(f"Starting optimization: {self.config.n_trials} trials")
        print(f"Study name: {self.config.study_name}")
        
        if len(study.trials) == 0:
            default_params = {p.name: p.default for p in self.parameter_ranges}
            study.enqueue_trial(default_params)
            print("Enqueued default parameters for the first trial")
        
        # DBから学習状態を完全復元 (履歴、ベスト、初期軌道)
        self._restore_state_from_db(study)
        
        try:
            study.optimize(
                self.objective,
                n_trials=self.config.n_trials,
                timeout=self.config.timeout,
                show_progress_bar=True,
            )
        except KeyboardInterrupt:
            print("Optimization interrupted by user")
        
        # 結果を表示
        print("\n=== Optimization Complete ===")
        print(f"Best score: {study.best_value:.4f}")
        print("Best parameters:")
        for name, value in study.best_params.items():
            print(f"  {name}: {value:.4f}")
        
        return study
    
    def save_results(self, output_path: str):
        """結果をJSONファイルに保存"""
        results = {
            "best_score": self.best_score,
            "best_params": self.best_params,
            "trial_history": self.trial_history,
            "trajectories": {
                "initial": self.initial_trajectories,
                "best": self.best_trajectories
            }
        }
        
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"Results saved to {output_path}")
    
    def load_results(self, input_path: str):
        """結果をJSONファイルから読み込み"""
        with open(input_path, 'r') as f:
            results = json.load(f)
        
        self.best_score = results.get("best_score", float('inf'))
        self.best_params = results.get("best_params")
        self.trial_history = results.get("trial_history", [])

    def run_ros2_scenario(self, real_data: ScenarioData):
        """
        ROS2経由でシナリオを実行
        replay_bag_on_awsim.py と完全に同じタイミングで実行する
        """
        print(f"Starting ROS2 scenario execution ({real_data.duration:.1f}s)...")
        
        # 1. ギアをDriveに入れる (replay_bag_on_awsim.py: set_gear_to_drive)
        self.ros2.set_gear_drive()
        
        # 2. システム安定化待機 (replay_bag_on_awsim.py: time.sleep(2.0))
        print("Waiting 2 seconds for system to stabilize...")
        time.sleep(2.0)
        
        # 3. バッファをクリア = 記録開始 (replay_bag_on_awsim.py: start_bag_recording)
        self.ros2.clear_odom_buffer()
        
        # 4. 記録安定化待機 (replay_bag_on_awsim.py: time.sleep(1.0) after start_bag_recording)
        print("Waiting 1 second before cmd_vel replay...")
        time.sleep(1.0)
        
        # 5. 開始時刻記録
        start_time = time.time()
        
        timestamps = real_data.cmd_vel.timestamps
        linear_x = real_data.cmd_vel.linear_x
        angular_z = real_data.cmd_vel.angular_z
        
        # replay_bag_on_awsim.py と同じロジックで再生
        # 最初のタイムスタンプからの相対時間で再生する
        # (ただしここではtimestampsは既に0始まりに正規化されているはず)
        
        for i in range(len(timestamps)):
            # 目標時刻を計算
            target_time = start_time + timestamps[i] # time_scale=1.0前提
            
            # 待機
            current_time = time.time()
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # 送信
            self.ros2.publish_cmd_vel(linear_x[i], angular_z[i])
            
        # シナリオ時間いっぱいまで待機（Odomデータ収集のため）
        remaining_time = (start_time + real_data.duration) - time.time()
        if remaining_time > 0:
            time.sleep(remaining_time)
            
        # バッファ取得とデバッグ表示
        buffer = self.ros2.get_odom_buffer()
        print(f"DEBUG: ROS2 Odom Buffer: {len(buffer)} msgs")
        if buffer:
            p0 = buffer[0].pose.pose.position
            try:
                p_end = buffer[-1].pose.pose.position
                dist = ((p_end.x - p0.x)**2 + (p_end.y - p0.y)**2)**0.5
                print(f"DEBUG: Start: ({p0.x:.2f}, {p0.y:.2f}), End: ({p_end.x:.2f}, {p_end.y:.2f}), Dist: {dist:.2f}m")
            except Exception as e:
                 print(f"DEBUG: Error calculating distance: {e}")

        # 停止
        self.ros2.stop_vehicle()
        print("Scenario execution completed.")
def run_optimization(config: Optional[OptimizerConfig] = None) -> optuna.Study:
    """
    最適化を実行するヘルパー関数
    
    Args:
        config: 最適化設定 (Noneでデフォルト使用)
        
    Returns:
        Optunaのstudyオブジェクト
    """
    if config is None:
        config = OptimizerConfig()
    
    optimizer = AWSimOptimizer(config)
    
    # 実機データ読み込み
    if not optimizer.load_real_data():
        raise RuntimeError("Failed to load real data")
    
    # gRPC接続
    if not optimizer.connect():
        raise RuntimeError("Failed to connect to AWSIM")
    
    try:
        # 最適化実行
        study = optimizer.optimize()
        
        # 結果保存
        optimizer.save_results(config.results_json_path)
        
        return study
    finally:
        optimizer.disconnect()


if __name__ == "__main__":
    # テスト実行
    config = OptimizerConfig(
        n_trials=10,
        scenarios=["straight"],  # テスト用に直進のみ
    )
    
    try:
        study = run_optimization(config)
    except Exception as e:
        print(f"Optimization failed: {e}")
