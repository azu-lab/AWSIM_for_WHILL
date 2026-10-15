"""
可視化モジュール

Optunaの最適化履歴、パラメータ重要度、軌跡比較プロットを生成します。
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # GUIなしで動作

from typing import Dict, List, Optional, Tuple
from pathlib import Path
import json

import optuna
from optuna.visualization import (
    plot_optimization_history,
    plot_param_importances,
    plot_parallel_coordinate,
    plot_contour,
)

from .evaluator import EvaluationResult
from .rosbag_loader import OdomData


class Visualizer:
    """可視化クラス"""
    
    def __init__(self, output_dir: str = "./visualization"):
        """
        Args:
            output_dir: 画像出力ディレクトリ
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    # ===== Optuna可視化 =====
    
    def plot_optimization_history_static(self, study: optuna.Study, 
                                         filename: str = "optimization_history.png"):
        """最適化履歴をプロット (静的画像)"""
        trials = study.trials
        
        values = [t.value for t in trials if t.value is not None]
        best_values = []
        current_best = float('inf')
        for v in values:
            current_best = min(current_best, v)
            best_values.append(current_best)
        
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # 全試行のスコア
        ax.scatter(range(len(values)), values, alpha=0.5, label='Trial Score', s=20)
        
        # ベストスコアの推移
        ax.plot(range(len(best_values)), best_values, 'r-', linewidth=2, label='Best Score')
        
        ax.set_xlabel('Trial Number')
        ax.set_ylabel('Score (RMSE Total)')
        ax.set_title('Optimization History')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        filepath = self.output_dir / filename
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
        return filepath
    
    def plot_param_importance_static(self, study: optuna.Study,
                                     filename: str = "param_importance.png"):
        """パラメータ重要度をプロット (静的画像)"""
        try:
            importance = optuna.importance.get_param_importances(study)
            
            names = list(importance.keys())
            values = list(importance.values())
            
            # 重要度順にソート
            sorted_idx = np.argsort(values)[::-1]
            names = [names[i] for i in sorted_idx]
            values = [values[i] for i in sorted_idx]
            
            fig, ax = plt.subplots(figsize=(12, max(6, len(names) * 0.4)))
            
            y_pos = np.arange(len(names))
            ax.barh(y_pos, values, align='center')
            ax.set_yticks(y_pos)
            ax.set_yticklabels(names)
            ax.invert_yaxis()
            ax.set_xlabel('Importance')
            ax.set_title('Parameter Importance')
            ax.grid(True, axis='x', alpha=0.3)
            
            filepath = self.output_dir / filename
            fig.savefig(filepath, dpi=150, bbox_inches='tight')
            plt.close(fig)
            print(f"Saved: {filepath}")
            return filepath
            
        except Exception as e:
            print(f"Could not compute param importance: {e}")
            return None
    
    def plot_score_distribution(self, study: optuna.Study,
                                filename: str = "score_distribution.png"):
        """スコア分布をプロット"""
        values = [t.value for t in study.trials if t.value is not None]
        
        fig, ax = plt.subplots(figsize=(10, 6))
        
        ax.hist(values, bins=30, edgecolor='black', alpha=0.7)
        ax.axvline(study.best_value, color='r', linestyle='--', 
                   linewidth=2, label=f'Best: {study.best_value:.4f}')
        
        ax.set_xlabel('Score (RMSE Total)')
        ax.set_ylabel('Frequency')
        ax.set_title('Score Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        filepath = self.output_dir / filename
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
        return filepath
    
    # ===== 軌跡比較可視化 =====
    
    def plot_trajectory_comparison(self, 
                                   eval_result: EvaluationResult,
                                   scenario_name: str,
                                   filename: Optional[str] = None):
        """軌跡比較をプロット"""
        if filename is None:
            filename = f"trajectory_{scenario_name}.png"
        
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        # 1. XY平面軌跡
        ax1 = axes[0]
        ax1.plot(eval_result.real_positions_x, eval_result.real_positions_y, 
                 'b-', linewidth=2, label='Real', alpha=0.8)
        
        # Use raw simulation data if available, otherwise fall back to interpolated
        if eval_result.sim_raw_x is not None and len(eval_result.sim_raw_x) > 0:
             ax1.plot(eval_result.sim_raw_x, eval_result.sim_raw_y, 
                     'r--', linewidth=2, label='Simulation (Raw)', alpha=0.8)
        else:
             ax1.plot(eval_result.sim_positions_x, eval_result.sim_positions_y, 
                     'r--', linewidth=2, label='Simulation (Interp)', alpha=0.8)
             
        ax1.scatter([eval_result.real_positions_x[0]], [eval_result.real_positions_y[0]], 
                    c='green', s=100, marker='o', label='Start', zorder=5)
        ax1.scatter([eval_result.real_positions_x[-1]], [eval_result.real_positions_y[-1]], 
                    c='orange', s=100, marker='s', label='End (Real)', zorder=5)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title(f'{scenario_name}: Trajectory Comparison')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # 2. 時間軸でのX/Y偏差
        ax2 = axes[1]
        error_x = eval_result.sim_positions_x - eval_result.real_positions_x
        error_y = eval_result.sim_positions_y - eval_result.real_positions_y
        t = eval_result.timestamps - eval_result.timestamps[0]
        
        ax2.plot(t, error_x, 'b-', label='X Error', alpha=0.8)
        ax2.plot(t, error_y, 'r-', label='Y Error', alpha=0.8)
        ax2.axhline(0, color='k', linestyle='-', linewidth=0.5)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Error [m]')
        ax2.set_title('Position Error over Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 総合誤差
        ax3 = axes[2]
        ax3.plot(t, eval_result.errors, 'g-', linewidth=2, alpha=0.8)
        ax3.axhline(eval_result.rmse_xy, color='r', linestyle='--', 
                    label=f'RMSE: {eval_result.rmse_xy:.4f}m')
        ax3.fill_between(t, 0, eval_result.errors, alpha=0.3)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Error [m]')
        ax3.set_title(f'Total Error (RMSE={eval_result.rmse_xy:.4f}m)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        filepath = self.output_dir / filename
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
        return filepath
    
    def plot_all_trajectories(self, 
                              eval_results: Dict[str, EvaluationResult],
                              filename: str = "all_trajectories.png"):
        """全シナリオの軌跡を1つの図にプロット"""
        n_scenarios = len(eval_results)
        if n_scenarios == 0:
            return None
        
        cols = min(2, n_scenarios)
        rows = (n_scenarios + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 6 * rows))
        
        if n_scenarios == 1:
            axes = [axes]
        else:
            axes = axes.flatten()
        
        for idx, (name, result) in enumerate(eval_results.items()):
            ax = axes[idx]
            ax.plot(result.real_positions_x, result.real_positions_y, 
                     'b-', linewidth=2, label='Real', alpha=0.8)
            
            # Use raw simulation data if available
            if result.sim_raw_x is not None and len(result.sim_raw_x) > 0:
                ax.plot(result.sim_raw_x, result.sim_raw_y, 
                        'r--', linewidth=2, label='Sim', alpha=0.8)
            else:
                ax.plot(result.sim_positions_x, result.sim_positions_y, 
                        'r--', linewidth=2, label='Sim', alpha=0.8)
            ax.scatter([result.real_positions_x[0]], [result.real_positions_y[0]], 
                       c='green', s=100, marker='o', zorder=5)
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_title(f'{name} (RMSE={result.rmse_xy:.4f}m)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
        
        # 余ったaxesを非表示
        for idx in range(n_scenarios, len(axes)):
            axes[idx].set_visible(False)
        
        plt.tight_layout()
        
        filepath = self.output_dir / filename
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
        return filepath
    
    # ===== サマリーレポート =====
    
    def generate_summary_report(self, study: optuna.Study,
                                eval_results: Optional[Dict[str, EvaluationResult]] = None,
                                filename: str = "optimization_summary.png"):
        """最適化サマリーレポートを生成"""
        n_rows = 2 if eval_results else 1
        fig, axes = plt.subplots(n_rows, 2, figsize=(14, 6 * n_rows))
        
        if n_rows == 1:
            axes = axes.reshape(1, -1)
        
        # 1. 最適化履歴
        ax = axes[0, 0]
        values = [t.value for t in study.trials if t.value is not None]
        best_values = []
        current_best = float('inf')
        for v in values:
            current_best = min(current_best, v)
            best_values.append(current_best)
        ax.scatter(range(len(values)), values, alpha=0.5, s=20)
        ax.plot(range(len(best_values)), best_values, 'r-', linewidth=2)
        ax.set_xlabel('Trial Number')
        ax.set_ylabel('Score')
        ax.set_title('Optimization History')
        ax.grid(True, alpha=0.3)
        
        # 2. ベストパラメータテーブル
        ax = axes[0, 1]
        ax.axis('off')
        
        params = study.best_params
        table_data = [[k, f"{v:.4f}"] for k, v in params.items()]
        table = ax.table(cellText=table_data, 
                         colLabels=['Parameter', 'Value'],
                         loc='center',
                         cellLoc='left')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)
        ax.set_title(f'Best Parameters (Score: {study.best_value:.4f})')
        
        # 3-4. 軌跡比較 (eval_resultsがある場合)
        if eval_results and len(eval_results) > 0:
            for idx, (name, result) in enumerate(list(eval_results.items())[:2]):
                ax = axes[1, idx]
                ax.plot(result.real_positions_x, result.real_positions_y, 
                        'b-', linewidth=2, label='Real', alpha=0.8)
                
                # Use raw simulation data if available
                if result.sim_raw_x is not None and len(result.sim_raw_x) > 0:
                    ax.plot(result.sim_raw_x, result.sim_raw_y, 
                            'r--', linewidth=2, label='Sim', alpha=0.8)
                else:
                    ax.plot(result.sim_positions_x, result.sim_positions_y, 
                            'r--', linewidth=2, label='Sim', alpha=0.8)
                ax.set_xlabel('X [m]')
                ax.set_ylabel('Y [m]')
                ax.set_title(f'{name} (RMSE={result.rmse_xy:.4f}m)')
                ax.legend()
                ax.grid(True, alpha=0.3)
                ax.axis('equal')
        
        plt.tight_layout()
        
        filepath = self.output_dir / filename
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
        return filepath


def create_visualization_report(study: optuna.Study, 
                                output_dir: str = "./visualization"):
    """可視化レポートを生成するヘルパー関数"""
    viz = Visualizer(output_dir)
    
    # 各種プロットを生成
    viz.plot_optimization_history_static(study)
    viz.plot_param_importance_static(study)
    viz.plot_score_distribution(study)
    
    print(f"\nVisualization saved to: {output_dir}/")
