"""
軌跡プロット編集ツール

t検定で生成された軌跡データ(JSON)を読み込み、
軸範囲や線スタイルを調整してPDF/PNGを再生成します。

使用方法:
    python3 -m whill_optimizer plot-trajectory ttest_trajectory_YYYYMMDD_HHMMSS.json
"""

import json
import argparse
import numpy as np
from pathlib import Path
from typing import Optional, Tuple

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def load_trajectory_json(json_path: str) -> dict:
    """JSONから軌跡データを読み込む"""
    with open(json_path, 'r') as f:
        return json.load(f)


def plot_trajectory(data: dict, 
                   output_path: str,
                   xlim: Optional[Tuple[float, float]] = None,
                   ylim: Optional[Tuple[float, float]] = None,
                   figsize: Tuple[float, float] = (10, 8),
                   real_color: str = 'blue',
                   initial_color: str = 'red',
                   optimized_color: str = 'green',
                   real_linewidth: float = 2.0,
                   initial_linewidth: float = 1.5,
                   optimized_linewidth: float = 1.5,
                   real_linestyle: str = '-',
                   initial_linestyle: str = '--',
                   optimized_linestyle: str = '-.',
                   show_grid: bool = True,
                   equal_aspect: bool = True,
                   title: str = 'Trajectory Comparison',
                   dpi: int = 150):
    """軌跡をプロットして保存"""
    
    fig, ax = plt.subplots(figsize=figsize)
    
    # Real trajectory
    real_x = np.array(data['real']['x'])
    real_y = np.array(data['real']['y'])
    ax.plot(real_x, real_y, 
            color=real_color, 
            linewidth=real_linewidth,
            linestyle=real_linestyle,
            label=data['real']['label'])
    
    # Initial trajectory
    init_x = np.array(data['initial']['x'])
    init_y = np.array(data['initial']['y'])
    ax.plot(init_x, init_y,
            color=initial_color,
            linewidth=initial_linewidth,
            linestyle=initial_linestyle,
            label=data['initial']['label'])
    
    # Optimized trajectory
    opt_x = np.array(data['optimized']['x'])
    opt_y = np.array(data['optimized']['y'])
    ax.plot(opt_x, opt_y,
            color=optimized_color,
            linewidth=optimized_linewidth,
            linestyle=optimized_linestyle,
            label=data['optimized']['label'])
    
    # 軸設定
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(title)
    ax.legend()
    
    if show_grid:
        ax.grid(True, alpha=0.3)
    
    if equal_aspect:
        ax.set_aspect('equal')
        #ax.set_box_aspect(1)
    # 保存
    if output_path.endswith('.pdf'):
        with PdfPages(output_path) as pdf:
            pdf.savefig(fig, bbox_inches='tight', dpi=dpi)
    else:
        fig.savefig(output_path, bbox_inches='tight', dpi=dpi)
    
    plt.close(fig)
    print(f"Saved: {output_path}")


def interactive_plot(json_path: str):
    """対話形式でプロットパラメータを設定"""
    print(f"\n=== 軌跡プロット編集 ===\n")
    print(f"読み込み: {json_path}\n")
    
    data = load_trajectory_json(json_path)
    
    # データ範囲を表示
    all_x = np.concatenate([data['real']['x'], data['initial']['x'], data['optimized']['x']])
    all_y = np.concatenate([data['real']['y'], data['initial']['y'], data['optimized']['y']])
    
    print(f"データ範囲:")
    print(f"  X: {all_x.min():.2f} ~ {all_x.max():.2f} m")
    print(f"  Y: {all_y.min():.2f} ~ {all_y.max():.2f} m")
    print(f"  RMSE (Initial): {data['initial']['rmse']:.4f} m")
    print(f"  RMSE (Optimized): {data['optimized']['rmse']:.4f} m")
    print()
    
    # 出力ファイル名
    default_output = json_path.replace('.json', '_plot.pdf')
    output = input(f"出力ファイル名 [デフォルト: {default_output}]: ").strip()
    if not output:
        output = default_output
    
    # X軸範囲
    xlim = None
    xlim_str = input("X軸範囲 (例: -5,10) [Enter: 自動]: ").strip()
    if xlim_str:
        parts = xlim_str.split(',')
        if len(parts) == 2:
            xlim = (float(parts[0]), float(parts[1]))
    
    # Y軸範囲
    ylim = None
    ylim_str = input("Y軸範囲 (例: -3,3) [Enter: 自動]: ").strip()
    if ylim_str:
        parts = ylim_str.split(',')
        if len(parts) == 2:
            ylim = (float(parts[0]), float(parts[1]))
    
    # アスペクト比
    aspect = input("等比アスペクト (y/n) [デフォルト: y]: ").strip().lower()
    equal_aspect = aspect != 'n'
    
    # 図のサイズ
    figsize_str = input("図サイズ (幅,高さ) [デフォルト: 10,8]: ").strip()
    if figsize_str:
        parts = figsize_str.split(',')
        if len(parts) == 2:
            figsize = (float(parts[0]), float(parts[1]))
    else:
        figsize = (10, 8)
    
    # タイトル
    title = input("タイトル [デフォルト: Trajectory Comparison]: ").strip()
    if not title:
        title = "Trajectory Comparison"
    
    # DPI
    dpi_str = input("DPI [デフォルト: 150]: ").strip()
    dpi = int(dpi_str) if dpi_str else 150
    
    print(f"\n生成中...")
    
    plot_trajectory(
        data=data,
        output_path=output,
        xlim=xlim,
        ylim=ylim,
        figsize=figsize,
        equal_aspect=equal_aspect,
        title=title,
        dpi=dpi
    )
    
    print(f"\n✅ 完了: {output}")


def run_plot_trajectory(args=None):
    """コマンドライン引数からプロット実行"""
    parser = argparse.ArgumentParser(
        description="軌跡プロット編集ツール",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
例:
  # 対話形式
  python3 -m whill_optimizer plot-trajectory ttest_trajectory_20260107.json
  
  # コマンドライン引数で指定
  python3 -m whill_optimizer plot-trajectory data.json -o output.pdf --xlim -5,10 --ylim -3,3
        """
    )
    
    parser.add_argument("json_file", help="軌跡データJSON")
    parser.add_argument("-o", "--output", help="出力ファイル (pdf/png)")
    parser.add_argument("--xlim", help="X軸範囲 (min,max)")
    parser.add_argument("--ylim", help="Y軸範囲 (min,max)")
    parser.add_argument("--figsize", default="10,8", help="図サイズ (width,height)")
    parser.add_argument("--title", default="Trajectory Comparison", help="タイトル")
    parser.add_argument("--dpi", type=int, default=150, help="DPI")
    parser.add_argument("--no-equal-aspect", action="store_true", help="等比アスペクト無効")
    parser.add_argument("--interactive", "-i", action="store_true", help="対話モード")
    
    if args is None:
        parsed = parser.parse_args()
    else:
        parsed = parser.parse_args(args)
    
    # ファイル存在チェック
    if not Path(parsed.json_file).exists():
        print(f"Error: File not found: {parsed.json_file}")
        return
    
    # 対話モード or 引数が少ない場合
    if parsed.interactive or (not parsed.output and not parsed.xlim and not parsed.ylim):
        interactive_plot(parsed.json_file)
        return
    
    # コマンドライン引数で実行
    data = load_trajectory_json(parsed.json_file)
    
    output = parsed.output or parsed.json_file.replace('.json', '_plot.pdf')
    
    xlim = None
    if parsed.xlim:
        parts = parsed.xlim.split(',')
        xlim = (float(parts[0]), float(parts[1]))
    
    ylim = None
    if parsed.ylim:
        parts = parsed.ylim.split(',')
        ylim = (float(parts[0]), float(parts[1]))
    
    figsize_parts = parsed.figsize.split(',')
    figsize = (float(figsize_parts[0]), float(figsize_parts[1]))
    
    plot_trajectory(
        data=data,
        output_path=output,
        xlim=xlim,
        ylim=ylim,
        figsize=figsize,
        equal_aspect=not parsed.no_equal_aspect,
        title=parsed.title,
        dpi=parsed.dpi
    )


if __name__ == "__main__":
    run_plot_trajectory()
