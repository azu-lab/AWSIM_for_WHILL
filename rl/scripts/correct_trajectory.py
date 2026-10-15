#!/usr/bin/env python3
"""
correct_trajectory.py

推論で記録された軌道CSVを読み込み、初期位置・初期yawを原点に補正した
CSVファイルと軌道プロットPNGを出力する。

使い方:
  # 1ファイル指定
  python scripts/correct_trajectory.py rosbags/inference_trajectory_20260225_144400.csv

  # 複数ファイル指定
  python scripts/correct_trajectory.py rosbags/inference_trajectory_*.csv

  # 出力先を指定
  python scripts/correct_trajectory.py rosbags/inference_trajectory_20260225_144400.csv -o output/
"""

import argparse
import csv
import math
import os
import sys


def correct_records(records):
    """
    初期位置(x0,y0)を原点に、初期yawをx軸正方向(0 rad)に補正する。

    座標変換:
      1. 平行移動: (x - x0, y - y0)
      2. 回転: 初期yaw方向をx軸に揃える

    Args:
        records: dict のリスト (各レコードに 'x', 'y', 'yaw' が含まれる)

    Returns:
        補正済みレコードのリスト, (x0, y0, yaw0)
    """
    if not records:
        return records, (0, 0, 0)

    x0 = records[0]['x']
    y0 = records[0]['y']
    yaw0 = records[0]['yaw']
    cos_yaw0 = math.cos(-yaw0)
    sin_yaw0 = math.sin(-yaw0)

    corrected = []
    for r in records:
        rc = dict(r)
        dx = r['x'] - x0
        dy = r['y'] - y0
        rc['x'] = cos_yaw0 * dx - sin_yaw0 * dy
        rc['y'] = sin_yaw0 * dx + cos_yaw0 * dy
        rc['yaw'] = r['yaw'] - yaw0
        corrected.append(rc)

    return corrected, (x0, y0, yaw0)


def save_csv(records, csv_path):
    """補正済みレコードをCSVに保存。"""
    if not records:
        print("  レコードが空です。スキップ。")
        return
    keys = list(records[0].keys())
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in records:
            writer.writerow(r)
    print(f"  CSV保存: {csv_path} ({len(records)} samples)")


def plot_trajectory(records, png_path, title=''):
    """x-y軌道プロットをPNGに保存。"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib未インストール。PNGスキップ。")
        return

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
    plt.title(title or 'Corrected Trajectory')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(png_path, dpi=150)
    plt.close()
    print(f"  PNG保存: {png_path}")


def process_file(input_csv, output_dir):
    """1つのCSVファイルを処理する。"""
    print(f"\n処理中: {input_csv}")

    # CSV読み込み
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        records = []
        for row in reader:
            rec = {}
            for k, v in row.items():
                try:
                    rec[k] = float(v)
                except ValueError:
                    rec[k] = v
            records.append(rec)

    if not records:
        print("  空のCSVです。スキップ。")
        return

    print(f"  読み込み: {len(records)} samples")
    print(f"  元の初期位置: x={records[0]['x']:.4f}, y={records[0]['y']:.4f}, yaw={records[0]['yaw']:.4f} rad")

    # 補正
    corrected, (x0, y0, yaw0) = correct_records(records)
    print(f"  補正量: dx={x0:.4f}, dy={y0:.4f}, dyaw={yaw0:.4f} rad")

    # 出力ファイル名
    base = os.path.splitext(os.path.basename(input_csv))[0]
    out_csv = os.path.join(output_dir, f"{base}_corrected.csv")
    out_png = os.path.join(output_dir, f"{base}_corrected.png")

    # 保存
    save_csv(corrected, out_csv)
    plot_trajectory(corrected, out_png, title=f"Corrected: {base}")


def main():
    parser = argparse.ArgumentParser(
        description='軌道CSVの初期位置補正ツール',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python scripts/correct_trajectory.py rosbags/inference_trajectory_20260225_144400.csv
  python scripts/correct_trajectory.py rosbags/inference_trajectory_*.csv
  python scripts/correct_trajectory.py rosbags/inference_trajectory_*.csv -o output/
        """
    )
    parser.add_argument('input_csv', nargs='+', help='入力CSVファイル (複数可)')
    parser.add_argument('-o', '--output-dir', type=str, default='',
                        help='出力先ディレクトリ (デフォルト: 入力ファイルと同じ場所)')
    args = parser.parse_args()

    print("=" * 60)
    print("軌道CSV 初期位置補正ツール")
    print("=" * 60)

    for input_csv in args.input_csv:
        if not os.path.isfile(input_csv):
            print(f"\n  ファイルが見つかりません: {input_csv}")
            continue

        output_dir = args.output_dir if args.output_dir else os.path.dirname(input_csv) or '.'
        os.makedirs(output_dir, exist_ok=True)

        process_file(input_csv, output_dir)

    print(f"\n{'=' * 60}")
    print("完了")
    print("=" * 60)


if __name__ == '__main__':
    main()
