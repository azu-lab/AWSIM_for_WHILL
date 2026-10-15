#!/usr/bin/env python3
"""
compare_bags.py

実機とAWSIMのROS2 bagファイルを読み込み、軌道を比較するスクリプト

使い方:
    python3 compare_bags.py <real_bag_dir> <awsim_bag_dir>
    
    例:
    python3 compare_bags.py whill_cmd_straight_20251216_130331/ awsim_replay_straight_20251217_154922/

出力:
    - comparison_<pattern>_<timestamp>.png - 軌道比較プロット
    - comparison_<pattern>_<timestamp>.csv - 比較データ

動作:
    - 実機のbagから /whill/states/odom を読み込み
    - AWSIMのbagから /ground_truth/odom を読み込み
    - 両方の軌道を車両ローカル座標系に変換
    - 軌道、速度、角速度、ホイール速度を比較
"""

import os
import sys
import math
import csv
from pathlib import Path
from datetime import datetime
import numpy as np
import shutil

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import matplotlib.pyplot as plt
    import seaborn as sns
except Exception as e:
    print("ERROR: could not import required modules.")
    print(e)
    sys.exit(1)


def read_odom_from_bag(bag_path, topic_name):
    """
    rosbagからオドメトリデータを読み込む
    
    Args:
        bag_path: bagファイルのパス
        topic_name: オドメトリトピック名
    
    Returns:
        list of dict: オドメトリレコードのリスト
    """
    print(f'\nReading bag: {bag_path}')
    print(f'Target topic: {topic_name}')
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # トピックとメッセージタイプのマッピング
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f'ERROR: Topic {topic_name} not found in bag file!')
        print(f'Available topics: {list(type_map.keys())}')
        return []
    
    records = []
    count = 0
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == topic_name:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            # タイムスタンプ
            stamp = msg.header.stamp
            ts = stamp.sec + stamp.nanosec * 1e-9
            
            # 位置
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # クォータニオン
            q = msg.pose.pose.orientation
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            
            # ヨー角を計算
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy**2 + qz**2)
            )
            
            # 速度
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            
            # 合成速度
            v = math.sqrt(vx**2 + vy**2)
            
            record = {
                'ts': ts,
                'x': x,
                'y': y,
                'yaw': yaw,
                'v': v,
                'vx': vx,
                'vy': vy,
                'w': wz,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw,
            }
            
            records.append(record)
            count += 1
    
    print(f'Loaded {count} messages from {topic_name}')
    
    if count > 0:
        duration = records[-1]['ts'] - records[0]['ts']
        print(f'Duration: {duration:.2f} seconds')
        print(f'Frequency: {count / duration:.1f} Hz')
    
    return records


def read_wheel_vel_from_bag(bag_path, topic_name):
    """
    rosbagからホイール速度データを読み込む
    
    Args:
        bag_path: bagファイルのパス
        topic_name: ホイール速度トピック名
    
    Returns:
        list of dict: ホイール速度レコードのリスト
    """
    print(f'Reading wheel velocities from: {topic_name}')
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # トピックとメッセージタイプのマッピング
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f'WARNING: Topic {topic_name} not found in bag file!')
        return []
    
    records = []
    count = 0
    
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == topic_name:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            # タイムスタンプはナノ秒単位
            ts = timestamp / 1e9
            
            # data[0]: 左後輪速度, data[1]: 右後輪速度
            if len(msg.data) >= 2:
                record = {
                    'ts': ts,
                    'wheel_l': msg.data[0],
                    'wheel_r': msg.data[1],
                }
                records.append(record)
                count += 1
    
    if count > 0:
        print(f'Loaded {count} wheel velocity messages')
    
    return records


def transform_to_vehicle_frame(records):
    """
    グローバル座標系から車両ローカル座標系に変換
    
    Args:
        records: オドメトリレコードのリスト
    
    Returns:
        list of dict: 変換後のレコードのリスト
    """
    if not records:
        return []
    
    # 初期位置と姿勢
    x0 = records[0]['x']
    y0 = records[0]['y']
    yaw0 = records[0]['yaw']
    
    print(f'Initial pose: x={x0:.3f}, y={y0:.3f}, yaw={math.degrees(yaw0):.2f}°')
    
    cos_yaw0 = math.cos(-yaw0)
    sin_yaw0 = math.sin(-yaw0)
    
    transformed = []
    
    for r in records:
        # グローバル座標系での相対位置
        dx_global = r['x'] - x0
        dy_global = r['y'] - y0
        
        # 車両ローカル座標系に回転
        x_local = cos_yaw0 * dx_global - sin_yaw0 * dy_global
        y_local = sin_yaw0 * dx_global + cos_yaw0 * dy_global
        
        # ヨー角も相対角度に
        yaw_local = r['yaw'] - yaw0
        
        # -π ~ π に正規化
        while yaw_local > math.pi:
            yaw_local -= 2 * math.pi
        while yaw_local < -math.pi:
            yaw_local += 2 * math.pi
        
        transformed_record = {
            'ts': r['ts'],
            'x': x_local,
            'y': y_local,
            'yaw': yaw_local,
            'v': r['v'],
            'vx': r['vx'],
            'vy': r['vy'],
            'w': r['w'],
            'wheel_l': r.get('wheel_l', 0.0),
            'wheel_r': r.get('wheel_r', 0.0),
        }
        
        transformed.append(transformed_record)
    
    return transformed


def merge_wheel_velocities(odom_records, wheel_records):
    """
    オドメトリレコードにホイール速度を追加（最近接補間）
    
    Args:
        odom_records: オドメトリレコードのリスト
        wheel_records: ホイール速度レコードのリスト
    
    Returns:
        list of dict: ホイール速度が追加されたレコードのリスト
    """
    if not wheel_records:
        # ホイール速度がない場合は0で埋める
        for r in odom_records:
            r['wheel_l'] = 0.0
            r['wheel_r'] = 0.0
        return odom_records
    
    # ホイール速度のタイムスタンプでソート
    wheel_records_sorted = sorted(wheel_records, key=lambda x: x['ts'])
    wheel_times = [r['ts'] for r in wheel_records_sorted]
    
    for odom_rec in odom_records:
        t = odom_rec['ts']
        
        # 最近接のホイール速度を見つける
        idx = np.searchsorted(wheel_times, t)
        
        if idx == 0:
            wheel_rec = wheel_records_sorted[0]
        elif idx == len(wheel_records_sorted):
            wheel_rec = wheel_records_sorted[-1]
        else:
            # 前後の近い方を選択
            if abs(wheel_times[idx] - t) < abs(wheel_times[idx-1] - t):
                wheel_rec = wheel_records_sorted[idx]
            else:
                wheel_rec = wheel_records_sorted[idx-1]
        
        odom_rec['wheel_l'] = wheel_rec['wheel_l']
        odom_rec['wheel_r'] = wheel_rec['wheel_r']
    
    return odom_records


def synchronize_time(real_records, sim_records):
    """
    実機とシミュレータのタイムスタンプを正規化
    両方とも開始時刻を0にする
    
    Args:
        real_records: 実機レコード
        sim_records: シミュレータレコード
    
    Returns:
        tuple: (正規化された実機レコード, 正規化されたシミュレータレコード)
    """
    if not real_records or not sim_records:
        return real_records, sim_records
    
    # 開始時刻を0にする
    t0_real = real_records[0]['ts']
    t0_sim = sim_records[0]['ts']
    
    for r in real_records:
        r['ts'] -= t0_real
    
    for r in sim_records:
        r['ts'] -= t0_sim
    
    print(f'\nTime synchronization:')
    print(f'  Real duration: {real_records[-1]["ts"]:.2f}s')
    print(f'  Sim duration:  {sim_records[-1]["ts"]:.2f}s')
    
    return real_records, sim_records


def compute_rmse_metrics(real_records, sim_records):
    """
    RMSE誤差を計算
    
    Args:
        real_records: 実機レコード
        sim_records: シミュレータレコード
    
    Returns:
        dict: RMSE値の辞書
    """
    if not real_records or not sim_records:
        print('ERROR: Cannot compute RMSE - missing data')
        return None
    
    # サンプリングレートを確認
    real_duration = real_records[-1]['ts'] - real_records[0]['ts']
    sim_duration = sim_records[-1]['ts'] - sim_records[0]['ts']
    real_freq = len(real_records) / real_duration
    sim_freq = len(sim_records) / sim_duration
    
    print(f'\nSampling rates:')
    print(f'  Real: {real_freq:.1f} Hz ({len(real_records)} samples)')
    print(f'  Sim:  {sim_freq:.1f} Hz ({len(sim_records)} samples)')
    
    # 短い方の時間区間に合わせる
    min_duration = min(real_duration, sim_duration)
    print(f'\nUsing common duration: {min_duration:.2f} seconds')
    
    # 時間区間でフィルタリング
    real_filtered = [r for r in real_records if r['ts'] <= min_duration]
    sim_filtered = [r for r in sim_records if r['ts'] <= min_duration]
    
    # サンプリングレートの低い方に合わせてダウンサンプリング
    if real_freq < sim_freq:
        # 実機の時間軸にシミュレータを補間
        print(f'Downsampling to Real frequency ({real_freq:.1f} Hz)')
        base_records = real_filtered
        target_records = sim_filtered
        base_is_real = True
    else:
        # シミュレータの時間軸に実機を補間
        print(f'Downsampling to Sim frequency ({sim_freq:.1f} Hz)')
        base_records = sim_filtered
        target_records = real_filtered
        base_is_real = False
    
    # 線形補間
    base_times = np.array([r['ts'] for r in base_records])
    target_times = np.array([r['ts'] for r in target_records])
    
    target_x_interp = np.interp(base_times, target_times, [r['x'] for r in target_records])
    target_y_interp = np.interp(base_times, target_times, [r['y'] for r in target_records])
    target_yaw_interp = np.interp(base_times, target_times, [r['yaw'] for r in target_records])
    target_v_interp = np.interp(base_times, target_times, [r['v'] for r in target_records])
    target_vx_interp = np.interp(base_times, target_times, [r['vx'] for r in target_records])
    target_vy_interp = np.interp(base_times, target_times, [r['vy'] for r in target_records])
    target_w_interp = np.interp(base_times, target_times, [r['w'] for r in target_records])
    
    # ベースデータ抽出
    base_x = np.array([r['x'] for r in base_records])
    base_y = np.array([r['y'] for r in base_records])
    base_yaw = np.array([r['yaw'] for r in base_records])
    base_v = np.array([r['v'] for r in base_records])
    base_vx = np.array([r['vx'] for r in base_records])
    base_vy = np.array([r['vy'] for r in base_records])
    base_w = np.array([r['w'] for r in base_records])
    
    # Real と Sim を正しく割り当て
    if base_is_real:
        real_x, real_y, real_yaw, real_v, real_vx, real_vy, real_w = base_x, base_y, base_yaw, base_v, base_vx, base_vy, base_w
        sim_x, sim_y, sim_yaw, sim_v, sim_vx, sim_vy, sim_w = target_x_interp, target_y_interp, target_yaw_interp, target_v_interp, target_vx_interp, target_vy_interp, target_w_interp
    else:
        sim_x, sim_y, sim_yaw, sim_v, sim_vx, sim_vy, sim_w = base_x, base_y, base_yaw, base_v, base_vx, base_vy, base_w
        real_x, real_y, real_yaw, real_v, real_vx, real_vy, real_w = target_x_interp, target_y_interp, target_yaw_interp, target_v_interp, target_vx_interp, target_vy_interp, target_w_interp
    
    # 1. Trajectory RMSE (2D Euclidean distance)
    trajectory_errors = np.sqrt((real_x - sim_x)**2 + (real_y - sim_y)**2)
    rmse_trajectory = np.sqrt(np.mean(trajectory_errors**2))
    
    # 2. Yaw RMSE (angle wrapping)
    yaw_diff = real_yaw - sim_yaw
    # Wrap to [-π, π]
    yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
    rmse_yaw = np.sqrt(np.mean(yaw_diff**2))
    rmse_yaw_deg = np.rad2deg(rmse_yaw)
    
    # 3. Linear Velocity RMSE (combined)
    v_diff = real_v - sim_v
    rmse_v = np.sqrt(np.mean(v_diff**2))
    
    # 4. Forward Velocity RMSE (vx)
    vx_diff = real_vx - sim_vx
    rmse_vx = np.sqrt(np.mean(vx_diff**2))
    
    # 5. Lateral Velocity RMSE (vy)
    vy_diff = real_vy - sim_vy
    rmse_vy = np.sqrt(np.mean(vy_diff**2))
    
    # 6. Angular Velocity RMSE
    w_diff = real_w - sim_w
    rmse_w = np.sqrt(np.mean(w_diff**2))
    
    rmse_dict = {
        'trajectory': rmse_trajectory,
        'yaw_rad': rmse_yaw,
        'yaw_deg': rmse_yaw_deg,
        'linear_velocity': rmse_v,
        'forward_velocity': rmse_vx,
        'lateral_velocity': rmse_vy,
        'angular_velocity': rmse_w,
        'num_samples': len(base_records),
        'duration': min_duration,
    }
    
    return rmse_dict


def print_rmse_table(rmse_dict):
    """
    RMSE結果をテーブル形式で表示
    
    Args:
        rmse_dict: RMSE値の辞書
    """
    if not rmse_dict:
        return
    
    print('\n' + '='*70)
    print('=== RMSE Error Metrics ===')
    print('='*70)
    print(f'Evaluation Duration: {rmse_dict["duration"]:.2f} seconds')
    print(f'Number of Samples:   {rmse_dict["num_samples"]}')
    print()
    print(f'{"Metric":<30} {"RMSE":>15} {"Unit":>10}')
    print('-'*70)
    print(f'{"Trajectory (2D)":<30} {rmse_dict["trajectory"]:>15.4f} {"m":>10}')
    print(f'{"Yaw Angle":<30} {rmse_dict["yaw_deg"]:>15.4f} {"deg":>10}')
    print(f'{"Linear Velocity (Combined)":<30} {rmse_dict["linear_velocity"]:>15.4f} {"m/s":>10}')
    print(f'{"Forward Velocity (vx)":<30} {rmse_dict["forward_velocity"]:>15.4f} {"m/s":>10}')
    print(f'{"Lateral Velocity (vy)":<30} {rmse_dict["lateral_velocity"]:>15.4f} {"m/s":>10}')
    print(f'{"Angular Velocity":<30} {rmse_dict["angular_velocity"]:>15.4f} {"rad/s":>10}')
    print('='*70)


def save_comparison_csv(real_records, sim_records, filename):
    """
    比較データをCSVに保存
    
    Args:
        real_records: 実機レコード
        sim_records: シミュレータレコード
        filename: 出力ファイル名
    """
    # 時間軸を統一（実機の時間軸に合わせてシミュレータを線形補間）
    if not real_records or not sim_records:
        print('ERROR: Cannot save comparison CSV - missing data')
        return
    
    real_times = [r['ts'] for r in real_records]
    
    # シミュレータデータを実機の時間軸に線形補間
    sim_times = [r['ts'] for r in sim_records]
    sim_x = [r['x'] for r in sim_records]
    sim_y = [r['y'] for r in sim_records]
    sim_yaw = [r['yaw'] for r in sim_records]
    sim_v = [r['v'] for r in sim_records]
    sim_vx = [r['vx'] for r in sim_records]
    sim_vy = [r['vy'] for r in sim_records]
    sim_w = [r['w'] for r in sim_records]
    sim_wheel_l = [r['wheel_l'] for r in sim_records]
    sim_wheel_r = [r['wheel_r'] for r in sim_records]
    
    sim_x_interp = np.interp(real_times, sim_times, sim_x)
    sim_y_interp = np.interp(real_times, sim_times, sim_y)
    sim_yaw_interp = np.interp(real_times, sim_times, sim_yaw)
    sim_v_interp = np.interp(real_times, sim_times, sim_v)
    sim_vx_interp = np.interp(real_times, sim_times, sim_vx)
    sim_vy_interp = np.interp(real_times, sim_times, sim_vy)
    sim_w_interp = np.interp(real_times, sim_times, sim_w)
    sim_wheel_l_interp = np.interp(real_times, sim_times, sim_wheel_l)
    sim_wheel_r_interp = np.interp(real_times, sim_times, sim_wheel_r)
    
    # CSV出力
    with open(filename, 'w', newline='') as f:
        fieldnames = [
            'time',
            'real_x', 'real_y', 'real_yaw', 'real_v', 'real_vx', 'real_vy', 'real_w', 'real_wheel_l', 'real_wheel_r',
            'sim_x', 'sim_y', 'sim_yaw', 'sim_v', 'sim_vx', 'sim_vy', 'sim_w', 'sim_wheel_l', 'sim_wheel_r'
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        for i, real_rec in enumerate(real_records):
            row = {
                'time': real_rec['ts'],
                'real_x': real_rec['x'],
                'real_y': real_rec['y'],
                'real_yaw': real_rec['yaw'],
                'real_v': real_rec['v'],
                'real_vx': real_rec['vx'],
                'real_vy': real_rec['vy'],
                'real_w': real_rec['w'],
                'real_wheel_l': real_rec['wheel_l'],
                'real_wheel_r': real_rec['wheel_r'],
                'sim_x': sim_x_interp[i],
                'sim_y': sim_y_interp[i],
                'sim_yaw': sim_yaw_interp[i],
                'sim_v': sim_v_interp[i],
                'sim_vx': sim_vx_interp[i],
                'sim_vy': sim_vy_interp[i],
                'sim_w': sim_w_interp[i],
                'sim_wheel_l': sim_wheel_l_interp[i],
                'sim_wheel_r': sim_wheel_r_interp[i],
            }
            writer.writerow(row)
    
    print(f'\nSaved comparison data to: {filename}')


def save_metric_pdfs(rmse_dict, output_dir):
    """
    各比較項目（RMSE）を1枚ずつのPDFに保存
    """
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print('Plotting dependencies not found for SVG export.')
        return

    # Find the comparison CSV saved earlier by save_comparison_csv
    csv_candidates = [p for p in os.listdir(output_dir) if p.endswith('.csv') and p.startswith('comparison_')]
    if not csv_candidates:
        print('No comparison CSV found in output directory; cannot generate per-metric SVGs.')
        return

    csv_path = os.path.join(output_dir, csv_candidates[0])

    # Load CSV columns into numpy arrays
    times = []
    real = {'x': [], 'y': [], 'yaw': [], 'v': [], 'vx': [], 'vy': [], 'w': []}
    sim = {'x': [], 'y': [], 'yaw': [], 'v': [], 'vx': [], 'vy': [], 'w': []}

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time']))
            real['x'].append(float(row['real_x']))
            real['y'].append(float(row['real_y']))
            real['yaw'].append(float(row['real_yaw']))
            real['v'].append(float(row['real_v']))
            real['vx'].append(float(row['real_vx']))
            real['vy'].append(float(row['real_vy']))
            real['w'].append(float(row['real_w']))

            sim['x'].append(float(row['sim_x']))
            sim['y'].append(float(row['sim_y']))
            sim['yaw'].append(float(row['sim_yaw']))
            sim['v'].append(float(row['sim_v']))
            sim['vx'].append(float(row['sim_vx']))
            sim['vy'].append(float(row['sim_vy']))
            sim['w'].append(float(row['sim_w']))

    times = np.array(times)
    for k in real:
        real[k] = np.array(real[k])
        sim[k] = np.array(sim[k])

    # Precompute trajectory error time series
    traj_err = np.sqrt((real['x'] - sim['x'])**2 + (real['y'] - sim['y'])**2)

    # Trajectory SVG (XY comparison) - unify style with other metric SVGs
    traj_svg = os.path.join(output_dir, 'trajectory.svg')
    # Use square aspect for individual SVGs: width:height = 1:1
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(real['x'], real['y'], 'b-', linewidth=2, label='Real')
    ax.plot(sim['x'], sim['y'], 'r--', linewidth=2, label='AWSIM')
    ax.scatter(real['x'][0], real['y'][0], color='green', s=80, marker='*', label='Start')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    # Use ASCII colon and consistent title format
    ax.set_title(f'<Trajectory(RMSE:{rmse_dict.get("trajectory", 0.0):.4f} m)>')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.subplots_adjust(top=0.92, bottom=0.14, left=0.08, right=0.98)
    fig.savefig(traj_svg, format='svg', bbox_inches='tight')
    plt.close(fig)
    print(f'Saved trajectory SVG: {traj_svg}')

    # Metric SVGs: one file per metric (time-series only)
    metric_specs = [
        ('yaw_deg', np.rad2deg(real['yaw']), np.rad2deg(sim['yaw']), 'Yaw (deg)', rmse_dict.get('yaw_deg', 0.0)),
        ('linear_velocity', real['v'], sim['v'], 'Linear Velocity (m/s)', rmse_dict.get('linear_velocity', 0.0)),
        ('forward_velocity', real['vx'], sim['vx'], 'Forward Velocity vx (m/s)', rmse_dict.get('forward_velocity', 0.0)),
        ('lateral_velocity', real['vy'], sim['vy'], 'Lateral Velocity vy (m/s)', rmse_dict.get('lateral_velocity', 0.0)),
        ('angular_velocity', real['w'], sim['w'], 'Angular Velocity (rad/s)', rmse_dict.get('angular_velocity', 0.0)),
        ('trajectory_error', traj_err, None, 'Trajectory Error (m)', rmse_dict.get('trajectory', 0.0)),
    ]

    for key, y_real, y_sim, ylabel, rmse_val in metric_specs:
        # Use square aspect for individual SVGs: width:height = 1:1
        fig, ax = plt.subplots(figsize=(6, 6))
        if y_sim is None:
            ax.plot(times, y_real, color='magenta', linewidth=2, label=ylabel)
        else:
            ax.plot(times, y_real, 'b-', linewidth=2, label='Real')
            ax.plot(times, y_sim, 'r--', linewidth=2, label='AWSIM')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid(alpha=0.3)
        # Title: use ASCII colon and consistent RMSE placement
        ax.set_title(f'<{ylabel}(RMSE:{rmse_val:.4f})>')
        fig.subplots_adjust(top=0.92, bottom=0.14, left=0.08, right=0.98)

        out_svg = os.path.join(output_dir, f'{key}.svg')
        fig.savefig(out_svg, format='svg', bbox_inches='tight')
        plt.close(fig)
        print(f'Saved metric SVG: {out_svg}')


def plot_comparison(real_records, sim_records, output_file, rmse_dict=None):
    """
    実機とシミュレータの軌道を比較プロット
    
    Args:
        real_records: 実機レコード
        sim_records: シミュレータレコード
        output_file: 出力ファイル名
        rmse_dict: RMSE値の辞書（オプション）
    """
    sns.set_style("whitegrid")
    plt.rcParams['font.size'] = 10
    
    fig = plt.figure(figsize=(20, 18))
    gs = fig.add_gridspec(5, 3, hspace=0.35, wspace=0.3, top=0.95, bottom=0.05)
    
    # データ抽出
    real_times = [r['ts'] for r in real_records]
    real_x = [r['x'] for r in real_records]
    real_y = [r['y'] for r in real_records]
    real_yaw = [r['yaw'] for r in real_records]
    real_v = [r['v'] for r in real_records]
    real_vx = [r['vx'] for r in real_records]
    real_vy = [r['vy'] for r in real_records]
    real_w = [r['w'] for r in real_records]
    real_wheel_l = [r['wheel_l'] for r in real_records]
    real_wheel_r = [r['wheel_r'] for r in real_records]
    
    sim_times = [r['ts'] for r in sim_records]
    sim_x = [r['x'] for r in sim_records]
    sim_y = [r['y'] for r in sim_records]
    sim_yaw = [r['yaw'] for r in sim_records]
    sim_v = [r['v'] for r in sim_records]
    sim_vx = [r['vx'] for r in sim_records]
    sim_vy = [r['vy'] for r in sim_records]
    sim_w = [r['w'] for r in sim_records]
    sim_wheel_l = [r['wheel_l'] for r in sim_records]
    sim_wheel_r = [r['wheel_r'] for r in sim_records]
    
    # 1. XY軌道比較 (大きく表示)
    ax1 = fig.add_subplot(gs[0:2, 0:2])
    ax1.plot(real_x, real_y, 'b-', linewidth=2, label='Real Robot', alpha=0.8)
    ax1.plot(sim_x, sim_y, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax1.scatter(real_x[0], real_y[0], color='green', s=200, marker='*', label='Start', zorder=5)
    ax1.scatter(real_x[-1], real_y[-1], color='blue', s=150, marker='o', label='Real End', zorder=5)
    ax1.scatter(sim_x[-1], sim_y[-1], color='red', s=150, marker='s', label='Sim End', zorder=5)
    ax1.set_xlabel('X Position (m) - Forward', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y Position (m) - Left', fontsize=12, fontweight='bold')
    
    # RMSEをタイトルに追加
    if rmse_dict:
        title = f'Trajectory Comparison (RMSE: {rmse_dict["trajectory"]:.4f} m)'
    else:
        title = 'Trajectory Comparison (Vehicle-Centric Frame)'
    ax1.set_title(title, fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11)
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)
    
    # 2. X位置
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(real_times, real_x, 'b-', linewidth=2, label='Real', alpha=0.8)
    ax2.plot(sim_times, sim_x, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylabel('X Position (m)', fontsize=10)
    ax2.set_title('X Position', fontsize=11, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # 3. Y位置
    ax3 = fig.add_subplot(gs[1, 2])
    ax3.plot(real_times, real_y, 'b-', linewidth=2, label='Real', alpha=0.8)
    ax3.plot(sim_times, sim_y, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Y Position (m)', fontsize=10)
    ax3.set_title('Y Position', fontsize=11, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # 4. ヨー角
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(real_times, np.rad2deg(real_yaw), 'b-', linewidth=2, label='Real', alpha=0.8)
    ax4.plot(sim_times, np.rad2deg(sim_yaw), 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Yaw Angle (deg)', fontsize=10)
    if rmse_dict:
        yaw_title = f'Yaw Angle (RMSE: {rmse_dict["yaw_deg"]:.4f}°)'
    else:
        yaw_title = 'Yaw Angle'
    ax4.set_title(yaw_title, fontsize=11, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # 5. 線速度（合成）
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.plot(real_times, real_v, 'b-', linewidth=2, label='Real', alpha=0.8)
    ax5.plot(sim_times, sim_v, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax5.set_xlabel('Time (s)', fontsize=10)
    ax5.set_ylabel('Linear Velocity (m/s)', fontsize=10)
    if rmse_dict:
        v_title = f'Linear Velocity (RMSE: {rmse_dict["linear_velocity"]:.4f} m/s)'
    else:
        v_title = 'Linear Velocity (Combined)'
    ax5.set_title(v_title, fontsize=11, fontweight='bold')
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)
    
    # 6. 角速度
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.plot(real_times, real_w, 'b-', linewidth=2, label='Real', alpha=0.8)
    ax6.plot(sim_times, sim_w, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax6.set_xlabel('Time (s)', fontsize=10)
    ax6.set_ylabel('Angular Velocity (rad/s)', fontsize=10)
    if rmse_dict:
        w_title = f'Angular Velocity (RMSE: {rmse_dict["angular_velocity"]:.4f} rad/s)'
    else:
        w_title = 'Angular Velocity'
    ax6.set_title(w_title, fontsize=11, fontweight='bold')
    ax6.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax6.legend(fontsize=9)
    ax6.grid(True, alpha=0.3)
    
    # 7. 前方速度（vx）
    ax7 = fig.add_subplot(gs[3, 0])
    ax7.plot(real_times, real_vx, 'b-', linewidth=2, label='Real', alpha=0.8)
    ax7.plot(sim_times, sim_vx, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax7.set_xlabel('Time (s)', fontsize=10)
    ax7.set_ylabel('Forward Velocity (m/s)', fontsize=10)
    if rmse_dict:
        vx_title = f'Forward Velocity (RMSE: {rmse_dict["forward_velocity"]:.4f} m/s)'
    else:
        vx_title = 'Forward Velocity (vx)'
    ax7.set_title(vx_title, fontsize=11, fontweight='bold')
    ax7.legend(fontsize=9)
    ax7.grid(True, alpha=0.3)
    
    # 8. 横方向速度（vy）
    ax8 = fig.add_subplot(gs[3, 1])
    ax8.plot(real_times, real_vy, 'b-', linewidth=2, label='Real', alpha=0.8)
    ax8.plot(sim_times, sim_vy, 'r--', linewidth=2, label='AWSIM', alpha=0.8)
    ax8.set_xlabel('Time (s)', fontsize=10)
    ax8.set_ylabel('Lateral Velocity (m/s)', fontsize=10)
    if rmse_dict:
        vy_title = f'Lateral Velocity (RMSE: {rmse_dict["lateral_velocity"]:.4f} m/s)'
    else:
        vy_title = 'Lateral Velocity (vy)'
    ax8.set_title(vy_title, fontsize=11, fontweight='bold')
    ax8.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax8.legend(fontsize=9)
    ax8.grid(True, alpha=0.3)
    
    # 9. RMSE結果テーブルを追加
    if rmse_dict:
        ax9 = fig.add_subplot(gs[4, :])
        ax9.axis('off')
        
        # テーブルデータ
        table_data = [
            ['Metric', 'RMSE', 'Unit'],
            ['Trajectory (2D)', f'{rmse_dict["trajectory"]:.4f}', 'm'],
            ['Yaw Angle', f'{rmse_dict["yaw_deg"]:.4f}', 'deg'],
            ['Linear Velocity (Combined)', f'{rmse_dict["linear_velocity"]:.4f}', 'm/s'],
            ['Forward Velocity (vx)', f'{rmse_dict["forward_velocity"]:.4f}', 'm/s'],
            ['Lateral Velocity (vy)', f'{rmse_dict["lateral_velocity"]:.4f}', 'm/s'],
            ['Angular Velocity', f'{rmse_dict["angular_velocity"]:.4f}', 'rad/s'],
        ]
        
        table = ax9.table(cellText=table_data, cellLoc='center', loc='center',
                         colWidths=[0.4, 0.3, 0.3])
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1, 2.2)
        
        # ヘッダー行のスタイル
        for i in range(3):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # データ行のスタイル（交互に色を変える）
        for i in range(1, 7):
            for j in range(3):
                if i % 2 == 0:
                    table[(i, j)].set_facecolor('#f0f0f0')
                else:
                    table[(i, j)].set_facecolor('white')
        
        ax9.text(0.5, 0.88, f'Evaluation Duration: {rmse_dict["duration"]:.2f}s | Samples: {rmse_dict["num_samples"]}',
                ha='center', va='center', transform=ax9.transAxes, fontsize=10, style='italic')
    
    plt.suptitle('Real Robot vs AWSIM Simulation Comparison', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    plt.savefig(output_file, dpi=200, bbox_inches='tight')
    # Also save as SVG
    svg_out = os.path.splitext(output_file)[0] + '.svg'
    plt.savefig(svg_out, format='svg', bbox_inches='tight')
    print(f'Saved comparison plot to: {output_file} and {svg_out}')


def extract_pattern_name(bag_dir):
    """bagディレクトリ名からパターン名を抽出"""
    base_name = os.path.basename(bag_dir.rstrip('/'))
    parts = base_name.split('_')
    
    # whill_cmd_<pattern>_<date>_<time> or awsim_replay_<pattern>_<date>_<time>
    if len(parts) >= 5:
        if parts[0] == 'whill' and parts[1] == 'cmd':
            # Find date (8 digits)
            for i in range(2, len(parts)):
                if len(parts[i]) == 8 and parts[i].isdigit():
                    return '_'.join(parts[2:i])
        elif parts[0] == 'awsim' and parts[1] == 'replay':
            # Find date (8 digits)
            for i in range(2, len(parts)):
                if len(parts[i]) == 8 and parts[i].isdigit():
                    return '_'.join(parts[2:i])
    
    return 'unknown'


def main():
    if len(sys.argv) < 3:
        print('Usage: python3 compare_bags.py <real_bag_directory> <awsim_bag_directory>')
        print('Example: python3 compare_bags.py whill_cmd_straight_20251216_130331/ awsim_replay_straight_20251217_154922/')
        sys.exit(1)
    
    real_bag_path = sys.argv[1]
    awsim_bag_path = sys.argv[2]
    
    # Check paths
    if not os.path.exists(real_bag_path):
        print(f'ERROR: Real bag path does not exist: {real_bag_path}')
        sys.exit(1)
    
    if not os.path.exists(awsim_bag_path):
        print(f'ERROR: AWSIM bag path does not exist: {awsim_bag_path}')
        sys.exit(1)
    
    print('='*70)
    print('=== Real Robot vs AWSIM Bag Comparison ===')
    print('='*70)
    print(f'Real bag:  {real_bag_path}')
    print(f'AWSIM bag: {awsim_bag_path}')
    
    # パターン名を抽出
    pattern_name = extract_pattern_name(real_bag_path)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # 実機データを読み込み
    print('\n--- Loading Real Robot Data ---')
    real_odom = read_odom_from_bag(real_bag_path, '/whill/odom')
    real_wheel = read_wheel_vel_from_bag(real_bag_path, '/whill/states/velocities')
    
    if not real_odom:
        print('ERROR: No real robot odometry data found!')
        sys.exit(1)
    
    # AWSIMデータを読み込み
    print('\n--- Loading AWSIM Data ---')
    awsim_odom = read_odom_from_bag(awsim_bag_path, '/ground_truth/odom')
    awsim_wheel = read_wheel_vel_from_bag(awsim_bag_path, '/whill/wheel_velocities_sim')
    
    if not awsim_odom:
        print('ERROR: No AWSIM odometry data found!')
        sys.exit(1)
    
    # ホイール速度をマージ
    real_odom = merge_wheel_velocities(real_odom, real_wheel)
    awsim_odom = merge_wheel_velocities(awsim_odom, awsim_wheel)
    
    # 車両ローカル座標系に変換
    print('\n--- Transforming to Vehicle-Centric Frame ---')
    real_local = transform_to_vehicle_frame(real_odom)
    awsim_local = transform_to_vehicle_frame(awsim_odom)
    
    # 時刻を正規化
    real_local, awsim_local = synchronize_time(real_local, awsim_local)
    
    # RMSE計算
    print('\n--- Computing RMSE Metrics ---')
    rmse_dict = compute_rmse_metrics(real_local, awsim_local)
    
    # RMSE結果をテーブル表示
    if rmse_dict:
        print_rmse_table(rmse_dict)
    
    # 出力フォルダを作成
    out_dir = f'comparison_{pattern_name}_{timestamp}'
    os.makedirs(out_dir, exist_ok=True)

    # 出力ファイル名（フォルダ内に保存）
    csv_file = os.path.join(out_dir, f'comparison_{pattern_name}_{timestamp}.csv')
    png_file = os.path.join(out_dir, f'comparison_{pattern_name}_{timestamp}.png')

    # CSVに保存
    save_comparison_csv(real_local, awsim_local, csv_file)

    # プロット生成
    print('\n--- Generating Comparison Plot ---')
    plot_comparison(real_local, awsim_local, png_file, rmse_dict)

    # 指標ごとのPDFを生成
    if rmse_dict:
        save_metric_pdfs(rmse_dict, out_dir)

    # 追加: 各項目をSVGで出力（X, Y, Yaw, Linear v, vx, vy, angular w）
    def save_metric_svgs(rmse_dict, output_dir):
        """CSVからデータを読み、各項目をSVGで保存する
        タイトルと軸スケールは `plot_comparison` の表示に揃える。
        """
        try:
            import matplotlib.pyplot as plt
        except Exception:
            print('Plotting dependencies not found for SVG export.')
            return

        csv_candidates = [p for p in os.listdir(output_dir) if p.endswith('.csv') and p.startswith('comparison_')]
        if not csv_candidates:
            print('No comparison CSV found in output directory; cannot generate SVGs.')
            return

        csv_path = os.path.join(output_dir, csv_candidates[0])

        times = []
        real = {'x': [], 'y': [], 'yaw': [], 'v': [], 'vx': [], 'vy': [], 'w': []}
        sim = {'x': [], 'y': [], 'yaw': [], 'v': [], 'vx': [], 'vy': [], 'w': []}

        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row['time']))
                real['x'].append(float(row['real_x']))
                real['y'].append(float(row['real_y']))
                real['yaw'].append(float(row['real_yaw']))
                real['v'].append(float(row['real_v']))
                real['vx'].append(float(row['real_vx']))
                real['vy'].append(float(row['real_vy']))
                real['w'].append(float(row['real_w']))

                sim['x'].append(float(row['sim_x']))
                sim['y'].append(float(row['sim_y']))
                sim['yaw'].append(float(row['sim_yaw']))
                sim['v'].append(float(row['sim_v']))
                sim['vx'].append(float(row['sim_vx']))
                sim['vy'].append(float(row['sim_vy']))
                sim['w'].append(float(row['sim_w']))

        times = np.array(times)
        for k in real:
            real[k] = np.array(real[k])
            sim[k] = np.array(sim[k])

        # Helper to set y limits consistent with PNG: use min/max of both series with small padding
        def compute_ylim(a, b):
            amin = np.nanmin(a) if a.size else 0.0
            bmin = np.nanmin(b) if b.size else 0.0
            amax = np.nanmax(a) if a.size else 0.0
            bmax = np.nanmax(b) if b.size else 0.0
            lo = min(amin, bmin)
            hi = max(amax, bmax)
            if hi - lo == 0:
                pad = abs(hi) * 0.1 + 0.1
            else:
                pad = (hi - lo) * 0.05
            return (lo - pad, hi + pad)

        exports = [
            ('x_position', real['x'], sim['x'], 'X Position', 'X Position (m)', None),
            ('y_position', real['y'], sim['y'], 'Y Position', 'Y Position (m)', None),
            ('yaw_angle', np.rad2deg(real['yaw']), np.rad2deg(sim['yaw']), 'Yaw Angle', 'Yaw Angle (deg)', f'Yaw Angle (RMSE: {rmse_dict.get("yaw_deg",0.0):.4f}°)'),
            ('linear_velocity', real['v'], sim['v'], 'Linear Velocity', 'Linear Velocity (m/s)', f'Linear Velocity (RMSE: {rmse_dict.get("linear_velocity",0.0):.4f} m/s)'),
            ('forward_velocity', real['vx'], sim['vx'], 'Forward Velocity (vx)', 'Forward Velocity (m/s)', f'Forward Velocity (RMSE: {rmse_dict.get("forward_velocity",0.0):.4f} m/s)'),
            ('lateral_velocity', real['vy'], sim['vy'], 'Lateral Velocity (vy)', 'Lateral Velocity (m/s)', f'Lateral Velocity (RMSE: {rmse_dict.get("lateral_velocity",0.0):.4f} m/s)'),
            ('angular_velocity', real['w'], sim['w'], 'Angular Velocity', 'Angular Velocity (rad/s)', f'Angular Velocity (RMSE: {rmse_dict.get("angular_velocity",0.0):.4f} rad/s)'),
        ]

        for name, rdata, sdata, title_short, ylabel, title_with_rmse in exports:
            # Use square aspect for individual SVGs: width:height = 1:1
            fig, ax = plt.subplots(figsize=(6, 6))
            ax.plot(times, rdata, 'b-', linewidth=2, label='Real')
            ax.plot(times, sdata, 'r--', linewidth=2, label='AWSIM')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            # title: use title_with_rmse if provided, else short title
            if title_with_rmse:
                ax.set_title(title_with_rmse)
            else:
                ax.set_title(title_short)
            # axis limits like PNG
            try:
                ylim = compute_ylim(rdata, sdata)
                ax.set_ylim(ylim)
            except Exception:
                pass
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3)
            fig.subplots_adjust(top=0.92, bottom=0.14, left=0.08, right=0.98)

            out_svg = os.path.join(output_dir, f'{name}.svg')
            fig.savefig(out_svg, format='svg', bbox_inches='tight')
            plt.close(fig)
            print(f'Saved SVG: {out_svg}')

    # 呼び出し
    if rmse_dict:
        save_metric_svgs(rmse_dict, out_dir)

    # 比較に使用したbagをコピーして保存（両ディレクトリ）
    try:
        real_bag_name = os.path.basename(os.path.abspath(real_bag_path.rstrip('/')))
        sim_bag_name = os.path.basename(os.path.abspath(awsim_bag_path.rstrip('/')))
        real_dest = os.path.join(out_dir, real_bag_name)
        sim_dest = os.path.join(out_dir, sim_bag_name)
        print(f'Copying real bag to {real_dest} ...')
        if os.path.isdir(real_bag_path):
            shutil.copytree(real_bag_path, real_dest, dirs_exist_ok=True)
        else:
            shutil.copy(real_bag_path, real_dest)

        print(f'Copying sim bag to {sim_dest} ...')
        if os.path.isdir(awsim_bag_path):
            shutil.copytree(awsim_bag_path, sim_dest, dirs_exist_ok=True)
        else:
            shutil.copy(awsim_bag_path, sim_dest)
    except Exception as e:
        print(f'Warning: failed to copy bag directories into output folder: {e}')

    print('\n' + '='*70)
    print('=== Comparison Complete ===')
    print('='*70)
    print(f'Output folder: {out_dir}')
    print(f'CSV:  {csv_file}')
    print(f'Plot: {png_file}')
    print('='*70)


if __name__ == '__main__':
    main()
