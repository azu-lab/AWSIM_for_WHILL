"""
rosbag2からデータを読み込むモジュール

rosbags ライブラリを使用してsqlite3形式のrosbag2を読み込み、
cmd_velとodomデータを時系列として抽出します。
"""

import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from .paths import DEFAULT_ROSBAG_DIR


@dataclass
class OdomData:
    """Odometryデータを格納するクラス"""
    timestamps: np.ndarray  # タイムスタンプ (秒)
    positions_x: np.ndarray  # X位置 (m)
    positions_y: np.ndarray  # Y位置 (m) - ROS座標系
    positions_z: np.ndarray  # Z位置 (m) - ROS座標系
    orientations: np.ndarray  # クォータニオン [x, y, z, w]
    linear_velocities: np.ndarray  # 線速度 [x, y, z]
    angular_velocities: np.ndarray  # 角速度 [x, y, z]


@dataclass
class CmdVelData:
    """cmd_velデータを格納するクラス"""
    timestamps: np.ndarray  # タイムスタンプ (秒)
    linear_x: np.ndarray  # 前進速度指令 (m/s)
    linear_y: np.ndarray  # 横速度指令 (m/s)
    linear_z: np.ndarray  # 垂直速度指令 (m/s)
    angular_x: np.ndarray  # Roll角速度指令 (rad/s)
    angular_y: np.ndarray  # Pitch角速度指令 (rad/s)
    angular_z: np.ndarray  # Yaw角速度指令 (rad/s)


@dataclass
class ScenarioData:
    """1つのシナリオのデータを格納するクラス"""
    name: str
    odom: OdomData
    cmd_vel: CmdVelData
    duration: float  # シナリオの長さ (秒)


class RosbagLoader:
    """rosbag2データを読み込むクラス"""
    
    # トピック名の候補
    ODOM_TOPICS = ['/whill/odom', '/odom', '/odometry']
    CMD_VEL_TOPICS = ['/whill/controller/cmd_vel', '/cmd_vel', '/whill/cmd_vel']
    
    def __init__(self, bag_path: str):
        """
        Args:
            bag_path: rosbag2ディレクトリへのパス
        """
        self.bag_path = Path(bag_path)
        if not self.bag_path.exists():
            raise FileNotFoundError(f"rosbag2 path not found: {bag_path}")
        
        # 型情報ストアを取得 (ROS2 Humble)
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    def load(self) -> Tuple[OdomData, CmdVelData]:
        """
        rosbag2からodomとcmd_velデータを読み込む
        
        Returns:
            (OdomData, CmdVelData) のタプル
        """
        odom_data = self._load_odom()
        cmd_vel_data = self._load_cmd_vel()
        return odom_data, cmd_vel_data
    
    def _find_topic(self, reader: Reader, candidates: List[str]) -> Optional[str]:
        """利用可能なトピックを候補から探す"""
        available_topics = set(conn.topic for conn in reader.connections)
        for topic in candidates:
            if topic in available_topics:
                return topic
        return None
    
    def _load_odom(self) -> OdomData:
        """Odometryデータを読み込む"""
        timestamps = []
        positions_x = []
        positions_y = []
        positions_z = []
        orientations = []
        linear_velocities = []
        angular_velocities = []
        
        with Reader(self.bag_path) as reader:
            topic = self._find_topic(reader, self.ODOM_TOPICS)
            if topic is None:
                available = [conn.topic for conn in reader.connections]
                raise ValueError(f"No odom topic found. Available topics: {available}")
            
            connections = [c for c in reader.connections if c.topic == topic]
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                # タイムスタンプ (メッセージヘッダから取得 - compare_bags.py と同様)
                stamp = msg.header.stamp
                timestamps.append(stamp.sec + stamp.nanosec * 1e-9)
                
                # 位置
                positions_x.append(msg.pose.pose.position.x)
                positions_y.append(msg.pose.pose.position.y)
                positions_z.append(msg.pose.pose.position.z)
                
                # 姿勢
                q = msg.pose.pose.orientation
                orientations.append([q.x, q.y, q.z, q.w])
                
                # 速度
                lin = msg.twist.twist.linear
                ang = msg.twist.twist.angular
                linear_velocities.append([lin.x, lin.y, lin.z])
                angular_velocities.append([ang.x, ang.y, ang.z])
        
        if len(timestamps) == 0:
            raise ValueError(f"No odom messages found in {self.bag_path}")
        
        # タイムスタンプは正規化しない (compare_bags.py と同様)
        # synchronize_time で正規化される
        timestamps = np.array(timestamps)
        
        return OdomData(
            timestamps=timestamps,
            positions_x=np.array(positions_x),
            positions_y=np.array(positions_y),
            positions_z=np.array(positions_z),
            orientations=np.array(orientations),
            linear_velocities=np.array(linear_velocities),
            angular_velocities=np.array(angular_velocities)
        )
    
    def _load_cmd_vel(self) -> CmdVelData:
        """cmd_velデータを読み込む"""
        timestamps = []
        linear_x = []
        linear_y = []
        linear_z = []
        angular_x = []
        angular_y = []
        angular_z = []
        
        with Reader(self.bag_path) as reader:
            topic = self._find_topic(reader, self.CMD_VEL_TOPICS)
            if topic is None:
                available = [conn.topic for conn in reader.connections]
                raise ValueError(f"No cmd_vel topic found. Available topics: {available}")
            
            connections = [c for c in reader.connections if c.topic == topic]
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
                
                timestamps.append(timestamp * 1e-9)
                linear_x.append(msg.linear.x)
                linear_y.append(msg.linear.y)
                linear_z.append(msg.linear.z)
                angular_x.append(msg.angular.x)
                angular_y.append(msg.angular.y)
                angular_z.append(msg.angular.z)
        
        if len(timestamps) == 0:
            raise ValueError(f"No cmd_vel messages found in {self.bag_path}")
        
        timestamps = np.array(timestamps)
        timestamps = timestamps - timestamps[0]
        
        return CmdVelData(
            timestamps=timestamps,
            linear_x=np.array(linear_x),
            linear_y=np.array(linear_y),
            linear_z=np.array(linear_z),
            angular_x=np.array(angular_x),
            angular_y=np.array(angular_y),
            angular_z=np.array(angular_z)
        )


def load_all_scenarios(base_path: str, scenario_filter: List[str] = None) -> Dict[str, ScenarioData]:
    """
    全シナリオのデータを読み込む
    
    Args:
        base_path: rosbagディレクトリが格納されているベースパス
        scenario_filter: 読み込むシナリオ名のリスト (Noneの場合は全て)
                        例: ['straight', 'turn', 'circle']
        
    Returns:
        シナリオ名をキーとするScenarioDataの辞書
    """
    base = Path(base_path)
    scenarios = {}
    
    # シナリオの定義
    scenario_dirs = {
        'straight': 'whill_cmd_straight_20251216_125820',
        's_curve': 'whill_cmd_s_curve_20251217_172829',
        'turn': 'whill_cmd_turn_20251216_131910',
        'circle': 'whill_cmd_circle_20251216_131510',
        'lemniscate': 'whill_cmd_lemniscate_20260108_113110',
    }
    
    # フィルタが指定されている場合、該当するシナリオのみ読み込む
    if scenario_filter:
        filtered_dirs = {k: v for k, v in scenario_dirs.items() if k in scenario_filter}
        if not filtered_dirs:
            print(f"Warning: No matching scenarios found for filter: {scenario_filter}")
            print(f"Available scenarios: {list(scenario_dirs.keys())}")
        scenario_dirs = filtered_dirs
    
    for name, dirname in scenario_dirs.items():
        bag_path = base / dirname
        if bag_path.exists():
            try:
                loader = RosbagLoader(str(bag_path))
                odom, cmd_vel = loader.load()
                # タイムスタンプから相対duration計算 (正規化なしでも正しく動作)
                odom_duration = odom.timestamps[-1] - odom.timestamps[0]
                cmd_vel_duration = cmd_vel.timestamps[-1]  # cmd_velは0始まりに正規化済み
                duration = max(odom_duration, cmd_vel_duration)
                scenarios[name] = ScenarioData(
                    name=name,
                    odom=odom,
                    cmd_vel=cmd_vel,
                    duration=duration
                )
                print(f"Loaded scenario '{name}': {len(odom.timestamps)} odom msgs, "
                      f"{len(cmd_vel.timestamps)} cmd_vel msgs, duration={duration:.2f}s")
            except Exception as e:
                print(f"Warning: Failed to load scenario '{name}': {e}")
        else:
            print(f"Warning: Scenario directory not found: {bag_path}")
    
    return scenarios


if __name__ == "__main__":
    # テスト実行
    import sys
    
    if len(sys.argv) > 1:
        base_path = sys.argv[1]
    else:
        base_path = DEFAULT_ROSBAG_DIR
    
    print(f"Loading scenarios from: {base_path}")
    scenarios = load_all_scenarios(base_path)
    
    for name, data in scenarios.items():
        print(f"\n=== {name} ===")
        print(f"Duration: {data.duration:.2f}s")
        print(f"Odom samples: {len(data.odom.timestamps)}")
        print(f"  X range: [{data.odom.positions_x.min():.3f}, {data.odom.positions_x.max():.3f}]")
        print(f"  Y range: [{data.odom.positions_y.min():.3f}, {data.odom.positions_y.max():.3f}]")
        print(f"CmdVel samples: {len(data.cmd_vel.timestamps)}")
        print(f"  Linear X range: [{data.cmd_vel.linear_x.min():.3f}, {data.cmd_vel.linear_x.max():.3f}]")
        print(f"  Angular Z range: [{data.cmd_vel.angular_z.min():.3f}, {data.cmd_vel.angular_z.max():.3f}]")
