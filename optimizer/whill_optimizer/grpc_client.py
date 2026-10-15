"""
TCPクライアント - AWSIM (Unity) との通信を行う

OptimizerBridge (TCP/JSON-RPC) と通信し、
パラメータ設定、シミュレーション制御、データ取得を行います。

メッセージフレーミング: 4バイトのbig-endian長さプレフィックスを使用
"""

import socket
import struct
import json
import numpy as np
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass, asdict
import time


@dataclass
class VehicleParameters:
    """車両パラメータを格納するクラス"""
    # 物理パラメータ
    skidding_cancel_rate: float = 10.0
    front_wheel_skidding_rate: float = 0.1
    forward_slip_multiplier: float = 1.0
    
    # PIDゲイン
    wheel_kp: float = 10.0
    wheel_ki: float = 0.1
    wheel_kd: float = 0.0
    wheel_integral_clamp: float = 0.5
    
    # 抵抗力
    air_drag: float = 0.2
    rolling_resistance: float = 0.02
    frontal_area: float = 0.8
    
    # 重心オフセット
    com_offset_x: float = 0.0
    com_offset_y: float = 0.0
    com_offset_z: float = 0.0
    
    def to_dict(self) -> Dict[str, float]:
        """辞書に変換"""
        return asdict(self)


@dataclass
class OdomSample:
    """単一のOdomサンプル"""
    timestamp: float
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # [x, y, z, w] (quaternion)
    linear_velocity: np.ndarray  # [x, y, z]
    angular_velocity: np.ndarray  # [x, y, z]


class TcpClient:
    """TCP/JSON-RPCクライアントクラス（長さプレフィックス付き）"""
    
    def __init__(self, host: str = "localhost", port: int = 50051, timeout: float = 60.0):
        """
        Args:
            host: サーバーのホスト
            port: サーバーのポート
            timeout: デフォルトタイムアウト (秒)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self._connected = False
    
    def connect(self) -> bool:
        """サーバーに接続"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            
            # 接続テスト (Ping)
            response = self._send_request("ping", {})
            self._connected = response.get("success", False)
            
            if self._connected:
                print(f"Connected to AWSIM at {self.host}:{self.port}")
            
            return self._connected
        except Exception as e:
            print(f"Failed to connect: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """接続を切断"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            self._connected = False
    
    def is_connected(self) -> bool:
        """接続状態を確認"""
        return self._connected
    
    def _recv_exact(self, n: int) -> bytes:
        """指定バイト数を確実に受信"""
        data = b''
        while len(data) < n:
            chunk = self.socket.recv(n - len(data))
            if not chunk:
                raise ConnectionError("Connection closed by server")
            data += chunk
        return data
    
    def _send_request(self, method: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """リクエストを送信してレスポンスを受信（長さプレフィックス付き）"""
        if self.socket is None:
            raise RuntimeError("Not connected")
        
        # リクエストを構築
        request = {
            "method": method,
            "params_json": json.dumps(params)
        }
        
        request_bytes = json.dumps(request).encode('utf-8')
        
        try:
            # 長さプレフィックス (4バイト, big-endian) + メッセージを送信
            length_prefix = struct.pack('>I', len(request_bytes))
            self.socket.sendall(length_prefix + request_bytes)
            
            # 長さプレフィックスを受信
            length_bytes = self._recv_exact(4)
            response_length = struct.unpack('>I', length_bytes)[0]
            
            # メッセージ本体を受信
            response_bytes = self._recv_exact(response_length)
            response_json = response_bytes.decode('utf-8')
            
            return json.loads(response_json)
        except socket.timeout:
            print(f"Request timeout: {method}")
            return {"success": False, "message": "Timeout"}
        except Exception as e:
            print(f"Request error: {e}")
            return {"success": False, "message": str(e)}
    
    def set_parameters(self, params: VehicleParameters) -> bool:
        """
        車両パラメータを設定
        
        Args:
            params: VehicleParametersオブジェクト
            
        Returns:
            成功した場合True
        """
        response = self._send_request("set_parameters", params.to_dict())
        return response.get("success", False)
    
    def get_parameters(self) -> Optional[Dict[str, float]]:
        """
        AWSIM で現在有効な車両パラメータを取得 (OptimizerBridge の get_parameters)

        Returns:
            パラメータ名 -> 値 の辞書。失敗した場合 None
        """
        response = self._send_request("get_parameters", {})
        if not response.get("success", False):
            return None
        return json.loads(response.get("data_json", "{}"))

    def reset_simulation(self, position: Tuple[float, float, float] = (0, 0, 0),
                         rotation: Tuple[float, float, float] = (0, 0, 0)) -> bool:
        """
        シミュレーションをリセット
        
        Args:
            position: 初期位置 (x, y, z)
            rotation: 初期回転 (euler angles in degrees)
            
        Returns:
            成功した場合True
        """
        params = {
            "position_x": position[0],
            "position_y": position[1],
            "position_z": position[2],
            "rotation_x": rotation[0],
            "rotation_y": rotation[1],
            "rotation_z": rotation[2],
        }
        
        response = self._send_request("reset_simulation", params)
        return response.get("success", False)
    
    def send_cmd_vel(self, linear: Tuple[float, float, float],
                     angular: Tuple[float, float, float]) -> bool:
        """
        cmd_velを送信
        
        Args:
            linear: 線速度 (x, y, z) [m/s]
            angular: 角速度 (x, y, z) [rad/s]
            
        Returns:
            成功した場合True
        """
        params = {
            "linear_x": linear[0],
            "linear_y": linear[1],
            "linear_z": linear[2],
            "angular_x": angular[0],
            "angular_y": angular[1],
            "angular_z": angular[2],
        }
        
        response = self._send_request("send_cmd_vel", params)
        return response.get("success", False)
    
    def get_current_odom(self) -> Optional[OdomSample]:
        """現在のOdomを取得"""
        response = self._send_request("get_current_odom", {})
        
        if not response.get("success", False):
            return None
        
        try:
            data_json = response.get("data_json", "{}")
            data = json.loads(data_json)
            
            return OdomSample(
                timestamp=data.get("timestamp", 0),
                position=np.array([
                    data.get("position_x", 0),
                    data.get("position_y", 0),
                    data.get("position_z", 0)
                ]),
                orientation=np.array([
                    data.get("orientation_x", 0),
                    data.get("orientation_y", 0),
                    data.get("orientation_z", 0),
                    data.get("orientation_w", 1)
                ]),
                linear_velocity=np.array([
                    data.get("linear_velocity_x", 0),
                    data.get("linear_velocity_y", 0),
                    data.get("linear_velocity_z", 0)
                ]),
                angular_velocity=np.array([
                    data.get("angular_velocity_x", 0),
                    data.get("angular_velocity_y", 0),
                    data.get("angular_velocity_z", 0)
                ]),
            )
        except Exception as e:
            print(f"Failed to parse odom: {e}")
            return None
    
    def run_scenario(self, params: VehicleParameters,
                     cmd_vel_timestamps: np.ndarray,
                     cmd_vel_linear_x: np.ndarray,
                     cmd_vel_angular_z: np.ndarray,
                     duration: float,
                     time_scale: float = 5.0,
                     odom_sample_interval: float = 0.02) -> Optional[List[OdomSample]]:
        """
        シナリオ全体を実行
        
        Args:
            params: 車両パラメータ
            cmd_vel_timestamps: cmd_velのタイムスタンプ配列
            cmd_vel_linear_x: 前進速度指令配列 [m/s]
            cmd_vel_angular_z: 角速度指令配列 [rad/s]
            duration: シナリオの長さ [秒]
            time_scale: 時間スケール (高速化)
            odom_sample_interval: odomサンプリング間隔 [秒]
            
        Returns:
            OdomSampleのリスト、失敗時None
        """
        # cmd_velシーケンスを作成
        cmd_vel_sequence = []
        for i in range(len(cmd_vel_timestamps)):
            cmd_vel_sequence.append({
                "timestamp": float(cmd_vel_timestamps[i]),
                "linear_x": float(cmd_vel_linear_x[i]),
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_x": 0.0,
                "angular_y": 0.0,
                "angular_z": float(cmd_vel_angular_z[i]),
            })
        
        request_params = {
            "parameters_json": json.dumps(params.to_dict()),
            "cmd_vel_sequence_json": json.dumps(cmd_vel_sequence),
            "duration_seconds": duration,
            "time_scale": time_scale,
            "odom_sample_interval": odom_sample_interval,
        }
        
        # タイムアウトを延長
        old_timeout = self.socket.gettimeout()
        new_timeout = max(self.timeout, duration / time_scale + 120.0)
        self.socket.settimeout(new_timeout)
        
        try:
            response = self._send_request("run_scenario", request_params)
        finally:
            self.socket.settimeout(old_timeout)
        
        if not response.get("success", False):
            print(f"RunScenario failed: {response.get('message', 'Unknown error')}")
            return None
        
        # 結果をパース
        try:
            data_json = response.get("data_json", "{}")
            result = json.loads(data_json)
            
            odom_trajectory_json = result.get("odom_trajectory_json", "[]")
            odom_list_raw = json.loads(odom_trajectory_json)
            
            odom_list = []
            for odom_data in odom_list_raw:
                sample = OdomSample(
                    timestamp=odom_data.get("timestamp", 0),
                    position=np.array([
                        odom_data.get("position_x", 0),
                        odom_data.get("position_y", 0),
                        odom_data.get("position_z", 0)
                    ]),
                    orientation=np.array([
                        odom_data.get("orientation_x", 0),
                        odom_data.get("orientation_y", 0),
                        odom_data.get("orientation_z", 0),
                        odom_data.get("orientation_w", 1)
                    ]),
                    linear_velocity=np.array([
                        odom_data.get("linear_velocity_x", 0),
                        odom_data.get("linear_velocity_y", 0),
                        odom_data.get("linear_velocity_z", 0)
                    ]),
                    angular_velocity=np.array([
                        odom_data.get("angular_velocity_x", 0),
                        odom_data.get("angular_velocity_y", 0),
                        odom_data.get("angular_velocity_z", 0)
                    ]),
                )
                odom_list.append(sample)
            
            print(f"Scenario completed: {len(odom_list)} odom samples")
            return odom_list
            
        except Exception as e:
            print(f"Failed to parse scenario result: {e}")
            return None


# シングルトンインスタンス
_default_client: Optional[TcpClient] = None


def get_client(host: str = "localhost", port: int = 50051) -> TcpClient:
    """デフォルトのTCPクライアントを取得"""
    global _default_client
    if _default_client is None:
        _default_client = TcpClient(host, port)
    return _default_client


# エイリアス (grpc_client.pyからの互換性)
GrpcClient = TcpClient
