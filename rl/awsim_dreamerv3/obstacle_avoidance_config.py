"""
Configuration for Obstacle Avoidance (Cardboard Box) Task

ダンボール回避前進タスク用の設定。
既存の config.py の設定を基盤とし、タスク固有の値のみ上書きする。

座標系:
  Unity X = -3m (ゴール方向) → ROS Y = +3m

行動空間 (実機転移想定):
  速度:  [0, 0.3] m/s    (前進のみ)
  角速度: [-0.3, 0.3] rad/s
"""

from . import config as base_config

# ===== 行動空間 (実機転移想定) =====
MAX_LINEAR_VEL = 0.3    # m/s (forward velocity)
MAX_ANGULAR_VEL = 0.6   # rad/s (yaw rate)

# ===== ゴール設定 =====
# Unity座標系で X方向に -3m = ROS座標系で Y方向に +3m
GOAL_DISTANCE_ROS_Y = 4.5    # meters (ROS Y方向のゴールオフセット)
GOAL_RADIUS = 0.5            # meters (ゴール到達判定半径: 円形エリア)
GOAL_SIZE = 0.5              # meters (正方形ゴールの辺の長さ = 0.5m x 0.5m)

# ===== エピソード設定 =====
EPISODE_TIMEOUT = 30.0   # seconds (低速のため長めに設定)

# ===== 報酬設定 =====
COLLISION_PENALTY = -50.0         # 障害物・壁衝突時のペナルティ
GOAL_REWARD = 100.0               # ゴール到達報酬
FORWARD_REWARD_SCALE = 30.0       # 前進報酬スケール (1m近づくと +30点)
TIME_PENALTY = -0.05              # 時間ペナルティ (/step)

# ===== 以下は base_config から継承 =====
IMAGE_SIZE = base_config.IMAGE_SIZE
IMAGE_WIDTH = base_config.IMAGE_WIDTH
IMAGE_HEIGHT = base_config.IMAGE_HEIGHT
STEP_DURATION = base_config.STEP_DURATION
ROS_SPIN_TIMEOUT = base_config.ROS_SPIN_TIMEOUT

# ROS 2 Topics (共通)
CAMERA_TOPIC = base_config.CAMERA_TOPIC
ODOM_TOPIC = base_config.ODOM_TOPIC
COLLISION_TOPIC = base_config.COLLISION_TOPIC
CMD_VEL_TOPIC = base_config.CMD_VEL_TOPIC
GEAR_CMD_TOPIC = base_config.GEAR_CMD_TOPIC
RESET_COMMAND_TOPIC = base_config.RESET_COMMAND_TOPIC
RESET_RESPONSE_TOPIC = base_config.RESET_RESPONSE_TOPIC
INITIAL_POSE_TOPIC = base_config.INITIAL_POSE_TOPIC
GOAL_POSITION_TOPIC = base_config.GOAL_POSITION_TOPIC

# AWSIM TCP Connection
AWSIM_HOST = base_config.AWSIM_HOST
AWSIM_PORT = base_config.AWSIM_PORT
