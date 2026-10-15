"""
環境設定 YAML (rl/configs/env/*.yaml) を読み込み、config / obstacle_avoidance_config の定数を上書きする。

報酬係数や行動範囲をコードに直書きせず、実験ごとの設定ファイルとして残すため。
YAML に書かれていない項目はコード上の既定値 (soturon e91b33f の値) のまま。

    from awsim_dreamerv3.env_config import apply_env_config
    applied = apply_env_config("rl/configs/env/obstacle_avoidance.yaml")

環境 (env.py / obstacle_avoidance_env.py) を生成する前に呼ぶこと。
"""

from typing import Any, Dict

import yaml

from . import config as base_config
from . import obstacle_avoidance_config as oa_config

# (section, key) -> (module, constant). module: "base" | "oa" | "both"
KEY_MAP = {
    ("action", "max_linear_vel"): ("task", "MAX_LINEAR_VEL"),
    ("action", "max_angular_vel"): ("task", "MAX_ANGULAR_VEL"),
    ("episode", "timeout_sec"): ("task", "EPISODE_TIMEOUT"),
    ("episode", "step_duration"): ("both", "STEP_DURATION"),
    ("goal", "radius"): ("task", "GOAL_RADIUS"),
    ("goal", "position"): ("base", "GOAL_POSITION"),
    ("episode", "start_pose"): ("base", "START_POSE"),
    ("goal", "fallback_distance_ros_x"): ("base", "GOAL_DISTANCE"),
    ("goal", "fallback_offset_ros_y"): ("oa", "GOAL_DISTANCE_ROS_Y"),
    ("reward", "goal"): ("task", "GOAL_REWARD"),
    ("reward", "collision"): ("task", "COLLISION_PENALTY"),
    ("reward", "progress_scale"): ("task:PROGRESS", None),
    ("reward", "time"): ("task", "TIME_PENALTY"),
    ("reward", "steering_change"): ("base", "STEERING_CHANGE_PENALTY"),
    ("topics", "camera"): ("both", "CAMERA_TOPIC"),
    ("topics", "odom"): ("both", "ODOM_TOPIC"),
    ("topics", "collision"): ("both", "COLLISION_TOPIC"),
    ("topics", "cmd_vel"): ("both", "CMD_VEL_TOPIC"),
    ("topics", "gear_cmd"): ("both", "GEAR_CMD_TOPIC"),
    ("topics", "reset_command"): ("both", "RESET_COMMAND_TOPIC"),
    ("topics", "reset_response"): ("both", "RESET_RESPONSE_TOPIC"),
    ("topics", "goal_position"): ("both", "GOAL_POSITION_TOPIC"),
}


def load_env_config(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def apply_env_config(path: str) -> Dict[str, Any]:
    """
    YAML の値を定数に反映する。

    Returns:
        {"task": タスク名, "<CONSTANT>": 値, ...} 反映した内容
    """
    data = load_env_config(path)
    task = data.get("task", "obstacle_avoidance")
    if task not in ("original", "obstacle_avoidance"):
        raise ValueError(f"{path}: task must be 'original' or 'obstacle_avoidance', got {task!r}")
    task_module = oa_config if task == "obstacle_avoidance" else base_config

    applied: Dict[str, Any] = {"task": task}
    for section, values in data.items():
        if section == "task":
            continue
        if not isinstance(values, dict):
            raise ValueError(f"{path}: section '{section}' must be a mapping")
        for key, value in values.items():
            if (section, key) not in KEY_MAP:
                raise KeyError(f"{path}: unknown setting {section}.{key}")
            target, const = KEY_MAP[(section, key)]
            if target == "task:PROGRESS":
                # 前進報酬の係数は original と obstacle_avoidance で定数名が違う
                const = "FORWARD_REWARD_SCALE" if task == "obstacle_avoidance" else "PROGRESS_REWARD_SCALE"
                modules = [task_module]
            elif target == "task":
                modules = [task_module]
            elif target == "both":
                modules = [base_config, oa_config]
            elif target == "oa":
                modules = [oa_config]
            else:
                modules = [base_config]
            for module in modules:
                if not hasattr(module, const):
                    raise AttributeError(f"{module.__name__} has no {const}")
                current = getattr(module, const)
                if value is None or current is None or isinstance(value, dict):
                    setattr(module, const, value)
                else:
                    setattr(module, const, type(current)(value))
            applied[const] = value
    return applied
