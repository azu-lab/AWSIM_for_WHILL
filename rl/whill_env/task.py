"""
Task definition: where an episode starts, where the goal is, and the settings of the
learning environment (action limits, reward, termination, observation, topics).

A task is one YAML file in rl/configs/tasks/ (see shinjuku_box.yaml for every key with
comments). Positions are in the map frame of AWSIM, the frame of /ground_truth/odom:

    map x = Unity z,   map y = -Unity x,   map z = Unity y (height),   yaw: counter-clockwise from map x

so a point picked in the Unity Editor at Unity (X, Y, Z) is map (Z, -X). Heights are not needed:
AWSIM puts the start pose and the goal on the ground below (x, y).

Obstacles are not part of the task file: they are objects placed in the scene with the Unity
Editor (see docs/DREAMER_TRAINING.md).
"""

import copy
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np
import yaml


@dataclass
class Pose2D:
    """Pose of base_link in the map frame. z: optional height hint for the ground search under bridges/roofs."""
    x: float
    y: float
    yaw: float = 0.0
    z: Optional[float] = None


@dataclass
class Episode:
    """What the environment sends to AWSIM before a reset."""
    start: Optional[Pose2D]      # None: the scene placement of the WHILL
    goal: Optional[Pose2D]       # None: the goal object of the scene


# Every key of a task file with its default. Unknown keys are rejected (typos would otherwise be
# silently ignored). The defaults are the values of the paper / soturon.
DEFAULTS: Dict[str, Any] = {
    "name": "unnamed",
    "description": "",
    "scene": "AutowareSimulationDemo",   # informational: the AWSIM scene/player the task is made for
    "start": {
        "poses": None,                   # list of {x, y, yaw[, z]}; one is drawn per episode. None: scene placement
        "noise": {"x": 0.0, "y": 0.0, "yaw": 0.0},   # uniform +- noise added per episode [m, m, rad]
    },
    "goal": {
        "position": None,                # {x, y[, z]} in the map frame, or
        "ahead": None,                   # distance [m] ahead of the start pose (along its yaw), and
        "left": 0.0,                     # lateral offset [m] (positive: left of the start pose)
        "noise": {"x": 0.0, "y": 0.0},   # uniform +- noise added per episode [m]
        "radius": 1.0,                   # success when the 2D distance base_link-goal is <= radius [m]
    },
    "action": {
        "max_linear_vel": 0.3,           # [m/s]   linear.x  = a0 * max_linear_vel
        "max_angular_vel": 0.6,          # [rad/s] angular.z = a1 * max_angular_vel
        "allow_reverse": False,          # False: linear.x is clipped to [0, max]
    },
    "episode": {
        "max_steps": 150,                # truncated after this many decision steps (None: no limit)
        "timeout_sec": None,             # truncated after this wall-clock time (None: no limit)
        "frame_timeout_sec": 0.5,        # a step waits at most this long for a new camera frame
        "reset_timeout_sec": 5.0,
    },
    "reward": {                          # r = goal + collision + progress*(d_prev - d) + time - smooth*|a1 - a1_prev|
        "goal": 100.0,
        "collision": -50.0,
        "progress": 30.0,
        "time": -0.05,
        "smooth": 1.0,
    },
    "observation": {
        "image_size": [64, 64],          # [height, width]
        # Keys of the observation dict. image, is_success, is_collision are the observation of the
        # paper/soturon. Optional: goal (distance [m], bearing [rad] in base_link), velocity (v, w).
        "keys": ["image", "is_success", "is_collision"],
        "domain_randomization": False,   # brightness/contrast/hue/saturation/noise per step (sim-to-real)
    },
    "topics": {                          # ROS 2 (sensor data, commands), see data_manager.py
        "camera": "/sensing/camera/front/image_raw",
        "odom": "/ground_truth/odom",
        "collision": "/collision_sensor",
        "cmd_vel": "/whill/controller/cmd_vel",
        "gear_cmd": "/control/command/gear_cmd",
        "goal_position": "/awsim/goal_position",
        "obstacles": "/awsim/episode/obstacles",
    },
    "awsim": {                           # synchronous TCP requests (episode reset) to AWSIM's OptimizerBridge
        "host": "127.0.0.1",
        "port": 50051,
        "timeout_sec": 10.0,
    },
}

OBSERVATION_KEYS = ("image", "is_success", "is_collision", "goal", "velocity")


def _merge(defaults: Dict[str, Any], values: Dict[str, Any], where: str) -> Dict[str, Any]:
    out = copy.deepcopy(defaults)
    for key, value in (values or {}).items():
        if key not in defaults:
            raise KeyError(f"unknown task setting '{where}{key}' (known: {', '.join(defaults)})")
        if isinstance(defaults[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f"task setting '{where}{key}' must be a mapping")
            out[key] = _merge(defaults[key], value, f"{where}{key}.")
        else:
            out[key] = value
    return out


def _pose(d: Dict[str, Any], where: str) -> Pose2D:
    unknown = set(d) - {"x", "y", "yaw", "z"}
    if unknown or "x" not in d or "y" not in d:
        raise ValueError(f"{where}: expected {{x, y[, yaw][, z]}}, got {d}")
    z = d.get("z")
    return Pose2D(float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0)), None if z is None else float(z))


class Task:
    """A task file, validated, and the per-episode sampling of start pose and goal."""

    def __init__(self, data: Dict[str, Any], path: str = "<dict>"):
        self.path = path
        self.cfg = _merge(DEFAULTS, data, "")
        c = self.cfg
        self.name = c["name"]

        poses = c["start"]["poses"]
        if isinstance(poses, dict):
            poses = [poses]
        self.start_poses: List[Pose2D] = [_pose(p, f"{path}: start.poses[{i}]") for i, p in enumerate(poses or [])]

        g = c["goal"]
        if g["position"] is not None and g["ahead"] is not None:
            raise ValueError(f"{path}: give either goal.position or goal.ahead, not both")
        if g["ahead"] is not None and not self.start_poses:
            raise ValueError(f"{path}: goal.ahead needs start.poses")
        self.goal_position = _pose(g["position"], f"{path}: goal.position") if g["position"] is not None else None

        for key in c["observation"]["keys"]:
            if key not in OBSERVATION_KEYS:
                raise ValueError(f"{path}: unknown observation key '{key}' (known: {', '.join(OBSERVATION_KEYS)})")
        if "image" not in c["observation"]["keys"]:
            raise ValueError(f"{path}: observation.keys must contain 'image'")

    # Shortcuts used by the environment
    @property
    def action(self): return self.cfg["action"]
    @property
    def episode(self): return self.cfg["episode"]
    @property
    def reward(self): return self.cfg["reward"]
    @property
    def observation(self): return self.cfg["observation"]
    @property
    def topics(self): return self.cfg["topics"]
    @property
    def goal_radius(self) -> float: return float(self.cfg["goal"]["radius"])

    def sample(self, rng: np.random.Generator) -> Episode:
        """Start pose and goal of the next episode."""
        start = None
        if self.start_poses:
            base = self.start_poses[rng.integers(len(self.start_poses))]
            n = self.cfg["start"]["noise"]
            start = Pose2D(
                base.x + rng.uniform(-n["x"], n["x"]),
                base.y + rng.uniform(-n["y"], n["y"]),
                _wrap(base.yaw + rng.uniform(-n["yaw"], n["yaw"])),
                base.z)

        g = self.cfg["goal"]
        goal = None
        if self.goal_position is not None:
            goal = Pose2D(self.goal_position.x, self.goal_position.y, 0.0, self.goal_position.z)
        elif g["ahead"] is not None:
            # relative to the start pose without noise: the goal stays where the task puts it
            ref = base
            c, s = math.cos(ref.yaw), math.sin(ref.yaw)
            ahead, left = float(g["ahead"]), float(g["left"])
            goal = Pose2D(ref.x + c * ahead - s * left, ref.y + s * ahead + c * left, 0.0, ref.z)
        if goal is not None:
            n = g["noise"]
            goal = Pose2D(goal.x + rng.uniform(-n["x"], n["x"]), goal.y + rng.uniform(-n["y"], n["y"]), 0.0, goal.z)
        return Episode(start, goal)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def load_task(path: str) -> Task:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return Task(data, str(path))
