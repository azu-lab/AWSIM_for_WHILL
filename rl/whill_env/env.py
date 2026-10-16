"""
Learning environment management code (paper Fig. 2), as a Gymnasium environment.

    agent (rl/agents/)  --action [-1, 1]^2-->  WhillEnv  --Twist-->  DataManager  -->  AWSIM
                        <--obs, reward, done--           <--Snapshot--

- Process data:               process_image(), WhillEnv._observation()
- Calculate reward:           compute_reward()
- Determine end of episode:   WhillEnv.step() (goal / collision / step limit)
- Convert actions to commands: action_to_command()

Everything here is model-agnostic; all numbers come from the task YAML (task.py). The functions
process_image() and action_to_command() are also used on the real WHILL
(rl/tools/real_whill_inference.py), so the policy sees the same inputs and produces the same
commands in simulation and on the vehicle.
"""

import math
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .domain_randomization import DomainRandomization
from .task import Episode, Task


# ---------------------------------------------------------------------- pure functions
def process_image(image: Optional[np.ndarray], size: Tuple[int, int]) -> np.ndarray:
    """Camera image (RGB, any size) -> uint8 (height, width, 3), INTER_AREA as in the paper."""
    h, w = size
    if image is None:
        return np.zeros((h, w, 3), np.uint8)
    if image.shape[:2] != (h, w):
        image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
    return np.asarray(image, dtype=np.uint8)


def action_to_command(action: np.ndarray, max_linear_vel: float, max_angular_vel: float,
                      allow_reverse: bool = False) -> Tuple[float, float]:
    """Normalized action [a_v, a_w] in [-1, 1]^2 -> (linear.x [m/s], angular.z [rad/s])."""
    a = np.clip(np.asarray(action, dtype=np.float64).reshape(-1)[:2], -1.0, 1.0)
    v = float(a[0]) * max_linear_vel
    if not allow_reverse:
        v = max(v, 0.0)
    return v, float(a[1]) * max_angular_vel


def compute_reward(weights: Dict[str, float], reached_goal: bool, collided: bool,
                   prev_distance: float, distance: float,
                   action: np.ndarray, prev_action: Optional[np.ndarray]) -> Tuple[float, Dict[str, float]]:
    """
    R = r_goal + r_dist + r_collision + r_time + r_smooth  (paper, Sec. IV-C)

        r_goal      = goal                          if the goal is reached
        r_dist      = progress * (d_{t-1} - d_t)    distance to the goal gained this step
        r_collision = collision                     on a collision
        r_time      = time                          every step
        r_smooth    = -smooth * |a_w,t - a_w,t-1|   (not on the terminal step)
    """
    terms = {
        "goal": weights["goal"] if reached_goal else 0.0,
        "dist": weights["progress"] * (prev_distance - distance) if math.isfinite(prev_distance) else 0.0,
        "collision": weights["collision"] if collided else 0.0,
        "time": weights["time"],
        "smooth": 0.0,
    }
    if prev_action is not None and not reached_goal and not collided:
        terms["smooth"] = -weights["smooth"] * abs(float(action[1]) - float(prev_action[1]))
    return float(sum(terms.values())), terms


# ---------------------------------------------------------------------- environment
class WhillEnv(gym.Env):
    """
    Observation (Dict, keys from task.observation.keys):
        image         uint8 (H, W, 3)     front camera
        is_success    float32 ()          1 on the step the goal is reached
        is_collision  float32 ()          1 on the step of a collision
        goal          float32 (2,)        distance [m] and bearing [rad] of the goal in base_link (optional)
        velocity      float32 (2,)        measured v [m/s], w [rad/s] (optional)
    Action: Box([-1, 1]^2) -> action_to_command()
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, task: Task, data_manager=None, seed: Optional[int] = None):
        super().__init__()
        self.task = task
        self._dm = data_manager          # created on the first reset() if None (needs ROS 2)
        self._rng = np.random.default_rng(seed)
        o = task.observation
        self._size = tuple(o["image_size"])
        self._keys: List[str] = list(o["keys"])
        self._dr = DomainRandomization() if o["domain_randomization"] else None

        all_spaces = {
            "image": spaces.Box(0, 255, (self._size[0], self._size[1], 3), np.uint8),
            "is_success": spaces.Box(0.0, 1.0, (), np.float32),
            "is_collision": spaces.Box(0.0, 1.0, (), np.float32),
            "goal": spaces.Box(np.array([0.0, -math.pi], np.float32), np.array([np.inf, math.pi], np.float32), dtype=np.float32),
            "velocity": spaces.Box(-np.inf, np.inf, (2,), np.float32),
        }
        self.observation_space = spaces.Dict({k: all_spaces[k] for k in self._keys})
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)

        self.episode: Optional[Episode] = None
        self._steps = 0
        self._start_time = 0.0
        self._stamp = 0.0
        self._prev_distance = math.inf
        self._prev_action: Optional[np.ndarray] = None
        self._last_image: Optional[np.ndarray] = None
        self._trajectory: List[Tuple[float, float]] = []
        self._frame_timeouts = 0

    # -------------------------------------------------------------- gym API
    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        if self._dm is None:
            from .data_manager import DataManager
            self._dm = DataManager(self.task.topics, self.task.cfg["awsim"])
            if not self._dm.wait_for_data(timeout=10.0):
                raise RuntimeError("no camera image / odometry from AWSIM within 10 s "
                                   "(is AWSIM running? same ROS_DOMAIN_ID? topics in the task file?)")

        self.episode = self.task.sample(self._rng)
        if not self._dm.reset_scenario(self.episode, timeout=self.task.episode["reset_timeout_sec"]):
            print("Warning: AWSIM did not acknowledge the reset; continuing")

        # first observation: a camera frame rendered after the reset (not one still in the pipeline)
        if not self._dm.wait_for_frame(self._dm.reset_time, 3 * self.task.episode["frame_timeout_sec"]):
            print("Warning: no camera frame rendered after the reset; the first observation may be stale")
        s = self._dm.snapshot()
        self._stamp = s.image_stamp
        self._steps = 0
        self._start_time = time.time()
        self._prev_distance = self._distance_to_goal(s)
        self._prev_action = None
        self._trajectory = [(s.x, s.y)]
        self._frame_timeouts = 0
        return self._observation(s, False, False), self._info(s)

    def step(self, action: np.ndarray):
        a = self.task.action
        v, w = action_to_command(action, a["max_linear_vel"], a["max_angular_vel"], a["allow_reverse"])
        self._dm.send_command(v, w)

        # Decision step = arrival of the next camera frame; everything else is sampled at that moment.
        if not self._dm.wait_for_frame(self._stamp, self.task.episode["frame_timeout_sec"]):
            self._frame_timeouts += 1
            print(f"Warning: no camera frame within {self.task.episode['frame_timeout_sec']} s (AWSIM lagging?)")
        s = self._dm.snapshot()
        self._stamp = s.image_stamp
        self._steps += 1

        distance = self._distance_to_goal(s)
        reached = distance <= self.task.goal_radius
        collided = s.collision
        action = np.asarray(action, dtype=np.float32)
        reward, terms = compute_reward(self.task.reward, reached, collided, self._prev_distance, distance,
                                       action, self._prev_action)
        self._prev_distance = distance
        self._prev_action = action.copy()
        self._trajectory.append((s.x, s.y))

        terminated = reached or collided
        e = self.task.episode
        truncated = (not terminated) and (
            (e["max_steps"] is not None and self._steps >= e["max_steps"])
            or (e["timeout_sec"] is not None and time.time() - self._start_time >= e["timeout_sec"]))

        info = self._info(s)
        info.update(reward_terms=terms, command=(v, w), distance_to_goal=distance,
                    is_success=reached, is_collision=collided)
        if terminated or truncated:
            self._dm.send_command(0.0, 0.0)
            info["trajectory"] = np.array(self._trajectory, np.float32)
            info["outcome"] = "success" if reached else "collision" if collided else "timeout"
        return self._observation(s, reached, collided), reward, terminated, truncated, info

    def render(self):
        return self._last_image

    def close(self):
        if self._dm is not None:
            self._dm.close()
            self._dm = None

    # -------------------------------------------------------------- helpers
    def _goal_xy(self, s) -> Optional[Tuple[float, float]]:
        if self.episode is not None and self.episode.goal is not None:
            return self.episode.goal.x, self.episode.goal.y
        return s.goal   # goal object of the scene, as published by AWSIM

    def _distance_to_goal(self, s) -> float:
        g = self._goal_xy(s)
        return math.inf if g is None else math.hypot(g[0] - s.x, g[1] - s.y)

    def _observation(self, s, reached: bool, collided: bool) -> Dict[str, np.ndarray]:
        image = process_image(s.image, self._size)
        if self._dr is not None:
            image = self._dr.apply(image)
        self._last_image = image
        obs = {"image": image,
               "is_success": np.float32(reached),
               "is_collision": np.float32(collided)}
        if "goal" in self._keys:
            g = self._goal_xy(s)
            if g is None:
                obs["goal"] = np.zeros(2, np.float32)
            else:
                bearing = math.atan2(g[1] - s.y, g[0] - s.x) - s.yaw
                bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
                obs["goal"] = np.array([math.hypot(g[0] - s.x, g[1] - s.y), bearing], np.float32)
        if "velocity" in self._keys:
            obs["velocity"] = np.array([s.linear_vel, s.angular_vel], np.float32)
        return {k: obs[k] for k in self._keys}

    def _info(self, s) -> Dict[str, Any]:
        g = self._goal_xy(s)
        return {"step": self._steps, "pose": (s.x, s.y, s.yaw), "goal": g, "obstacles": s.obstacles,
                "elapsed": time.time() - self._start_time, "frame_timeouts": self._frame_timeouts}
