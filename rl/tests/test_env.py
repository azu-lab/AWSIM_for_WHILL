"""WhillEnv with a fake data manager: reward, termination and action scaling without AWSIM."""
import math

import numpy as np
import pytest

from whill_env.data_manager import Snapshot
from whill_env.env import WhillEnv, action_to_command, compute_reward
from whill_env.task import Task


class FakeDataManager:
    """Unicycle at 10 frames/s; collision when x passes `wall_x`."""

    def __init__(self, wall_x=None):
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.stamp = 0.0
        self.wall_x = wall_x
        self.commands = []
        self.reset_time = 0.0

    def reset_scenario(self, episode, timeout):
        s = episode.start
        self.x, self.y, self.yaw = (s.x, s.y, s.yaw) if s else (0.0, 0.0, 0.0)
        return True

    def send_command(self, v, w):
        self.commands.append((v, w))
        self.v, self.w = v, w

    def wait_for_frame(self, after, timeout):
        self.stamp += 0.1
        self.yaw += self.w * 0.1
        self.x += self.v * 0.1 * math.cos(self.yaw)
        self.y += self.v * 0.1 * math.sin(self.yaw)
        return True

    def snapshot(self, consume_collision=True):
        hit = self.wall_x is not None and self.x >= self.wall_x
        return Snapshot(np.full((120, 160, 3), 128, np.uint8), self.stamp, self.x, self.y, 0.0, self.yaw,
                        self.v, self.w, None, (), hit)

    def close(self):
        pass


def make(dm, **task):
    base = {"start": {"poses": [{"x": 0, "y": 0}]}, "goal": {"ahead": 1.0, "radius": 0.2},
            "action": {"max_linear_vel": 1.0, "max_angular_vel": 1.0}, "episode": {"max_steps": 50}}
    base.update(task)
    return WhillEnv(Task(base), data_manager=dm, seed=0)


def test_action_to_command():
    assert action_to_command([0.5, -1.0], 0.3, 0.6) == pytest.approx((0.15, -0.6))
    assert action_to_command([-1.0, 2.0], 0.3, 0.6) == pytest.approx((0.0, 0.6))          # no reverse, clipped
    assert action_to_command([-1.0, 0.0], 0.3, 0.6, allow_reverse=True)[0] == pytest.approx(-0.3)


def test_compute_reward_terms():
    w = {"goal": 100, "progress": 30, "collision": -50, "time": -0.05, "smooth": 1.0}
    r, t = compute_reward(w, False, False, 2.0, 1.9, np.array([0, 0.5]), np.array([0, 0.0]))
    assert t["dist"] == pytest.approx(3.0) and t["smooth"] == pytest.approx(-0.5)
    assert r == pytest.approx(3.0 - 0.05 - 0.5)
    r, t = compute_reward(w, True, False, 0.3, 0.2, np.array([0, 1.0]), np.array([0, 0.0]))
    assert t["goal"] == 100 and t["smooth"] == 0.0


def test_reaches_goal():
    env = make(FakeDataManager())
    obs, info = env.reset()
    assert obs["image"].shape == (64, 64, 3) and set(obs) == {"image", "is_success", "is_collision"}
    for _ in range(50):
        obs, r, term, trunc, info = env.step(np.array([1.0, 0.0], np.float32))
        if term or trunc:
            break
    assert term and info["outcome"] == "success" and obs["is_success"] == 1.0
    assert env._dm.commands[-1] == (0.0, 0.0)            # stopped at the end


def test_collision_terminates():
    env = make(FakeDataManager(wall_x=0.35))
    env.reset()
    for _ in range(50):
        obs, r, term, trunc, info = env.step(np.array([1.0, 0.0], np.float32))
        if term or trunc:
            break
    assert info["outcome"] == "collision" and r < -40 and obs["is_collision"] == 1.0


def test_step_limit_truncates():
    env = make(FakeDataManager(), episode={"max_steps": 5})
    env.reset()
    for i in range(5):
        _, _, term, trunc, info = env.step(np.zeros(2, np.float32))
    assert trunc and not term and info["outcome"] == "timeout"


def test_optional_observation_keys():
    env = make(FakeDataManager(), observation={"keys": ["image", "goal", "velocity"]})
    obs, _ = env.reset()
    assert obs["goal"][0] == pytest.approx(1.0) and obs["goal"][1] == pytest.approx(0.0)
    assert env.observation_space.contains(obs)
