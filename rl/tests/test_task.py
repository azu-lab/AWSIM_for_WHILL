import math
from pathlib import Path

import numpy as np
import pytest

from whill_env.task import Task, load_task

TASKS = sorted((Path(__file__).resolve().parents[1] / "configs" / "tasks").glob("*.yaml"))


@pytest.mark.parametrize("path", TASKS, ids=lambda p: p.stem)
def test_task_files_load(path):
    task = load_task(str(path))
    episode = task.sample(np.random.default_rng(0))
    assert task.name == path.stem
    assert (episode.start is None) == (not task.start_poses)


def test_goal_ahead_follows_start_yaw():
    task = Task({"start": {"poses": [{"x": 1.0, "y": 2.0, "yaw": math.pi / 2}]}, "goal": {"ahead": 4.0, "left": 1.0}})
    e = task.sample(np.random.default_rng(0))
    assert e.goal.x == pytest.approx(1.0 - 1.0)   # left of +y is -x
    assert e.goal.y == pytest.approx(2.0 + 4.0)


def test_noise_is_bounded():
    task = Task({"start": {"poses": [{"x": 0, "y": 0}], "noise": {"x": 0.5, "y": 0.0, "yaw": 0.1}},
                 "goal": {"position": {"x": 5, "y": 0}, "noise": {"x": 0.0, "y": 0.2}}})
    rng = np.random.default_rng(1)
    for _ in range(100):
        e = task.sample(rng)
        assert abs(e.start.x) <= 0.5 and e.start.y == 0 and abs(e.start.yaw) <= 0.1
        assert e.goal.x == 5 and abs(e.goal.y) <= 0.2


def test_unknown_keys_are_rejected():
    with pytest.raises(KeyError):
        Task({"reward": {"gaol": 1.0}})
    with pytest.raises(ValueError):
        Task({"goal": {"ahead": 3.0}})                 # needs a start pose
    with pytest.raises(ValueError):
        Task({"observation": {"keys": ["lidar"]}})
