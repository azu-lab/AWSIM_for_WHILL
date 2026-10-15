"""Environment YAML loader (no ROS 2 / AWSIM needed)."""

import importlib
from pathlib import Path

import pytest

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs" / "env"


@pytest.fixture
def modules():
    # Fresh module state for every test (apply_env_config mutates module constants)
    import awsim_dreamerv3.config as base
    import awsim_dreamerv3.obstacle_avoidance_config as oa
    importlib.reload(base)
    importlib.reload(oa)
    import awsim_dreamerv3.env_config as env_config
    importlib.reload(env_config)
    return base, oa, env_config


def test_obstacle_avoidance_yaml_equals_code_defaults(modules):
    base, oa, env_config = modules
    defaults = {k: getattr(oa, k) for k in dir(oa) if k.isupper()}
    applied = env_config.apply_env_config(str(CONFIG_DIR / "obstacle_avoidance.yaml"))
    assert applied["task"] == "obstacle_avoidance"
    for key, value in applied.items():
        if key in defaults:
            assert defaults[key] == value, key


def test_original_yaml_equals_code_defaults(modules):
    base, oa, env_config = modules
    defaults = {k: getattr(base, k) for k in dir(base) if k.isupper()}
    applied = env_config.apply_env_config(str(CONFIG_DIR / "original.yaml"))
    assert applied["task"] == "original"
    for key, value in applied.items():
        if key in defaults:
            assert defaults[key] == value, key


def test_override_and_unknown_key(modules, tmp_path):
    base, oa, env_config = modules
    cfg = tmp_path / "env.yaml"
    cfg.write_text("task: obstacle_avoidance\nreward: {goal: 10.0, progress_scale: 5}\n"
                   "episode: {start_pose: {x: 1.0, y: 2.0, yaw: 0.5}}\n")
    env_config.apply_env_config(str(cfg))
    assert oa.GOAL_REWARD == 10.0
    assert oa.FORWARD_REWARD_SCALE == 5.0
    assert base.START_POSE == {"x": 1.0, "y": 2.0, "yaw": 0.5}

    bad = tmp_path / "bad.yaml"
    bad.write_text("task: obstacle_avoidance\nreward: {gaol: 1}\n")
    with pytest.raises(KeyError):
        env_config.apply_env_config(str(bad))
