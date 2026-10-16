"""Agents through the episode loop with the fake data manager of test_env.py (no AWSIM)."""
import numpy as np
import pytest

import agents
from loop import run_episode
from test_env import FakeDataManager, make


@pytest.mark.parametrize("name", ["random", "my_agent"])
def test_agent_runs_and_checkpoints(name, tmp_path):
    env = make(FakeDataManager())
    cfg = agents.agent_config(name)
    agent = agents.create_agent(name, env.observation_space, env.action_space, cfg, tmp_path)
    stats = run_episode(env, agent, training=True)
    assert stats["outcome"] in ("success", "collision", "timeout") and stats["length"] > 0
    assert len(stats["trajectory"]) == stats["length"] + 1
    agent.save(tmp_path)
    agent.load(tmp_path)


def test_constant_agent_reaches_goal():
    env = make(FakeDataManager())
    cfg = agents.agent_config("random", overrides=["mode=constant", "constant=[1.0, 0.0]"])
    agent = agents.create_agent("random", env.observation_space, env.action_space, cfg, None)
    assert run_episode(env, agent, training=False)["outcome"] == "success"


def test_unknown_agent():
    with pytest.raises(KeyError):
        agents.agent_class("nope")
