"""
Models (E2E policies). This is where models are implemented; the environment is in rl/whill_env/.

    base.py          Agent: the interface every model implements (read this first)
    my_agent.py      template for your own model (runs as it is)
    random_agent.py  random / constant actions (checks the environment)
    dreamerv3/       DreamerV3 (paper), wrapping third_party/dreamerv3

Add your agent to AGENTS below, or pass "package.module:ClassName" to --agent.
"""

import importlib
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from .base import Agent  # noqa: F401

AGENTS = {
    "dreamerv3": "agents.dreamerv3.agent:DreamerV3Agent",
    "random": "agents.random_agent:RandomAgent",
    "my_agent": "agents.my_agent:MyAgent",
}


def agent_class(name: str):
    target = AGENTS.get(name, name)
    if ":" not in target:
        raise KeyError(f"unknown agent '{name}' (known: {', '.join(AGENTS)}; or give package.module:ClassName)")
    module, cls = target.split(":")
    return getattr(importlib.import_module(module), cls)


def agent_config(name: str, config_file: Optional[str] = None, overrides=()) -> Dict[str, Any]:
    """Default config of the agent <- config_file <- overrides ("a.b=value", value parsed as YAML)."""
    cls = agent_class(name)
    config: Dict[str, Any] = {}
    if cls.default_config is not None and Path(cls.default_config).exists():
        config = yaml.safe_load(Path(cls.default_config).read_text()) or {}
    if config_file:
        _update(config, yaml.safe_load(Path(config_file).read_text()) or {})
    for item in overrides:
        key, _, value = item.partition("=")
        node = config
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = yaml.safe_load(value)
    return config


def create_agent(name: str, observation_space, action_space, config: Dict[str, Any], logdir) -> "Agent":
    return agent_class(name)(observation_space, action_space, config, logdir)


def _update(base: Dict[str, Any], new: Dict[str, Any]):
    for k, v in new.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _update(base[k], v)
        else:
            base[k] = v
