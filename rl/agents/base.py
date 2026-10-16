"""
Interface between a model (E2E policy) and the learning environment.

To use your own model, implement a subclass of Agent (start from my_agent.py), register it in
agents/__init__.py (or pass "package.module:ClassName" to --agent), and run it with the same
commands as DreamerV3:

    python3 rl/train.py    --agent my_agent --task rl/configs/tasks/shinjuku_box.yaml --logdir logs/rl/mine
    python3 rl/evaluate.py --logdir logs/rl/mine

The environment (rl/whill_env/) is the same for every agent. An agent only sees:

    obs     dict of numpy arrays, keys/shapes = observation_space (task.observation.keys), e.g.
            obs["image"]: uint8 (64, 64, 3) RGB of the front camera
    action  float32 (2,) in [-1, 1]: [linear velocity, angular velocity], scaled by the task limits
    reward  float, terminated (goal or collision), truncated (step/time limit)

Call order, per episode (train.py / evaluate.py):

    begin_episode(obs, training)
    repeat: action = act(obs, training)
            next_obs, reward, terminated, truncated, info = env.step(action)
            observe(action, reward, next_obs, terminated, truncated, info)    # learning goes here
            obs = next_obs
    end_episode(stats, training)

save(directory) / load(directory) are called by train.py for checkpoints and resuming, and by
evaluate.py to load a trained model.
"""

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np


class Agent:
    """Base class of all models. Override act(); override the rest when the model learns or has state."""

    #: YAML with the default configuration of the agent (merged with --agent-config and --set)
    default_config: Optional[Path] = None

    def __init__(self, observation_space, action_space, config: Dict[str, Any], logdir: Optional[Path]):
        """
        observation_space: gymnasium.spaces.Dict, action_space: gymnasium.spaces.Box([-1, 1]^2)
        config: agent configuration (dict), logdir: run directory (None in evaluate.py without --logdir)
        """
        self.observation_space = observation_space
        self.action_space = action_space
        self.config = config
        self.logdir = Path(logdir) if logdir is not None else None

    def begin_episode(self, obs: Dict[str, np.ndarray], training: bool) -> None:
        """A new episode starts with observation `obs` (reset recurrent state here)."""

    def act(self, obs: Dict[str, np.ndarray], training: bool) -> np.ndarray:
        """Action for `obs`: float32 array (2,) in [-1, 1]."""
        raise NotImplementedError

    def observe(self, action: np.ndarray, reward: float, next_obs: Dict[str, np.ndarray],
                terminated: bool, truncated: bool, info: Dict[str, Any], training: bool) -> Dict[str, float]:
        """Result of `action`. Store the transition / update the model here. Returns metrics to log."""
        return {}

    def end_episode(self, stats: Dict[str, Any], training: bool) -> None:
        """stats: reward, length, outcome ("success" / "collision" / "timeout"), trajectory, goal, ..."""

    def save(self, directory: Path) -> None:
        """Write a checkpoint into `directory` (it exists)."""

    def load(self, directory: Path) -> None:
        """Read a checkpoint written by save()."""

    def close(self) -> None:
        pass
