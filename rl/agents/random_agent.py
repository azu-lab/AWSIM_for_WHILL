"""Random (or constant) actions: checks the environment without any model."""

from pathlib import Path

import numpy as np

from .base import Agent


class RandomAgent(Agent):
    """
    config:
        mode: random      # random: uniform in [-1, 1]^2;  constant: always `constant`
        constant: [1.0, 0.0]
        seed: 0
    """

    default_config = Path(__file__).with_name("random_agent.yaml")

    def __init__(self, observation_space, action_space, config, logdir):
        super().__init__(observation_space, action_space, config, logdir)
        self._rng = np.random.default_rng(config.get("seed", 0))

    def act(self, obs, training):
        if self.config.get("mode", "random") == "constant":
            return np.asarray(self.config.get("constant", [1.0, 0.0]), np.float32)
        return self._rng.uniform(-1.0, 1.0, size=2).astype(np.float32)
