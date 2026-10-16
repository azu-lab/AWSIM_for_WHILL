"""
Template for your own model. Copy this file (e.g. to agents/ppo_agent.py), rename the class,
register it in agents/__init__.py, and fill in the three marked places:

    1. build the model                      -> __init__
    2. compute the action from the input    -> act
    3. learn from the result                -> observe (and/or end_episode)

It runs as it is (it drives slowly ahead with a little random steering), so the whole pipeline
can be checked first:

    python3 rl/train.py --agent my_agent --task rl/configs/tasks/shinjuku_box.yaml \
        --logdir logs/rl/my_agent_test --steps 300

Inputs (obs) and the action are described in agents/base.py. The environment, the reward and
the reset are not your concern here: they are in rl/whill_env/ and in the task YAML.
"""

from collections import deque
from pathlib import Path

import numpy as np

from .base import Agent


class MyAgent(Agent):

    # Defaults of the configuration; override with --agent-config FILE or --set key=value
    default_config = Path(__file__).with_name("my_agent.yaml")

    def __init__(self, observation_space, action_space, config, logdir):
        super().__init__(observation_space, action_space, config, logdir)
        self._rng = np.random.default_rng(config["seed"])
        # ---- 1. build the model -------------------------------------------------------------
        # e.g. self.net = MyNetwork(image_shape=observation_space["image"].shape, ...)
        #      self.optimizer = ...
        self.steering_noise = float(config["steering_noise"])
        self.buffer = deque(maxlen=int(config["buffer_size"]))   # (obs, action, reward, next_obs, done)
        self.updates = 0

    def begin_episode(self, obs, training):
        self._prev_obs = obs
        # reset the hidden state of a recurrent model here

    def act(self, obs, training):
        # ---- 2. compute the action -----------------------------------------------------------
        # obs["image"]: uint8 (64, 64, 3). Return [linear, angular] in [-1, 1].
        # e.g. return self.net(obs["image"] / 255.0, explore=training)
        steer = self._rng.normal(0.0, self.steering_noise) if training else 0.0
        return np.array([0.5, np.clip(steer, -1.0, 1.0)], np.float32)

    def observe(self, action, reward, next_obs, terminated, truncated, info, training):
        if not training:
            return {}
        # ---- 3. learn ------------------------------------------------------------------------
        # `terminated` (goal / collision) ends the return; `truncated` (time limit) does not.
        self.buffer.append((self._prev_obs, action, reward, next_obs, terminated))
        self._prev_obs = next_obs
        # e.g. if len(self.buffer) >= batch_size: loss = self.update(sample(self.buffer)); self.updates += 1
        return {"buffer_size": len(self.buffer)}

    def end_episode(self, stats, training):
        pass   # e.g. on-policy update at the end of the episode

    def save(self, directory):
        # write the model parameters (and optimizer state) into `directory`
        np.savez(Path(directory) / "my_agent.npz", steering_noise=self.steering_noise, updates=self.updates)

    def load(self, directory):
        data = np.load(Path(directory) / "my_agent.npz")
        self.steering_noise = float(data["steering_noise"])
        self.updates = int(data["updates"])
