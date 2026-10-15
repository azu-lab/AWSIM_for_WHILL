"""
gymnasium の環境を DreamerV3 の embodied.Env として包むアダプタ。

third_party/dreamerv3 (b65cf81) の embodied/envs/from_gym.py は旧 OpenAI gym の API
(reset() -> obs, step() -> (obs, reward, done, info)) 前提で、しかも `import gym` する。
本リポジトリの環境 (env.py / obstacle_avoidance_env.py) は gymnasium の API
(reset() -> (obs, info), step() -> (obs, reward, terminated, truncated, info)) なので、
from_gym.FromGym をそのまま使うと reset/step の戻り値の展開で落ちる。

soturon でどう繋いでいたかはリポジトリに残っていない (dreamerv3 は .gitmodules の無い
gitlink で、手元の改変は失われている)。third_party を書き換えずに済むよう、
FromGym と同じ観測・行動の扱いでここに実装し直した。

- is_last     = terminated or truncated
- is_terminal = terminated  (タイムアウトは終端ではない)
- 直近の info は .info で取れる (軌跡など、観測に入れられない可変長データ用)
"""

import functools

import elements
import embodied
import numpy as np


class FromGymnasium(embodied.Env):

    def __init__(self, env, obs_key='image', act_key='action'):
        self._env = env
        self._obs_dict = hasattr(self._env.observation_space, 'spaces')
        self._act_dict = hasattr(self._env.action_space, 'spaces')
        self._obs_key = obs_key
        self._act_key = act_key
        self._done = True
        self._info = {}

    @property
    def env(self):
        return self._env

    @property
    def info(self):
        return self._info

    @functools.cached_property
    def obs_space(self):
        if self._obs_dict:
            spaces = self._flatten(self._env.observation_space.spaces)
        else:
            spaces = {self._obs_key: self._env.observation_space}
        spaces = {k: self._convert(v) for k, v in spaces.items()}
        return {
            **spaces,
            'reward': elements.Space(np.float32),
            'is_first': elements.Space(bool),
            'is_last': elements.Space(bool),
            'is_terminal': elements.Space(bool),
        }

    @functools.cached_property
    def act_space(self):
        if self._act_dict:
            spaces = self._flatten(self._env.action_space.spaces)
        else:
            spaces = {self._act_key: self._env.action_space}
        spaces = {k: self._convert(v) for k, v in spaces.items()}
        spaces['reset'] = elements.Space(bool)
        return spaces

    def step(self, action):
        if action['reset'] or self._done:
            self._done = False
            obs, self._info = self._env.reset()
            return self._obs(obs, 0.0, is_first=True)
        if self._act_dict:
            action = self._unflatten(action)
        else:
            action = action[self._act_key]
        obs, reward, terminated, truncated, self._info = self._env.step(action)
        self._done = bool(terminated or truncated)
        return self._obs(obs, reward, is_last=self._done, is_terminal=bool(terminated))

    def _obs(self, obs, reward, is_first=False, is_last=False, is_terminal=False):
        if not self._obs_dict:
            obs = {self._obs_key: obs}
        obs = self._flatten(obs)
        obs = {k: np.asarray(v) for k, v in obs.items()}
        obs.update(
            reward=np.float32(reward),
            is_first=is_first,
            is_last=is_last,
            is_terminal=is_terminal)
        return obs

    def render(self):
        return self._env.render()

    def close(self):
        try:
            self._env.close()
        except Exception:
            pass

    def _flatten(self, nest, prefix=None):
        result = {}
        for key, value in nest.items():
            key = prefix + '/' + key if prefix else key
            if hasattr(value, 'spaces') and isinstance(value.spaces, dict):
                value = value.spaces
            if isinstance(value, dict):
                result.update(self._flatten(value, key))
            else:
                result[key] = value
        return result

    def _unflatten(self, flat):
        result = {}
        for key, value in flat.items():
            parts = key.split('/')
            node = result
            for part in parts[:-1]:
                node = node.setdefault(part, {})
            node[parts[-1]] = value
        return result

    def _convert(self, space):
        if hasattr(space, 'n'):
            return elements.Space(np.int32, (), 0, space.n)
        return elements.Space(space.dtype, space.shape, space.low, space.high)
