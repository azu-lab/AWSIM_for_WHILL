"""
DreamerV3 (third_party/dreamerv3 @ b65cf81) as an Agent of this repository.

The training procedure is the loop of soturon (rl/scripts/train.py there): every environment step
is added to the replay buffer, and one gradient step is made every `loop.train_every` steps once
the buffer holds a batch. Not dreamerv3/main.py (which drives its own environments).

Files in the run directory:
    config.yaml                    full DreamerV3 configuration
    metrics.jsonl, events.out.*    losses, episode metrics, report images (TensorBoard)
    replay/                        replay buffer chunks (saved with every checkpoint, reloaded on resume)
    checkpoints/<step>/dreamerv3.pkl   agent state (about 460 MB with the default sizes)
"""

import io
import json
import pickle
import sys
import tempfile
from collections import deque
from pathlib import Path

import numpy as np

from ..base import Agent

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "third_party" / "dreamerv3"))

EPISODE_KEYS = ("reward", "is_first", "is_last", "is_terminal")


def _space(space):
    import elements
    if hasattr(space, "n"):
        return elements.Space(np.int32, (), 0, space.n)
    return elements.Space(space.dtype, space.shape, space.low, space.high)


def _update(base, new):
    for k, v in new.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _update(base[k], v)
        elif not (isinstance(v, list) and not v):   # elements.Config does not accept empty lists
            base[k] = v


class DreamerV3Agent(Agent):

    default_config = Path(__file__).with_name("config.yaml")

    def __init__(self, observation_space, action_space, config, logdir):
        super().__init__(observation_space, action_space, config, logdir)
        import elements
        import ruamel.yaml as ryaml
        from dreamerv3.agent import Agent as Dreamer
        from .jax_compat import adjust_jax_config

        self.loop = dict(config.get("loop", {}))
        overrides = {k: v for k, v in config.items() if k != "loop"}
        defaults = ryaml.YAML(typ="safe").load(
            (REPO_ROOT / "third_party" / "dreamerv3" / "dreamerv3" / "configs.yaml").read_text())["defaults"]
        _update(defaults, overrides)
        workdir = self.logdir or Path(tempfile.mkdtemp(prefix="dreamerv3_"))
        cfg = elements.Config(defaults).update({
            "logdir": str(workdir), "task": "awsim_whill", "replay_context": 0})
        self.cfg = cfg

        self.obs_space = {k: _space(v) for k, v in observation_space.spaces.items()}
        self.obs_space.update(reward=elements.Space(np.float32), is_first=elements.Space(bool),
                              is_last=elements.Space(bool), is_terminal=elements.Space(bool))
        self.act_space = {"action": _space(action_space)}
        self.agent = Dreamer(self.obs_space, self.act_space, elements.Config({
            **cfg.agent,
            "logdir": cfg.logdir,
            "seed": cfg.seed,
            "jax": adjust_jax_config(cfg.jax, self.loop.get("compute_dtype")),
            "batch_size": cfg.batch_size,
            "batch_length": cfg.batch_length,
            "replay_context": cfg.replay_context,
            "report_length": cfg.report_length,
            "replica": 0,
            "replicas": 1,
        }))
        self.step = elements.Counter()
        self.replay = None
        self.logger = None
        self.train_carry = None
        self.history = {"success": deque(maxlen=self.loop.get("history", 100)),
                        "collision": deque(maxlen=self.loop.get("history", 100))}

    # ------------------------------------------------------------------ training resources
    def _ensure_training(self):
        if self.replay is not None:
            return
        import elements
        import embodied
        assert self.logdir is not None, "training needs a run directory"
        self.cfg.save(elements.Path(self.logdir) / "config.yaml")
        self.replay = embodied.replay.Replay(
            length=self.cfg.batch_length + self.cfg.replay_context,
            capacity=int(self.cfg.replay.size),
            directory=elements.Path(self.logdir) / "replay")
        self.replay.load()   # chunks of a previous run (resume); nothing on a new run
        if len(self.replay):
            print(f"[DreamerV3] replay buffer: {len(self.replay)} items loaded from {self.logdir}/replay")
        self.logger = elements.Logger(self.step, [
            elements.logger.TerminalOutput(self.cfg.logger.filter, "Agent"),
            elements.logger.JSONLOutput(self.logdir, "metrics.jsonl"),
            elements.logger.TensorBoardOutput(self.logdir),
        ], 1)

    # ------------------------------------------------------------------ Agent API
    def _embodied(self, obs, reward, is_first, is_last, is_terminal):
        out = {k: np.asarray(v) for k, v in obs.items()}
        out.update(reward=np.float32(reward), is_first=np.bool_(is_first),
                   is_last=np.bool_(is_last), is_terminal=np.bool_(is_terminal))
        return out

    def begin_episode(self, obs, training):
        self._obs = self._embodied(obs, 0.0, True, False, False)
        self._carry = self.agent.init_policy(batch_size=1)
        self._length = 0
        if training:
            self._ensure_training()

    def act(self, obs, training):
        batch = {k: v[None] for k, v in self._obs.items()}
        self._carry, action, _ = self.agent.policy(self._carry, batch, mode="train" if training else "eval")
        return np.asarray(action["action"][0], np.float32)

    def observe(self, action, reward, next_obs, terminated, truncated, info, training):
        if training:
            self._add(self._obs, action)
        self._obs = self._embodied(next_obs, reward, False, terminated or truncated, terminated)
        self._length += 1
        if not training:
            return {}
        self.step.increment()
        step = int(self.step)
        metrics = {}
        if len(self.replay) >= self.cfg.batch_size and step % int(self.loop.get("train_every", 2)) == 0:
            metrics = self._train(step)
        return metrics

    def end_episode(self, stats, training):
        if not training:
            return
        # last observation of the episode (its action is never used)
        self._add(self._obs, np.zeros(self.act_space["action"].shape, np.float32))
        self.history["success"].append(float(stats["outcome"] == "success"))
        self.history["collision"].append(float(stats["outcome"] == "collision"))
        self.logger.scalar("episode/reward", stats["reward"])
        self.logger.scalar("episode/length", stats["length"])
        self.logger.scalar("eval/success_rate", float(np.mean(self.history["success"])))
        self.logger.scalar("eval/collision_rate", float(np.mean(self.history["collision"])))
        if stats.get("trajectory") is not None:
            try:
                self.logger.image("eval/trajectory", _plot_trajectory(stats))
            except Exception as e:  # noqa: BLE001
                print(f"[DreamerV3] trajectory plot failed: {e}")
        self.logger.write()

    def save(self, directory):
        with open(Path(directory) / "dreamerv3.pkl", "wb") as f:
            pickle.dump(self.agent.save(), f)
        (Path(directory) / "dreamerv3_state.json").write_text(json.dumps({"step": int(self.step)}))
        if self.replay is not None:
            self.replay.save()

    def load(self, directory):
        with open(Path(directory) / "dreamerv3.pkl", "rb") as f:
            data = pickle.load(f)
        # Published checkpoints (rl/tools/export_checkpoint.py) leave out the optimizer state ("opt/..."):
        # missing parameters keep their freshly initialized values.
        missing = set(self.agent.save()["params"]) - set(data["params"])
        if missing:
            current = self.agent.save()["params"]
            data["params"] = {**{k: current[k] for k in missing}, **data["params"]}
            groups = sorted({k.split("/")[0] for k in missing})
            print(f"[DreamerV3] {len(missing)} parameters not in the checkpoint ({', '.join(groups)}): initialized")
        self.agent.load(data)
        state = Path(directory) / "dreamerv3_state.json"
        if state.exists():
            self.step.value = json.loads(state.read_text())["step"]

    def close(self):
        if self.logger is not None:
            self.logger.close()

    # ------------------------------------------------------------------ internals
    def _add(self, obs, action):
        transition = dict(obs)
        transition["action"] = np.asarray(action, np.float32)
        transition["consec"] = np.int32(self._length)
        transition["stepid"] = np.zeros(20, np.uint8)   # overwritten by the replay buffer
        self.replay.add(transition)

    def _train(self, step):
        from embodied.jax import internal
        batch = self.replay.sample(self.cfg.batch_size)
        for key in ("is_first", "is_last", "is_terminal"):
            if key in batch and batch[key].dtype == object:
                batch[key] = np.asarray(batch[key], np.bool_)
        if "action" in batch and batch["action"].dtype == object:
            batch["action"] = np.asarray(batch["action"], np.float32)
        batch = internal.device_put(batch, self.agent.train_sharded)
        batch["seed"] = self.agent._seeds(step, self.agent.train_mirrored)
        if self.train_carry is None:
            self.train_carry = self.agent.init_train(self.cfg.batch_size)
        self.train_carry, _, metrics = self.agent.train(self.train_carry, batch)
        if step % int(self.loop.get("log_every", 100)) == 0:
            for key, value in metrics.items():
                self.logger.scalar(key, value)
            self.logger.write()
        if step % int(self.loop.get("report_every", 10000)) == 0:
            self._report(batch, step)
        return {}

    def _report(self, batch, step):
        """World-model reconstructions for TensorBoard (as soturon)."""
        import jax
        from embodied.jax import internal
        try:
            with jax.transfer_guard("allow"):
                n = int(self.cfg.report_length)
                rb = {k: (np.asarray(v)[:, :n] if np.asarray(v).ndim >= 2 else np.asarray(v))
                      for k, v in batch.items() if k != "seed"}
                rb = internal.device_put(rb, self.agent.train_sharded)
                rb["seed"] = self.agent._seeds(step + 1, self.agent.train_mirrored)
                _, metrics = self.agent.report(self.agent.init_report(self.cfg.batch_size), rb)
                for key, value in metrics.items():
                    if isinstance(value, str):
                        continue
                    value = np.asarray(value)
                    if value.ndim == 4:
                        self.logger.video(key, value)
                    elif value.ndim == 3:
                        self.logger.image(key, value)
                    else:
                        self.logger.scalar(f"report/{key}", float(value.mean()))
                self.logger.write()
            print(f"[DreamerV3] report written at step {step}")
        except Exception as e:  # noqa: BLE001
            print(f"[DreamerV3] report failed: {e}")


def _plot_trajectory(stats):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    traj = np.asarray(stats["trajectory"], np.float32)
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(traj[:, 0], traj[:, 1], "b-", label="path")
    ax.plot(traj[0, 0], traj[0, 1], "go", label="start")
    ax.plot(traj[-1, 0], traj[-1, 1], "ro", label=stats["outcome"])
    if stats.get("goal") is not None:
        ax.add_patch(plt.Circle(stats["goal"], stats.get("goal_radius", 1.0), color="r", fill=False, label="goal"))
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="png")
    plt.close(fig)
    buf.seek(0)
    return (plt.imread(buf)[..., :3] * 255).astype(np.uint8)   # (H, W, 3): elements.Logger.image takes one image
