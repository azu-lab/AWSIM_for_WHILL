"""Episode loop shared by train.py and evaluate.py: connects a WhillEnv and an Agent."""

import json
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import numpy as np


def run_episode(env, agent, training: bool, on_step: Optional[Callable[[], None]] = None,
                verbose: bool = False) -> Dict[str, Any]:
    """One episode. on_step() is called after every environment step (checkpoints)."""
    obs, info = env.reset()
    agent.begin_episode(obs, training)
    total, length, start = 0.0, 0, time.time()
    terms_sum: Dict[str, float] = {}
    while True:
        action = agent.act(obs, training)
        next_obs, reward, terminated, truncated, info = env.step(action)
        agent.observe(action, reward, next_obs, terminated, truncated, info, training)
        obs = next_obs
        total += reward
        length += 1
        for k, v in info["reward_terms"].items():
            terms_sum[k] = terms_sum.get(k, 0.0) + v
        if verbose:
            v, w = info["command"]
            print(f"  step {length:4d}  a=[{action[0]:+.2f} {action[1]:+.2f}]  cmd=({v:.2f} m/s, {w:+.2f} rad/s)  "
                  f"r={reward:+7.2f}  d_goal={info['distance_to_goal']:.2f} m")
        if on_step is not None:
            on_step()
        if terminated or truncated:
            break
    stats = {
        "outcome": info["outcome"],
        "reward": float(total),
        "length": length,
        "seconds": time.time() - start,
        "distance_to_goal": float(info["distance_to_goal"]),
        "goal": None if info["goal"] is None else tuple(float(x) for x in info["goal"]),
        "goal_radius": env.task.goal_radius,
        "obstacles": [list(o) for o in info["obstacles"]],
        "start": None if env.episode.start is None else [env.episode.start.x, env.episode.start.y, env.episode.start.yaw],
        "reward_terms": terms_sum,
        "frame_timeouts": info["frame_timeouts"],
        "trajectory": info["trajectory"],
    }
    agent.end_episode(stats, training)
    return stats


def append_jsonl(path: Path, record: Dict[str, Any]):
    record = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in record.items()}
    with open(path, "a") as f:
        f.write(json.dumps(record) + "\n")


def plot_episode(stats: Dict[str, Any], path: Path, title: str = ""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    traj = np.asarray(stats["trajectory"])
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(traj[:, 0], traj[:, 1], "b-", label="path")
    ax.plot(traj[0, 0], traj[0, 1], "go", label="start")
    ax.plot(traj[-1, 0], traj[-1, 1], "ro", label=stats["outcome"])
    if stats["goal"] is not None:
        ax.add_patch(plt.Circle(stats["goal"], stats["goal_radius"], color="r", fill=False, label="goal"))
        ax.plot(*stats["goal"], "r+")
    for i, (ox, oy, oyaw) in enumerate(stats.get("obstacles") or []):
        ax.plot(ox, oy, "s", color="saddlebrown", markersize=8, label="obstacle" if i == 0 else None)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.02, 1.0))   # outside: do not hide the obstacles
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    ax.set_title(title or f"{stats['outcome']}, reward {stats['reward']:.1f}, {stats['length']} steps")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)
