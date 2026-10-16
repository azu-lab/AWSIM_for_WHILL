#!/usr/bin/env python3
"""
Run an agent without learning and report success / collision / timeout.

    # trained model: newest checkpoint of a run, 10 episodes
    python3 rl/evaluate.py --logdir logs/rl/shinjuku_box --episodes 10
    # several checkpoints
    python3 rl/evaluate.py --logdir logs/rl/shinjuku_box --checkpoints 000005000,000010000
    # no model: check a task / the environment with random or constant actions
    python3 rl/evaluate.py --agent random --task rl/configs/tasks/shinjuku_box.yaml --episodes 2 --verbose
    python3 rl/evaluate.py --agent random --set mode=constant --set constant=[1.0,0.0] --episodes 1

Output (default <logdir>/eval/ or logs/rl/eval_<agent>/): results.csv, episodes.jsonl (with the
trajectories and obstacle poses), one PNG per episode (trajectory, start, goal, obstacles),
summary on the terminal.
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import agents  # noqa: E402
from loop import append_jsonl, plot_episode, run_episode  # noqa: E402
from whill_env import load_task, make_env  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--logdir", default=None, help="run directory of train.py")
    p.add_argument("--checkpoints", default="latest", help="'latest', 'all' or comma-separated names in <logdir>/checkpoints")
    p.add_argument("--agent", default=None, help="agent without a run directory (e.g. random)")
    p.add_argument("--agent-config", default=None)
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    p.add_argument("--task", default=None, help="task YAML (default: task.yaml of --logdir, else shinjuku_box.yaml)")
    p.add_argument("--episodes", type=int, default=5)
    p.add_argument("--out", default=None, help="output directory")
    p.add_argument("--seed", type=int, default=1000, help="seed of the start/goal sampling")
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    if args.logdir:
        logdir = Path(args.logdir)
        run = yaml.safe_load((logdir / "run.yaml").read_text())
        agent_name, agent_cfg = run["agent"], run["agent_config"]
        task_path = args.task or str(logdir / "task.yaml")
        names = sorted(d.name for d in (logdir / "checkpoints").iterdir() if d.is_dir())
        if not names:
            sys.exit(f"no checkpoints in {logdir}/checkpoints")
        if args.checkpoints == "latest":
            names = names[-1:]
        elif args.checkpoints != "all":
            wanted = args.checkpoints.split(",")
            missing = [n for n in wanted if n not in names]
            if missing:
                sys.exit(f"checkpoints {missing} not found; available: {', '.join(names)}")
            names = wanted
        out = Path(args.out or logdir / "eval")
    else:
        if not args.agent:
            sys.exit("give --logdir (trained model) or --agent (e.g. random)")
        agent_name = args.agent
        agent_cfg = agents.agent_config(agent_name, args.agent_config, args.set)
        task_path = args.task or str(REPO_ROOT / "rl" / "configs" / "tasks" / "shinjuku_box.yaml")
        names = [None]
        out = Path(args.out or REPO_ROOT / "logs" / "rl" / f"eval_{agent_name.replace(':', '_')}")
    out.mkdir(parents=True, exist_ok=True)
    (out / "episodes.jsonl").unlink(missing_ok=True)

    task = load_task(task_path)
    env = make_env(task, seed=args.seed)
    agent = agents.create_agent(agent_name, env.observation_space, env.action_space, agent_cfg, out)
    print(f"agent {agent_name}, task {task.name} ({task_path}), {args.episodes} episode(s) per checkpoint -> {out}")

    rows = []
    try:
        for name in names:
            if name is not None:
                print(f"\n== checkpoint {name}")
                agent.load(logdir / "checkpoints" / name)
            for ep in range(1, args.episodes + 1):
                s = run_episode(env, agent, training=False, verbose=args.verbose)
                label = f"{name or agent_name}_ep{ep:03d}"
                plot_episode(s, out / f"{label}.png", f"{label}: {s['outcome']}, reward {s['reward']:.1f}")
                append_jsonl(out / "episodes.jsonl", {"checkpoint": name or "", "episode": ep, **s})
                rows.append({"checkpoint": name or "", "episode": ep, "outcome": s["outcome"],
                             "reward": round(s["reward"], 3), "length": s["length"],
                             "seconds": round(s["seconds"], 2), "final_distance_to_goal": round(s["distance_to_goal"], 3),
                             "frame_timeouts": s["frame_timeouts"]})
                print(f"  episode {ep}: {s['outcome']:9s} reward {s['reward']:8.2f} length {s['length']:4d} "
                      f"final distance {s['distance_to_goal']:.2f} m")
    finally:
        agent.close()
        env.close()

    with open(out / "results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("\ncheckpoint      episodes  success  collision  timeout  mean reward  mean length")
    for name in names:
        r = [x for x in rows if x["checkpoint"] == (name or "")]
        rate = lambda o: sum(x["outcome"] == o for x in r) / len(r)  # noqa: E731
        print(f"{(name or agent_name):15s} {len(r):8d}  {rate('success'):7.0%}  {rate('collision'):9.0%}  "
              f"{rate('timeout'):7.0%}  {np.mean([x['reward'] for x in r]):11.2f}  {np.mean([x['length'] for x in r]):11.1f}")
    print(f"results: {out / 'results.csv'}")


if __name__ == "__main__":
    main()
