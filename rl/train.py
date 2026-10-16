#!/usr/bin/env python3
"""
Train an agent (rl/agents/) on a task (rl/configs/tasks/) in AWSIM.

    python3 rl/train.py --agent dreamerv3 --task rl/configs/tasks/shinjuku_box.yaml \
        --logdir logs/rl/shinjuku_box --steps 200000

Run it again with the same --logdir to resume from the newest checkpoint.

Run directory:
    run.yaml            agent name, agent configuration, command line
    task.yaml           copy of the task file (used on resume and by evaluate.py)
    episodes.jsonl      one line per episode: outcome, reward, length, reward terms, start, goal, trajectory
    progress.json       steps / episodes done, newest checkpoint
    checkpoints/<step>/ written by the agent (Agent.save)
    ...                 files of the agent (DreamerV3: metrics.jsonl, TensorBoard events, replay/)
"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import agents  # noqa: E402
from loop import append_jsonl, run_episode  # noqa: E402
from whill_env import load_task, make_env  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--agent", default=None, help=f"one of {', '.join(agents.AGENTS)} or package.module:Class "
                   "(default: dreamerv3, or the agent of an existing --logdir)")
    p.add_argument("--task", default=None, help="task YAML (default: rl/configs/tasks/shinjuku_box.yaml, "
                   "or task.yaml of an existing --logdir)")
    p.add_argument("--logdir", required=True)
    p.add_argument("--steps", type=int, default=1_000_000, help="total environment steps")
    p.add_argument("--save-every", type=int, default=5000, help="checkpoint interval [steps]")
    p.add_argument("--keep-checkpoints", type=int, default=0, help="keep only the newest N checkpoints (0: all)")
    p.add_argument("--agent-config", default=None, help="YAML merged into the default config of the agent")
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="override one agent setting, e.g. --set batch_size=8 (repeatable)")
    p.add_argument("--seed", type=int, default=0, help="seed of the start/goal sampling")
    p.add_argument("--verbose", action="store_true", help="print every step")
    args = p.parse_args()

    logdir = Path(args.logdir)
    run_file, progress_file = logdir / "run.yaml", logdir / "progress.json"
    resume = run_file.exists()
    if resume:
        run = yaml.safe_load(run_file.read_text())
        if args.agent and args.agent != run["agent"]:
            sys.exit(f"{logdir} was trained with --agent {run['agent']}, not {args.agent}")
        agent_name, agent_cfg = run["agent"], run["agent_config"]
        if args.agent_config or args.set:
            print("Note: --agent-config/--set are ignored when resuming (configuration of run.yaml)")
        task_path = args.task or str(logdir / "task.yaml")
    else:
        agent_name = args.agent or "dreamerv3"
        agent_cfg = agents.agent_config(agent_name, args.agent_config, args.set)
        task_path = args.task or str(Path(__file__).resolve().parent / "configs" / "tasks" / "shinjuku_box.yaml")
    task = load_task(task_path)

    logdir.mkdir(parents=True, exist_ok=True)
    if not resume:
        run_file.write_text(yaml.safe_dump({"agent": agent_name, "agent_config": agent_cfg,
                                            "command": " ".join(sys.argv)}, sort_keys=False))
    if Path(task_path).resolve() != (logdir / "task.yaml").resolve():
        shutil.copyfile(task_path, logdir / "task.yaml")

    print("=" * 70)
    print(f"agent : {agent_name}\ntask  : {task.name} ({task_path})\nlogdir: {logdir}"
          f"{'  (resuming)' if resume else ''}\nsteps : {args.steps}")
    print("=" * 70)

    env = make_env(task, seed=args.seed)
    agent = agents.create_agent(agent_name, env.observation_space, env.action_space, agent_cfg, logdir)

    progress = {"steps": 0, "episodes": 0, "checkpoint": None}
    if resume and progress_file.exists():
        progress = json.loads(progress_file.read_text())
        if progress.get("checkpoint"):
            print(f"Loading checkpoint {progress['checkpoint']}")
            agent.load(logdir / "checkpoints" / progress["checkpoint"])

    def save_checkpoint():
        name = f"{progress['steps']:09d}"
        directory = logdir / "checkpoints" / name
        directory.mkdir(parents=True, exist_ok=True)
        agent.save(directory)
        progress["checkpoint"] = name
        progress_file.write_text(json.dumps(progress, indent=2))
        print(f"Checkpoint {directory}")
        if args.keep_checkpoints > 0:
            old = sorted((logdir / "checkpoints").iterdir())[:-args.keep_checkpoints]
            for d in old:
                shutil.rmtree(d)

    def on_step():
        progress["steps"] += 1
        if progress["steps"] % args.save_every == 0:
            save_checkpoint()

    try:
        while progress["steps"] < args.steps:
            t0 = time.time()
            stats = run_episode(env, agent, training=True, on_step=on_step, verbose=args.verbose)
            progress["episodes"] += 1
            append_jsonl(logdir / "episodes.jsonl", {"episode": progress["episodes"], "step": progress["steps"], **stats})
            print(f"episode {progress['episodes']:5d}  step {progress['steps']:8d}  {stats['outcome']:9s}  "
                  f"reward {stats['reward']:8.2f}  length {stats['length']:4d}  ({time.time() - t0:.1f} s)")
        if progress["checkpoint"] != f"{progress['steps']:09d}":
            save_checkpoint()
    except KeyboardInterrupt:
        print("Interrupted; the newest checkpoint is", progress["checkpoint"])
    finally:
        agent.close()
        env.close()


if __name__ == "__main__":
    main()
