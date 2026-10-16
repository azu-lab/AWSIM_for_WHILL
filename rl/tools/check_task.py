#!/usr/bin/env python3
"""
Check that a task can be solved, before training on it.

Runs the task with a scripted driver that uses ground truth (odometry, goal, and the obstacle
poses AWSIM publishes on /awsim/episode/obstacles) — information the agent does NOT get — and
steers around the obstacles to the goal within the velocity limits of the task. The episodes
run through WhillEnv, so reward, collision, success and the step limit are exactly those of
training.

    python3 rl/tools/check_task.py --task rl/configs/tasks/shinjuku_box.yaml --episodes 10

If the scripted driver does not reach the goal in most episodes, the task is probably not
learnable either (goal too far for the step limit, obstacle blocking the way, start in contact
with something, ...).

Output (default logs/rl/check_<task>/): per episode a plot (path, start, goal, obstacles) and the
first camera frame in full resolution and as the 64x64 observation; summary.csv.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "rl"))

from loop import plot_episode  # noqa: E402
from whill_env import load_task, make_env  # noqa: E402

CLEARANCE = 0.9      # [m] lateral distance kept from an obstacle center (box half-diagonal + half vehicle width + margin)
CORRIDOR = 0.9       # [m] obstacles closer than this to the straight line are avoided
LOOKAHEAD = 0.6      # [m] a waypoint is reached within this distance


def plan(start, goal, obstacles):
    """Waypoints from start to goal passing obstacles near the straight line on the side away from them."""
    sx, sy = start
    gx, gy = goal
    d = np.array([gx - sx, gy - sy])
    length = float(np.linalg.norm(d))
    if length < 1e-6:
        return [goal]
    u = d / length
    n = np.array([-u[1], u[0]])          # left normal
    points = []
    for ox, oy, _ in obstacles:
        rel = np.array([ox - sx, oy - sy])
        along, lateral = float(rel @ u), float(rel @ n)
        if 0.0 < along < length and abs(lateral) < CORRIDOR:
            side = -1.0 if lateral > 0 else 1.0          # pass on the other side
            offset = lateral + side * CLEARANCE
            for a in (along - 1.0, along + 0.6):
                p = np.array([sx, sy]) + u * a + n * offset
                points.append((a, (float(p[0]), float(p[1]))))
    points.sort()
    return [p for _, p in points] + [goal]


def drive(env, max_v, max_w, verbose):
    obs, info = env.reset()
    dm = env._dm   # ground truth for the scripted driver only
    s = dm.snapshot(consume_collision=False)
    frames = (s.image.copy() if s.image is not None else None, obs["image"].copy())
    goal = info["goal"]
    waypoints = plan((s.x, s.y), goal, s.obstacles)
    if verbose:
        print("  obstacles:", [tuple(round(v, 2) for v in o) for o in s.obstacles], " waypoints:",
              [tuple(round(v, 2) for v in p) for p in waypoints])
    total, wp = 0.0, 0
    while True:
        s = dm.snapshot(consume_collision=False)
        while wp < len(waypoints) - 1 and math.hypot(waypoints[wp][0] - s.x, waypoints[wp][1] - s.y) < LOOKAHEAD:
            wp += 1
        tx, ty = waypoints[wp]
        err = math.atan2(ty - s.y, tx - s.x) - s.yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi
        w = float(np.clip(2.0 * err, -max_w, max_w))
        v = max_v * max(0.0, math.cos(err)) if abs(err) < 1.2 else 0.0
        obs, r, term, trunc, info = env.step(np.array([v / max_v, w / max_w], np.float32))
        total += r
        if term or trunc:
            break
    stats = {"outcome": info["outcome"], "reward": total, "length": info["step"],
             "distance_to_goal": info["distance_to_goal"], "goal": goal, "goal_radius": env.task.goal_radius,
             "obstacles": [list(o) for o in info["obstacles"]], "trajectory": info["trajectory"]}
    return stats, frames, waypoints


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", default=str(REPO_ROOT / "rl" / "configs" / "tasks" / "shinjuku_box.yaml"))
    p.add_argument("--episodes", type=int, default=10)
    p.add_argument("--out", default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    task = load_task(args.task)
    out = Path(args.out or REPO_ROOT / "logs" / "rl" / f"check_{task.name}")
    out.mkdir(parents=True, exist_ok=True)
    env = make_env(task, seed=args.seed)
    a = task.action
    rows = []
    try:
        for ep in range(1, args.episodes + 1):
            stats, (full, small), waypoints = drive(env, a["max_linear_vel"], a["max_angular_vel"], args.verbose)
            plot_episode(stats, out / f"ep{ep:03d}.png", f"scripted driver, ep {ep}: {stats['outcome']}, {stats['length']} steps")
            if full is not None:
                cv2.imwrite(str(out / f"ep{ep:03d}_camera.jpg"), cv2.cvtColor(full, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(out / f"ep{ep:03d}_obs64.png"),
                        cv2.resize(cv2.cvtColor(small, cv2.COLOR_RGB2BGR), (256, 256), interpolation=cv2.INTER_NEAREST))
            obstacles = "; ".join(f"({x:.2f}, {y:.2f}, {math.degrees(t):.0f} deg)" for x, y, t in stats["obstacles"])
            rows.append({"episode": ep, "outcome": stats["outcome"], "steps": stats["length"],
                         "reward": round(stats["reward"], 2), "final_distance": round(stats["distance_to_goal"], 2),
                         "start": f"({stats['trajectory'][0][0]:.2f}, {stats['trajectory'][0][1]:.2f})",
                         "goal": f"({stats['goal'][0]:.2f}, {stats['goal'][1]:.2f})", "obstacles": obstacles})
            print(f"episode {ep:3d}: {stats['outcome']:9s} {stats['length']:4d} steps  reward {stats['reward']:7.2f}  "
                  f"start {rows[-1]['start']} goal {rows[-1]['goal']} obstacles {obstacles}")
    finally:
        env.close()
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    n = len(rows)
    count = lambda o: sum(r["outcome"] == o for r in rows)  # noqa: E731
    print(f"\n{task.name}: success {count('success')}/{n}, collision {count('collision')}/{n}, timeout {count('timeout')}/{n}; "
          f"steps of successes: {[r['steps'] for r in rows if r['outcome'] == 'success']} (limit {task.episode['max_steps']})")
    print(f"plots and camera frames: {out}")


if __name__ == "__main__":
    main()
