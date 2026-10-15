#!/usr/bin/env python3
"""
Run the AWSIM gym environment with random (or constant) actions, without DreamerV3.

Checks the whole loop that training relies on: reset, observation, action, reward,
termination (goal / collision / timeout). Prints one line per step and a summary per episode.

    python3 rl/scripts/random_agent.py --episodes 2
    python3 rl/scripts/random_agent.py --episodes 1 --constant 1.0 0.0   # drive straight on
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'rl'))

from awsim_dreamerv3.env_config import apply_env_config  # noqa: E402


def make_env(task):
    if task == "obstacle_avoidance":
        from awsim_dreamerv3.obstacle_avoidance_env import ObstacleAvoidanceEnv
        import awsim_dreamerv3.obstacle_avoidance_config as c
        return ObstacleAvoidanceEnv(render_mode="rgb_array", max_linear_vel=c.MAX_LINEAR_VEL,
                                    max_angular_vel=c.MAX_ANGULAR_VEL, episode_timeout=c.EPISODE_TIMEOUT)
    from awsim_dreamerv3.env import AWSIMWheelchairEnv
    import awsim_dreamerv3.config as c
    return AWSIMWheelchairEnv(render_mode="rgb_array", max_linear_vel=c.MAX_LINEAR_VEL,
                              max_angular_vel=c.MAX_ANGULAR_VEL, goal_distance=c.GOAL_DISTANCE,
                              episode_timeout=c.EPISODE_TIMEOUT)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--env-config", default=str(REPO_ROOT / "rl" / "configs" / "env" / "obstacle_avoidance.yaml"))
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=1000, help="Safety limit per episode")
    parser.add_argument("--constant", type=float, nargs=2, default=None, metavar=("A0", "A1"),
                        help="Use this action in [-1, 1] instead of random actions")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--save-image", default=None, help="Save the first observation image (PNG)")
    args = parser.parse_args()

    settings = apply_env_config(args.env_config)
    env = make_env(settings["task"])
    rng = np.random.default_rng(args.seed)
    print(f"task={settings['task']} observation_space={env.observation_space} action_space={env.action_space}")

    summaries = []
    for episode in range(args.episodes):
        t0 = time.time()
        obs, info = env.reset(seed=args.seed + episode)
        img = obs["image"]
        print(f"[ep {episode}] reset: image {img.shape} {img.dtype} mean={img.mean():.1f} "
              f"goal={env._get_goal_center() if hasattr(env, '_get_goal_center') else '-'}")
        if args.save_image and episode == 0:
            import cv2
            cv2.imwrite(args.save_image, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        total, steps, terminated, truncated = 0.0, 0, False, False
        while not (terminated or truncated) and steps < args.max_steps:
            action = (np.array(args.constant, dtype=np.float32) if args.constant
                      else rng.uniform(-1.0, 1.0, size=2).astype(np.float32))
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            steps += 1
            if steps % 10 == 0 or terminated or truncated:
                print(f"[ep {episode}] step {steps:4d} a=({action[0]:+.2f},{action[1]:+.2f}) r={reward:+8.3f} "
                      f"sum={total:+9.2f} d_goal={info.get('distance_to_goal', float('nan')):.2f} "
                      f"v={info.get('forward_velocity', 0.0):+.2f} collision={info.get('is_collision')} "
                      f"success={info.get('is_success')}")
        outcome = ("success" if info.get("is_success") else "collision" if info.get("is_collision")
                   else "timeout" if truncated else "step_limit")
        summaries.append((episode, steps, total, outcome, time.time() - t0))
        print(f"[ep {episode}] END outcome={outcome} steps={steps} return={total:.2f} "
              f"wall={time.time() - t0:.1f}s trajectory_len={len(info.get('trajectory', []))}")

    env.close()
    print("\nepisode  steps   return  outcome    wall[s]")
    for e, n, r, o, w in summaries:
        print(f"{e:7d}  {n:5d}  {r:7.2f}  {o:<9}  {w:7.1f}")


if __name__ == "__main__":
    main()
