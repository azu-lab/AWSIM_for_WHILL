#!/usr/bin/env python3
"""
Batch Evaluation Script for AWSIM Wheelchair with DreamerV3

Evaluates multiple checkpoints and generates:
- Console output (summary table)
- CSV file with detailed results
"""

import os
import sys
import argparse
import csv
import time
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np

# Add paths
# rl/ (awsim_dreamerv3) と third_party/dreamerv3 を import できるようにする
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'rl'))
sys.path.insert(0, str(REPO_ROOT / 'third_party' / 'dreamerv3'))

import awsim_dreamerv3.config as awsim_config
from awsim_dreamerv3.env import AWSIMWheelchairEnv
import embodied
from awsim_dreamerv3.embodied_adapter import FromGymnasium


@dataclass
class EpisodeResult:
    """Result of a single evaluation episode."""
    success: bool
    collision: bool
    timeout: bool
    steps: int
    time_seconds: float
    total_reward: float


@dataclass
class CheckpointResult:
    """Aggregated results for a single checkpoint."""
    step: int
    episodes: List[EpisodeResult] = field(default_factory=list)
    
    @property
    def success_rate(self) -> float:
        if not self.episodes:
            return 0.0
        return sum(1 for e in self.episodes if e.success) / len(self.episodes)
    
    @property
    def collision_rate(self) -> float:
        if not self.episodes:
            return 0.0
        return sum(1 for e in self.episodes if e.collision) / len(self.episodes)
    
    @property
    def timeout_rate(self) -> float:
        if not self.episodes:
            return 0.0
        return sum(1 for e in self.episodes if e.timeout) / len(self.episodes)
    
    @property
    def avg_time(self) -> float:
        if not self.episodes:
            return 0.0
        return np.mean([e.time_seconds for e in self.episodes])
    
    @property
    def std_time(self) -> float:
        if len(self.episodes) < 2:
            return 0.0
        return np.std([e.time_seconds for e in self.episodes], ddof=1)
    
    @property
    def avg_reward(self) -> float:
        if not self.episodes:
            return 0.0
        return np.mean([e.total_reward for e in self.episodes])
    
    @property
    def std_reward(self) -> float:
        if len(self.episodes) < 2:
            return 0.0
        return np.std([e.total_reward for e in self.episodes], ddof=1)
    
    @property
    def avg_steps(self) -> float:
        if not self.episodes:
            return 0.0
        return np.mean([e.steps for e in self.episodes])


def evaluate_checkpoint(
    agent,
    env,
    checkpoint_path: Path,
    num_episodes: int,
    step_duration: float
) -> CheckpointResult:
    """
    Evaluate a single checkpoint.
    """
    import pickle
    
    # Extract step number from checkpoint filename
    step = int(checkpoint_path.stem.split('_')[-1]) if '_' in checkpoint_path.stem else 0
    result = CheckpointResult(step=step)
    
    # Load checkpoint
    print(f"\nLoading checkpoint: {checkpoint_path.name}")
    with open(str(checkpoint_path), 'rb') as f:
        data = pickle.load(f)
        agent.load(data)
    
    for ep in range(num_episodes):
        print(f"  Episode {ep + 1}/{num_episodes}", end='', flush=True)
        
        # Reset
        act = {k: v.sample() for k, v in env.act_space.items()}
        act['reset'] = True
        obs = env.step(act)
        
        state = agent.init_policy(batch_size=1)
        done = False
        step_count = 0
        total_reward = 0.0
        start_time = time.time()
        
        success = False
        collision = False
        
        while not done:
            # Prepare obs batch - include all keys (including is_collision, is_success)
            obs_batch = {
                k: np.asarray(v)[None] for k, v in obs.items()
            }
            
            # Policy (eval mode = deterministic)
            state, action, _ = agent.policy(state, obs_batch, mode='eval')
            
            # Remove batch dim
            action = {k: v[0] for k, v in action.items()}
            action['reset'] = False
            
            # Step
            next_obs = env.step(action)
            reward = next_obs['reward']
            done = next_obs['is_last'] or next_obs['is_terminal']
            
            # Check outcome
            if done:
                if next_obs.get('is_success', False):
                    success = True
                elif next_obs.get('is_collision', False):
                    collision = True
            
            obs = next_obs
            total_reward += reward
            step_count += 1
        
        elapsed_time = time.time() - start_time
        timeout = not success and not collision
        
        episode_result = EpisodeResult(
            success=success,
            collision=collision,
            timeout=timeout,
            steps=step_count,
            time_seconds=elapsed_time,
            total_reward=total_reward
        )
        result.episodes.append(episode_result)
        
        outcome = "SUCCESS" if success else ("COLLISION" if collision else "TIMEOUT")
        print(f" -> {outcome} (steps: {step_count}, reward: {total_reward:.1f})")
    
    return result


def print_summary_table(results: List[CheckpointResult]):
    """Print summary table to console."""
    print("\n" + "=" * 90)
    print("EVALUATION SUMMARY")
    print("=" * 90)
    
    # Header
    header = f"{'Step':>10} | {'Success':>8} | {'Collision':>10} | {'Timeout':>8} | {'Avg Time (s)':>16} | {'Avg Reward':>18}"
    print(header)
    print("-" * 90)
    
    # Rows (x±SD format)
    for r in results:
        time_str = f"{r.avg_time:.2f}±{r.std_time:.2f}"
        reward_str = f"{r.avg_reward:.2f}±{r.std_reward:.2f}"
        row = f"{r.step:>10} | {r.success_rate*100:>7.1f}% | {r.collision_rate*100:>9.1f}% | {r.timeout_rate*100:>7.1f}% | {time_str:>16} | {reward_str:>18}"
        print(row)
    
    print("=" * 90)


def save_csv(results: List[CheckpointResult], output_path: Path):
    """Save results to CSV file."""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Header
        writer.writerow([
            'checkpoint_step',
            'success_rate',
            'collision_rate',
            'timeout_rate',
            'avg_time_seconds',
            'avg_steps',
            'avg_reward',
            'num_episodes'
        ])
        
        # Data rows
        for r in results:
            writer.writerow([
                r.step,
                f"{r.success_rate:.4f}",
                f"{r.collision_rate:.4f}",
                f"{r.timeout_rate:.4f}",
                f"{r.avg_time:.2f}",
                f"{r.avg_steps:.1f}",
                f"{r.avg_reward:.2f}",
                len(r.episodes)
            ])
    
    print(f"CSV saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Batch evaluation of DreamerV3 checkpoints')
    parser.add_argument('--logdir', type=Path, 
                        default=str(REPO_ROOT / 'logs' / 'dreamer' / 'awsim_wheelchair_v3'),
                        help='Directory containing training logs and checkpoints')
    parser.add_argument('--checkpoints', type=int, nargs='+',
                        help='List of checkpoint steps to evaluate (e.g., 10000 20000 30000)')
    parser.add_argument('--step-interval', type=int,
                        help='Alternative: auto-detect checkpoints at this interval')
    parser.add_argument('--episodes', type=int, default=5,
                        help='Number of episodes per checkpoint')
    parser.add_argument('--output', type=str, default='eval_results',
                        help='Base name for output files (CSV)')
    parser.add_argument('--task', type=str, default='original',
                        choices=['original', 'obstacle_avoidance'],
                        help='タスク選択: original=既存の障害物回避, obstacle_avoidance=ダンボール回避前進')
    parser.add_argument('--env-config', type=str, default=None,
                        help='Environment YAML (default: <logdir>/env_config.yaml written by train.py, '
                             'else rl/configs/env/<task>.yaml)')
    args = parser.parse_args()

    # Use the same environment settings as the training run
    from awsim_dreamerv3.env_config import apply_env_config
    if args.env_config is None:
        saved = Path(args.logdir) / 'env_config.yaml'
        args.env_config = str(saved if saved.exists() else REPO_ROOT / 'rl' / 'configs' / 'env' / f'{args.task}.yaml')
    env_settings = apply_env_config(args.env_config)
    if env_settings['task'] != args.task:
        print(f"Task '{env_settings['task']}' from {args.env_config} overrides --task {args.task}")
        args.task = env_settings['task']
    print(f"Env config: {args.env_config}")

    logdir = args.logdir
    print("=" * 60)
    print("DreamerV3 Batch Evaluation")
    print("=" * 60)
    print(f"Log directory: {logdir}")

    # Find checkpoints
    checkpoint_dir = logdir / 'agent'
    if not checkpoint_dir.exists():
        print(f"Error: Checkpoint directory not found: {checkpoint_dir}")
        return
    
    # Determine which checkpoints to evaluate
    if args.checkpoints:
        checkpoint_steps = args.checkpoints
    elif args.step_interval:
        all_checkpoints = list(checkpoint_dir.glob('checkpoint_*.pkl'))
        checkpoint_steps = []
        for cp in all_checkpoints:
            try:
                step = int(cp.stem.split('_')[-1])
                if step % args.step_interval == 0:
                    checkpoint_steps.append(step)
            except ValueError:
                continue
        checkpoint_steps = sorted(checkpoint_steps)
    else:
        all_checkpoints = list(checkpoint_dir.glob('checkpoint_*.pkl'))
        checkpoint_steps = []
        for cp in all_checkpoints:
            try:
                step = int(cp.stem.split('_')[-1])
                checkpoint_steps.append(step)
            except ValueError:
                continue
        checkpoint_steps = sorted(checkpoint_steps)
    
    if not checkpoint_steps:
        print("Error: No checkpoints found to evaluate.")
        return
    
    print(f"Checkpoints to evaluate: {checkpoint_steps}")
    print(f"Episodes per checkpoint: {args.episodes}")

    # Import DreamerV3 components
    import elements
    import ruamel.yaml as yaml

    # Load config from logdir (THIS IS CRITICAL - use the training config)
    config_path = logdir / 'config.yaml'
    if config_path.exists():
        print(f"Loading config from {config_path}")
        config = elements.Config.load(config_path)
    else:
        print("Config not found in logdir, loading defaults...")
        dreamerv3_folder = REPO_ROOT / 'third_party' / 'dreamerv3' / 'dreamerv3'
        defaults_config = yaml.YAML(typ='safe').load(
            (dreamerv3_folder / 'configs.yaml').read_text()
        )
        config = elements.Config(defaults_config['defaults'])
        config = config.update({
            'logdir': logdir,
            'batch_size': 1,
            'batch_length': 64,
        })

    # Initialize environment
    print(f"\nInitializing environment (task: {args.task})...")
    
    if args.task == 'obstacle_avoidance':
        from awsim_dreamerv3.obstacle_avoidance_env import ObstacleAvoidanceEnv
        import awsim_dreamerv3.obstacle_avoidance_config as oa_config
        gym_env = ObstacleAvoidanceEnv(
            render_mode="rgb_array",
            image_size=oa_config.IMAGE_SIZE,
            max_linear_vel=oa_config.MAX_LINEAR_VEL,
            max_angular_vel=oa_config.MAX_ANGULAR_VEL,
            episode_timeout=oa_config.EPISODE_TIMEOUT,
            step_duration=oa_config.STEP_DURATION,
        )
        print(f"[Task] obstacle_avoidance: vel={oa_config.MAX_LINEAR_VEL}m/s, "
              f"ang={oa_config.MAX_ANGULAR_VEL}rad/s, "
              f"goal=Unity X -3m (ROS Y +{oa_config.GOAL_DISTANCE_ROS_Y}m)")
    else:
        gym_env = AWSIMWheelchairEnv(
            render_mode="rgb_array",
            image_size=awsim_config.IMAGE_SIZE,
            max_linear_vel=awsim_config.MAX_LINEAR_VEL,
            max_angular_vel=awsim_config.MAX_ANGULAR_VEL,
            goal_distance=awsim_config.GOAL_DISTANCE,
            episode_timeout=awsim_config.EPISODE_TIMEOUT,
            step_duration=awsim_config.STEP_DURATION,
        )
        print(f"[Task] original: vel=1.0m/s, goal={awsim_config.GOAL_DISTANCE}m (ROS X)")
    env = FromGymnasium(gym_env, obs_key='image', act_key='action')
    
    # IMPORTANT: obs_space must match the CHECKPOINT structure.
    # 
    # The checkpoints in awsim_wheelchair_v3 were trained WITH is_collision and 
    # is_success in the observation space. Therefore, we need to include these
    # in obs_space to match the checkpoint structure.
    #
    # Note: DreamerV3 will:
    # - Encode is_collision/is_success with MLP (they have shape <= 2)
    # - Create decoder heads for them
    # This matches the checkpoint which has enc/mlp0-2 and dec/vec/is_collision, etc.
    obs_space = {
        'image': elements.Space(np.uint8, (64, 64, 3), 0, 255),
        'is_first': elements.Space(bool, ()),
        'is_last': elements.Space(bool, ()),
        'is_terminal': elements.Space(bool, ()),
        'reward': elements.Space(np.float32, ()),
        'is_collision': elements.Space(np.float32, ()),
        'is_success': elements.Space(np.float32, ()),
    }
    
    act_space = {k: v for k, v in env.act_space.items() if k != 'reset'}
    
    print(f"Observation space keys: {list(obs_space.keys())}")
    print(f"Action space keys: {list(act_space.keys())}")

    # Create agent
    from dreamerv3.agent import Agent
    
    print("Creating DreamerV3 agent...")
    
    from awsim_dreamerv3.jax_compat import adjust_jax_config
    agent_config = elements.Config({
        **config.agent,
        'logdir': config.logdir,
        'seed': config.seed,
        'jax': adjust_jax_config(config.jax, None),
        'batch_size': 1,
        'batch_length': config.batch_length,
        'replay_context': config.replay_context,
        'report_length': config.report_length,
        'replica': 0,
        'replicas': 1,
    })
    agent = Agent(obs_space, act_space, agent_config)

    # Evaluate each checkpoint
    print("\n" + "=" * 60)
    print("Starting evaluation...")
    print("=" * 60)
    
    all_results = []
    for step in checkpoint_steps:
        checkpoint_path = checkpoint_dir / f'checkpoint_{step}.pkl'
        if not checkpoint_path.exists():
            print(f"Warning: Checkpoint not found: {checkpoint_path}, skipping...")
            continue
        
        result = evaluate_checkpoint(
            agent=agent,
            env=env,
            checkpoint_path=checkpoint_path,
            num_episodes=args.episodes,
            step_duration=awsim_config.STEP_DURATION
        )
        all_results.append(result)
    
    env.close()

    if not all_results:
        print("Error: No checkpoints were successfully evaluated.")
        return

    # Output results
    print_summary_table(all_results)
    
    # Save CSV
    csv_path = Path(args.output + '_results.csv')
    save_csv(all_results, csv_path)
    
    print("\n" + "=" * 60)
    print("Evaluation complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
