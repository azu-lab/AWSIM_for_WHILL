#!/usr/bin/env python3
"""
Training Script for AWSIM Wheelchair with DreamerV3

This script integrates the AWSIM Gym environment with DreamerV3
for training an obstacle avoidance policy.
"""

import os
import sys
import argparse
from pathlib import Path
from functools import partial as bind

# Add paths
# rl/ (awsim_dreamerv3) と third_party/dreamerv3 を import できるようにする
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'rl'))
sys.path.insert(0, str(REPO_ROOT / 'third_party' / 'dreamerv3'))

import numpy as np
import matplotlib.pyplot as plt
import io
import awsim_dreamerv3.config as awsim_config


def plot_trajectory(trajectory):
    """Generate trajectory plot image."""
    fig, ax = plt.subplots(figsize=(6, 6))
    if len(trajectory) > 0:
        traj = np.array(trajectory)
        ax.plot(traj[:, 0], traj[:, 1], 'b-', label='Path')
        ax.plot(traj[0, 0], traj[0, 1], 'go', label='Start')
        ax.plot(traj[-1, 0], traj[-1, 1], 'ro', label='End')
        
        # Plot goal (approximate)
        # Goal is start + 4m in Z.
        start_pos = traj[0]
        goal_z = start_pos[1] + 4.0
        goal_x = start_pos[0]
        circle = plt.Circle((goal_x, goal_z), 0.5, color='r', fill=False, label='Goal')
        ax.add_patch(circle)
        
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_title('Episode Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    
    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    plt.close(fig)
    buf.seek(0)
    
    # Convert buffer to numpy array for logger
    # Use plt.imread (returns 0-1 float)
    image = plt.imread(buf)
    # Convert to 0-255 uint8
    image = (image * 255).astype(np.uint8)
    # Discard alpha if present
    if image.shape[2] == 4:
        image = image[:, :, :3]
    # Add batch dim
    image = image[None] 
    return image


def make_awsim_env(env_id=0, task="original", enable_dr=False):
    """Create AWSIM environment based on task selection.
    
    Args:
        env_id: Environment ID (for multi-env setups)
        task: Task selection - "original" or "obstacle_avoidance"
        enable_dr: Whether to enable domain randomization
    """
    import embodied
    from awsim_dreamerv3.embodied_adapter import FromGymnasium
    
    if task == "obstacle_avoidance":
        from awsim_dreamerv3.obstacle_avoidance_env import ObstacleAvoidanceEnv
        import awsim_dreamerv3.obstacle_avoidance_config as oa_config
        gym_env = ObstacleAvoidanceEnv(
            render_mode="rgb_array",
            image_size=oa_config.IMAGE_SIZE,
            max_linear_vel=oa_config.MAX_LINEAR_VEL,
            max_angular_vel=oa_config.MAX_ANGULAR_VEL,
            episode_timeout=oa_config.EPISODE_TIMEOUT,
            step_duration=oa_config.STEP_DURATION,
            enable_domain_randomization=enable_dr,
        )
        print(f"[Task] obstacle_avoidance: vel={oa_config.MAX_LINEAR_VEL}m/s, "
              f"ang={oa_config.MAX_ANGULAR_VEL}rad/s, "
              f"goal=Unity X -3m (ROS Y +{oa_config.GOAL_DISTANCE_ROS_Y}m)")
    else:
        from awsim_dreamerv3.env import AWSIMWheelchairEnv
        gym_env = AWSIMWheelchairEnv(
            render_mode="rgb_array",
            image_size=awsim_config.IMAGE_SIZE,
            max_linear_vel=awsim_config.MAX_LINEAR_VEL,
            max_angular_vel=awsim_config.MAX_ANGULAR_VEL,
            goal_distance=awsim_config.GOAL_DISTANCE,
            episode_timeout=awsim_config.EPISODE_TIMEOUT,
            step_duration=awsim_config.STEP_DURATION,
            enable_domain_randomization=enable_dr,
        )
        print(f"[Task] original: vel={awsim_config.MAX_LINEAR_VEL}m/s, "
              f"ang={awsim_config.MAX_ANGULAR_VEL}rad/s, goal={awsim_config.GOAL_DISTANCE}m (ROS X)")
    
    # Wrap the gymnasium env as an embodied.Env (see awsim_dreamerv3/embodied_adapter.py)
    env = FromGymnasium(gym_env, obs_key='image', act_key='action')
    
    return env


def check_reward_convergence(reward_history, window_size, threshold):
    """
    Check if reward has converged based on moving average improvement.
    
    Args:
        reward_history: List of episode rewards
        window_size: Number of episodes for moving average
        threshold: Improvement rate threshold (e.g., 0.01 = 1%)
    
    Returns:
        (converged: bool, improvement_rate: float, current_avg: float, prev_avg: float)
    """
    if len(reward_history) < window_size * 2:
        return False, float('inf'), 0.0, 0.0
    
    # Calculate current and previous window averages
    current_avg = np.mean(reward_history[-window_size:])
    prev_avg = np.mean(reward_history[-window_size*2:-window_size])
    
    # Avoid division by zero
    if abs(prev_avg) < 1e-8:
        return False, float('inf'), current_avg, prev_avg
    
    # Calculate improvement rate
    improvement_rate = (current_avg - prev_avg) / abs(prev_avg)
    
    # Check if improvement is below threshold
    converged = improvement_rate < threshold
    
    return converged, improvement_rate, current_avg, prev_avg


def main():
    parser = argparse.ArgumentParser(description="Train DreamerV3 on AWSIM Wheelchair")
    parser.add_argument("--logdir", type=str, 
                        default=str(REPO_ROOT / 'logs' / 'dreamer' / 'awsim_wheelchair_v3'),
                        help="Directory for logs and checkpoints")
    parser.add_argument("--steps", type=int, default=1000000,
                        help="Total training steps (max steps if early-stop enabled)")
    parser.add_argument("--seed", type=int, default=0,
                        help="Random seed")
    parser.add_argument("--batch-size", type=int, default=16,
                        help="Batch size")
    parser.add_argument("--replay-size", type=int, default=1000000,
                        help="Replay buffer size")
    parser.add_argument("--config", type=str, default=None,
                        help="DreamerV3 config YAML (default: rl/configs/awsim_obstacle_avoidance.yaml "
                             "for obstacle_avoidance, rl/configs/awsim_wheelchair.yaml for original)")
    parser.add_argument("--env-config", type=str, default=None,
                        help="Environment YAML: action limits, reward, topics "
                             "(default: rl/configs/env/<task>.yaml)")
    parser.add_argument("--save-every", type=int, default=None,
                        help="Checkpoint interval in env steps (overrides run.save_every of --config)")
    parser.add_argument("--log-every", type=int, default=None,
                        help="Scalar log interval in env steps (overrides run.log_every)")
    parser.add_argument("--report-every", type=int, default=None,
                        help="Image report interval in env steps (overrides run.report_every)")
    parser.add_argument("--compute-dtype", type=str, default=None,
                        choices=["float16", "bfloat16", "float32"],
                        help="Override jax.compute_dtype (default: from --config; replaced by float32 "
                             "on GPUs with compute capability >= 12, see awsim_dreamerv3/jax_compat.py)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print debug output on every step")
    parser.add_argument("--task", type=str, default="original",
                        choices=["original", "obstacle_avoidance"],
                        help="タスク選択: original=既存の障害物回避, obstacle_avoidance=ダンボール回避前進")
    parser.add_argument("--enable-dr", action="store_true",
                        help="カメラ画像のDomain Randomization を有効化")
    
    # Early stopping options
    parser.add_argument("--early-stop", action="store_true",
                        help="Enable early stopping based on reward convergence")
    parser.add_argument("--min-steps", type=int, default=10000,
                        help="Minimum steps before early stopping can trigger (default: 10000)")
    parser.add_argument("--window-size", type=int, default=50,
                        help="Moving average window size in episodes (default: 50)")
    parser.add_argument("--improvement-threshold", type=float, default=0.01,
                        help="Stop if improvement rate falls below this (default: 0.01 = 1%%)")
    parser.add_argument("--patience", type=int, default=10,
                        help="Episodes to wait after threshold before stopping (default: 10)")
    
    args = parser.parse_args()

    # Environment settings (action limits, reward, topics) from YAML
    from awsim_dreamerv3.env_config import apply_env_config
    if args.env_config is None:
        args.env_config = str(REPO_ROOT / 'rl' / 'configs' / 'env' / f'{args.task}.yaml')
    env_settings = apply_env_config(args.env_config)
    if env_settings['task'] != args.task:
        print(f"Task '{env_settings['task']}' from {args.env_config} overrides --task {args.task}")
        args.task = env_settings['task']
    if args.config is None:
        name = 'awsim_obstacle_avoidance.yaml' if args.task == 'obstacle_avoidance' else 'awsim_wheelchair.yaml'
        args.config = str(REPO_ROOT / 'rl' / 'configs' / name)
    
    # Import DreamerV3 components
    import embodied
    import elements
    import ruamel.yaml as yaml
    
    print("=" * 60)
    print("DreamerV3 Training on AWSIM Wheelchair")
    print("=" * 60)
    print(f"Task: {args.task}")
    print(f"Log directory: {args.logdir}")
    print(f"Training steps: {args.steps}")
    print(f"Config file: {args.config}")
    print(f"Env config: {args.env_config}")
    print(f"Domain Randomization: {'ENABLED' if args.enable_dr else 'disabled'}")
    
    # Load default config
    dreamerv3_folder = REPO_ROOT / 'third_party' / 'dreamerv3' / 'dreamerv3'
    defaults_config = yaml.YAML(typ='safe').load(
        (dreamerv3_folder / 'configs.yaml').read_text()
    )
    
    # Helper for recursive update
    def recursive_update(base, update):
        for k, v in update.items():
            if isinstance(v, dict) and k in base and isinstance(base[k], dict):
                recursive_update(base[k], v)
            else:
                base[k] = v
    
    # Load task config from YAML
    # We merge configs at dict level BEFORE creating elements.Config
    # This avoids KeyError when adding new keys (like env.awsim or agent.enc.mlp_keys)
    defaults = defaults_config['defaults']
    
    config_path = elements.Path(args.config)
    if config_path.exists():
        print(f"Loading task config from {config_path}")
        with config_path.open() as f:
            task_config = yaml.YAML(typ='safe').load(f)
        
            task_config = task_config['defaults']
            
        # Filter out empty lists which elements.Config forbids
        def remove_empty_lists(d):
            keys_to_remove = []
            for k, v in d.items():
                if isinstance(v, list) and len(v) == 0:
                    keys_to_remove.append(k)
                elif isinstance(v, dict):
                    remove_empty_lists(v)
            for k in keys_to_remove:
                del d[k]
                
        remove_empty_lists(task_config)
            
        print("Merging task config into defaults...")
        recursive_update(defaults, task_config)
    else:
        print(f"Warning: Config file {config_path} not found. Using defaults.")
    
    config = elements.Config(defaults)

    # Apply command line overrides (Args > YAML > Defaults)
    # Ensure nested keys are updated correctly without wiping dicts
    # elements.Config.update is recursive
    overrides = {
        'logdir': args.logdir,
        'seed': args.seed,
        'batch_size': args.batch_size,
        'task': 'awsim_wheelchair',
        'replay_context': 0,
        'replay': {'size': args.replay_size},
        'run': {'steps': args.steps,
                **({'save_every': args.save_every} if args.save_every else {}),
                **({'log_every': args.log_every} if args.log_every else {}),
                **({'report_every': args.report_every} if args.report_every else {})},
        # Ensure log frequency from code constants is used IF NOT defined in YAML?
        # User requested to use YAML settings, so we prioritize YAML.
        # But if YAML misses something vital like 'eval_envs', defaults handle it.
    }
    config = config.update(overrides)
    
    print("=" * 60)
    
    # Create log directory
    logdir = elements.Path(config.logdir)
    logdir.mkdir()
    config.save(logdir / 'config.yaml')
    # Keep the environment settings next to the agent config (reproducibility)
    import shutil
    shutil.copyfile(args.env_config, str(logdir / 'env_config.yaml'))
    
    print(f"\nConfig saved to {logdir / 'config.yaml'}")
    
    # Import agent
    from dreamerv3.agent import Agent
    
    # Create environment to get spaces
    print("\nInitializing environment...")
    env = make_awsim_env(0, task=args.task, enable_dr=args.enable_dr)
    
    # Get observation and action spaces
    obs_space = {k: v for k, v in env.obs_space.items() if not k.startswith('log/')}
    act_space = {k: v for k, v in env.act_space.items() if k != 'reset'}
    
    print(f"Observation space: {obs_space}")
    print(f"Action space: {act_space}")
    
    # Create agent
    print("\nCreating DreamerV3 agent...")
    from awsim_dreamerv3.jax_compat import adjust_jax_config
    agent_config = elements.Config({
        **config.agent,
        'logdir': config.logdir,
        'seed': config.seed,
        'jax': adjust_jax_config(config.jax, args.compute_dtype),
        'batch_size': config.batch_size,
        'batch_length': config.batch_length,
        'replay_context': config.replay_context,
        'report_length': config.report_length,
        'replica': 0,
        'replicas': 1,
    })
    agent = Agent(obs_space, act_space, agent_config)
    
    # Check for existing checkpoint
    checkpoint_dir = logdir / 'agent'
    checkpoint_path = None
    if checkpoint_dir.exists():
        checkpoints = sorted([p for p in checkpoint_dir.glob('checkpoint_*.pkl')], key=lambda p: os.path.getmtime(str(p)))
        if checkpoints:
            checkpoint_path = checkpoints[-1]
            print(f"Loading existing checkpoint: {checkpoint_path}")
            import pickle
            with open(str(checkpoint_path), 'rb') as f:
                data = pickle.load(f)
                agent.load(data)
    
    # Create replay buffer
    print("Creating replay buffer...")
    replay = embodied.replay.Replay(
        length=config.batch_length + config.replay_context,
        capacity=int(config.replay.size),
        directory=logdir / 'replay',
    )
    
    # Determine total steps from replay or metrics
    total_steps = len(replay)
    
    # Fallback to metrics.jsonl if replay is empty (or inconsistent)
    metrics_path = logdir / 'metrics.jsonl'
    if metrics_path.exists():
        try:
            with open(metrics_path, 'r') as f:
                lines = f.readlines()
                if lines:
                    import json
                    last_line = json.loads(lines[-1])
                    if 'step' in last_line:
                        json_steps = last_line['step']
                        if json_steps > total_steps:
                            print(f"Updating total_steps from {total_steps} to {json_steps} based on metrics.jsonl")
                            total_steps = json_steps
        except Exception as e:
            print(f"Failed to read steps from metrics.jsonl: {e}")
            
    print(f"Resuming from step {total_steps}")

    # Set up callback for training
    step = elements.Counter()
    step.value = total_steps
    logger = elements.Logger(step, [
        elements.logger.TerminalOutput(config.logger.filter, 'Agent'),
        elements.logger.JSONLOutput(logdir, 'metrics.jsonl'),
        elements.logger.TensorBoardOutput(logdir),
    ], 1)
    
    print("\nStarting training loop...")
    print("=" * 60)
    episode = 0
    train_carry = None  # Will be initialized on first train call
    
    # Evaluation metrics tracking
    success_history = []
    collision_history = []
    history_len = 100  # Moving average window
    
    # Early stopping: reward history tracking
    reward_history = []  # Store all episode rewards for convergence detection
    convergence_counter = 0  # Count consecutive episodes meeting convergence criterion
    early_stopped = False
    
    while total_steps < args.steps:
        episode += 1
        
        # Reset environment (embodied style)
        # We need a dummy action with reset=True
        act = {k: v.sample() for k, v in env.act_space.items()}
        act['reset'] = True
        obs = env.step(act)
        
        episode_reward = 0
        episode_length = 0
        done = False
        
        # Initial state for agent
        # Initialize policy state (carry)
        # batch_size is 1 in this loop context because we added [None] manually,
        # but agent.init_policy expects the actual batch size.
        # However, we are running inference for 1 env step by step.
        state = agent.init_policy(batch_size=1)
        
        while not done and total_steps < args.steps:
            # Debug: Check obs types
            if total_steps == 0:
                print("Obs keys and types:")
                for k, v in obs.items():
                    print(f"  {k}: {type(v)}")
                    if hasattr(v, 'shape'):
                        print(f"    shape: {v.shape}")
            
            # Get action from agent
            # Add batch dim for agent
            try:
                obs_batch = {k: np.asarray(v)[None] for k, v in obs.items()}
                if total_steps == 0:
                    print("Obs batch shapes:")
                    for k, v in obs_batch.items():
                        print(f"  {k}: {v.shape}, dtype: {v.dtype}")
            except Exception as e:
                print(f"Error creating obs_batch: {e}")
                raise e

            if args.verbose:
                print(f"Calling agent.policy at step {total_steps}...")
            # agent.policy(carry, obs, mode='train') -> carry, act, out
            state, action, _ = agent.policy(state, obs_batch, mode='train')
            if args.verbose:
                print("Agent policy returned.")
            
            # Remove batch dim
            action = {k: v[0] for k, v in action.items()}
            action['reset'] = False
            
            # Step environment
            next_obs = env.step(action)
            
            # Extract info
            reward = next_obs['reward']
            done = next_obs['is_last'] or next_obs['is_terminal']
            
            # Add transition to replay
            # Replay expects: obs, action, reward, is_first, is_last, is_terminal
            # embodied.Env.step returns 'reward', 'is_first', 'is_last', 'is_terminal' in obs dict
            transition = obs.copy()
            # Convert Python bool to numpy bool_ - JAX cannot handle Python's native bool type
            for key in ['is_first', 'is_last', 'is_terminal']:
                if key in transition:
                    transition[key] = np.bool_(transition[key])
            # Store action as numpy array, not dict - DreamerV3 expects numeric array
            # action dict has 'action' key with the actual numpy array and 'reset' bool
            transition['action'] = np.asarray(action['action'], dtype=np.float32)
            # Add required ext_space keys
            transition['consec'] = np.int32(episode_length)
            transition['stepid'] = np.zeros(20, dtype=np.uint8)  # Placeholder
            replay.add(transition)
            
            # Update
            obs = next_obs
            episode_reward += reward
            episode_length += 1
            total_steps += 1
            step.increment()
            
            # Train
            # Train Ratio 0.5: Train once every 2 environment steps
            if len(replay) >= config.batch_size and total_steps % 2 == 0:
                batch = replay.sample(config.batch_size)
                
                # Debug: print batch dtypes on first train call
                if train_carry is None:
                    print("Batch keys and dtypes before conversion:")
                    for k, v in batch.items():
                        if hasattr(v, 'dtype'):
                            print(f"  {k}: shape={v.shape}, dtype={v.dtype}")
                        else:
                            print(f"  {k}: type={type(v)}")
                
                # Convert any object dtype arrays to proper numeric types
                # This handles data coming from replay buffer
                for key in list(batch.keys()):
                    if hasattr(batch[key], 'dtype'):
                        if batch[key].dtype == object:
                            # Convert based on expected type
                            if key in ['is_first', 'is_last', 'is_terminal']:
                                batch[key] = np.array(batch[key], dtype=np.bool_)
                            elif key == 'action':
                                # Action might be a dict stored as object - need special handling
                                print(f"  Warning: {key} has object dtype, attempting conversion")
                                try:
                                    batch[key] = np.array(batch[key], dtype=np.float32)
                                except:
                                    print(f"  Could not convert {key}")
                            else:
                                print(f"  Warning: {key} has object dtype")
                
                # Debug: print batch dtypes after conversion
                if train_carry is None:
                    print("Batch keys and dtypes after conversion:")
                    for k, v in batch.items():
                        if hasattr(v, 'dtype'):
                            print(f"  {k}: shape={v.shape}, dtype={v.dtype}")
                        else:
                            print(f"  {k}: type={type(v)}")
                
                # Device put batch data - DreamerV3 requires explicit device placement
                # due to JAX transfer_guard='disallow' setting
                from embodied.jax import internal
                batch = internal.device_put(batch, agent.train_sharded)
                
                # Add seed to batch (required by DreamerV3)
                # Use agent's _seeds method to properly place seed on device with correct sharding
                batch['seed'] = agent._seeds(total_steps, agent.train_mirrored)
                # Initialize train carry on first call
                if train_carry is None:
                    train_carry = agent.init_train(config.batch_size)
                train_carry, _, metrics = agent.train(train_carry, batch)
                if total_steps % int(config.run.log_every) == 0:
                    for key, value in metrics.items():
                        logger.scalar(key, value)
                        
                # Generate report (image reconstructions) periodically
                report_every = int(config.run.report_every)
                if total_steps % report_every == 0 and total_steps > 0:
                    print(f"Generating report at step {total_steps}...")
                    try:
                        import jax
                        # Temporarily allow transfers for report generation
                        with jax.transfer_guard('allow'):
                            # Create report batch with correct length (report_length, not batch_length)
                            report_len = int(config.report_length)
                            report_batch = {}
                            for k, v in batch.items():
                                # Convert to numpy for slicing
                                v_np = np.asarray(v)
                                if len(v_np.shape) >= 2:
                                    # Truncate time dimension (axis 1) to report_length
                                    report_batch[k] = v_np[:, :report_len]
                                else:
                                    report_batch[k] = v_np
                            # Place back on device
                            from embodied.jax import internal
                            report_batch = internal.device_put(report_batch, agent.train_sharded)
                            report_batch['seed'] = agent._seeds(total_steps + 1, agent.train_mirrored)
                            report_carry = agent.init_report(config.batch_size)
                            _, report_metrics = agent.report(report_carry, report_batch)
                            
                            # Log images/videos to TensorBoard
                            for key, value in report_metrics.items():
                                if isinstance(value, str):
                                    # e.g. the parameter summary of the agent; not a metric
                                    continue
                                if hasattr(value, 'shape'):
                                    # Convert JAX array to numpy
                                    value = np.asarray(value)
                                    print(f"  Report metric '{key}': shape={value.shape}, dtype={value.dtype}")
                                    if len(value.shape) == 4:
                                        # 4D tensor: video (T, H, W, C)
                                        logger.video(key, value)
                                    elif len(value.shape) == 3:
                                        # 3D tensor: image (H, W, C)
                                        logger.image(key, value)
                                    else:
                                        logger.scalar(f"report/{key}", float(value.mean()))
                                else:
                                    logger.scalar(f"report/{key}", float(value))
                            logger.write()  # Flush to TensorBoard
                        print(f"Report generated with {len(report_metrics)} metrics.")
                    except Exception as e:
                        import traceback
                        print(f"Warning: Report generation failed: {e}")
                        traceback.print_exc()
            
            # Save checkpoint periodically (inside step loop for accurate timing)
            if config.run.save_every and total_steps % int(config.run.save_every) == 0:
                ckpt_data = agent.save()
                ckpt_path = logdir / 'agent' / f'checkpoint_{total_steps}.pkl'
                ckpt_path.parent.mkdir(exist_ok=True, parents=True)
                import pickle
                with open(str(ckpt_path), 'wb') as f:
                    pickle.dump(ckpt_data, f)
                print(f"Checkpoint saved at step {total_steps}: {ckpt_path}")
            
            # Log
            if total_steps % int(config.run.log_every) == 0:
                print(f"Step {total_steps}/{args.steps}, Episode {episode}, "
                      f"Length: {episode_length}, Reward: {episode_reward:.2f}")
        
        # Add last transition (terminal state)
        if done:
            transition = obs.copy()
            # Convert Python bool to numpy bool_ - JAX cannot handle Python's native bool type
            for key in ['is_first', 'is_last', 'is_terminal']:
                if key in transition:
                    transition[key] = np.bool_(transition[key])
            # Action for last step doesn't matter much but needs to match structure
            # Store as numpy array, not dict - DreamerV3 expects numeric array
            dummy_act = env.act_space['action'].sample()
            transition['action'] = np.asarray(dummy_act, dtype=np.float32)
            transition['consec'] = np.int32(episode_length)
            transition['stepid'] = np.zeros(20, dtype=np.uint8)  # Placeholder
            replay.add(transition)
            
            # Evaluation Metrics & Visualization
            # Extract metrics from terminal observation
            if 'is_success' in obs:
                success_history.append(float(obs['is_success']))
            if 'is_collision' in obs:
                collision_history.append(float(obs['is_collision']))
            
            # Maintain history window
            if len(success_history) > history_len:
                success_history.pop(0)
            if len(collision_history) > history_len:
                collision_history.pop(0)
                
            # Log rates if history is available
            if len(success_history) > 0:
                logger.scalar('eval/success_rate', np.mean(success_history))
                logger.scalar('eval/collision_rate', np.mean(collision_history))
            
            # Plot trajectory
            if 'trajectory' in env.info:
                try:
                    traj = env.info['trajectory']
                    # Ensure it's not object array
                    if traj.dtype == object:
                        traj = np.array(traj.tolist(), dtype=np.float32)
                    
                    plot_img = plot_trajectory(traj)
                    logger.image('eval/trajectory', plot_img)
                    print(f"Logged trajectory (length {len(traj)})")
                except Exception as e:
                    print(f"Failed to plot trajectory: {e}")
            
        # End of episode
        logger.scalar('episode/reward', episode_reward)
        logger.scalar('episode/length', episode_length)
        
        # Early stopping: record reward and check convergence
        reward_history.append(episode_reward)
        
        if args.early_stop and total_steps >= args.min_steps:
            converged, improvement_rate, current_avg, prev_avg = check_reward_convergence(
                reward_history, args.window_size, args.improvement_threshold
            )
            
            # Log convergence metrics
            if len(reward_history) >= args.window_size * 2:
                logger.scalar('early_stop/improvement_rate', improvement_rate)
                logger.scalar('early_stop/reward_moving_avg', current_avg)
            
            if converged:
                convergence_counter += 1
                print(f"  Convergence detected ({convergence_counter}/{args.patience}): "
                      f"improvement={improvement_rate:.4f}, avg={current_avg:.2f}")
            else:
                convergence_counter = 0
            
            # Check if we should stop
            if convergence_counter >= args.patience:
                early_stopped = True
                print(f"\n{'='*60}")
                print(f"EARLY STOPPING: Reward converged after {total_steps} steps")
                print(f"Improvement rate: {improvement_rate:.4f} < {args.improvement_threshold}")
                print(f"Final reward moving average: {current_avg:.2f}")
                print(f"{'='*60}")
                break
        
        logger.write()
    
    # Final save
    ckpt_data = agent.save()
    ckpt_path = logdir / 'agent' / 'checkpoint_final.pkl'
    ckpt_path.parent.mkdir()
    import pickle
    with open(str(ckpt_path), 'wb') as f:
        pickle.dump(ckpt_data, f)
    env.close()
    
    print("\n" + "=" * 60)
    print("Training complete!")
    print(f"Total steps: {total_steps}")
    print(f"Total episodes: {episode}")
    print(f"Logs saved to: {logdir}")
    print("=" * 60)


if __name__ == "__main__":
    main()
