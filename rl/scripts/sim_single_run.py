#!/usr/bin/env python3
"""
Single Simulation Run with Trajectory Recording

シミュレータ（AWSIM）上で学習済みDreamerV3モデルを使い、
1回のシミュレーションを実行して軌道を記録・可視化するスクリプト。

ゴール・衝突判定を無視し、指定時間（デフォルト20秒）走り続けて軌道を記録する。
AWSIM内のダンボールや車椅子の配置は変更しない。

使用例:
  # デフォルト設定（finalチェックポイント）
  python3 scripts/sim_single_run.py

  # チェックポイント指定
  python3 scripts/sim_single_run.py --checkpoint 30000

  # タスク・モデル指定
  python3 scripts/sim_single_run.py \\
    --logdir /path/to/dreamerv3_logs/awsim_wheelchair_v3 \\
    --checkpoint 30000 \\
    --task original \\
    --output-dir ./results
"""

import os
import sys
import argparse
import time
import csv
from pathlib import Path
from datetime import datetime
from typing import List, Tuple, Optional

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

# Add paths
# rl/ (awsim_dreamerv3) と third_party/dreamerv3 を import できるようにする
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'rl'))
sys.path.insert(0, str(REPO_ROOT / 'third_party' / 'dreamerv3'))

# config は軽量なので直接インポート（__init__.py の重い依存を回避）
from awsim_dreamerv3 import config as awsim_config
import pickle


# ─────────────────────────────────────────────
# Data container for per-step records
# ─────────────────────────────────────────────
class StepRecord:
    """1ステップ分の記録データ。"""
    __slots__ = [
        'step', 'time_sec', 'x', 'y',
        'linear_vel', 'angular_vel',
        'action_0', 'action_1', 'reward',
    ]

    def __init__(self, step, time_sec, x, y,
                 linear_vel, angular_vel,
                 action_0, action_1, reward):
        self.step = step
        self.time_sec = time_sec
        self.x = x
        self.y = y
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.action_0 = action_0
        self.action_1 = action_1
        self.reward = reward


# ─────────────────────────────────────────────
# Agent loading (reused from eval.py / real_whill_inference.py)
# ─────────────────────────────────────────────
def load_agent(logdir: Path, checkpoint_step: int):
    """
    DreamerV3 Agentをロードする。

    Args:
        logdir: 学習ログディレクトリ
        checkpoint_step: チェックポイントのステップ番号 (0 = finalを使用)

    Returns:
        (agent, initial_state) のタプル
    """
    import elements
    import ruamel.yaml as yaml
    from dreamerv3.agent import Agent

    # 設定ファイルのロード
    config_path = logdir / 'config.yaml'
    if config_path.exists():
        print(f"  Config: {config_path}")
        config = elements.Config.load(config_path)
    else:
        print("  Config not found in logdir, loading defaults...")
        dreamerv3_folder = REPO_ROOT / 'third_party' / 'dreamerv3' / 'dreamerv3'
        defaults_config = yaml.YAML(typ='safe').load(
            (dreamerv3_folder / 'configs.yaml').read_text()
        )
        config = elements.Config(defaults_config['defaults'])
        config = config.update({
            'logdir': str(logdir),
            'batch_size': 1,
            'batch_length': 64,
        })

    # 観測・行動空間の定義 (学習時と一致させる)
    obs_space = {
        'image': elements.Space(np.uint8, (64, 64, 3), 0, 255),
        'is_first': elements.Space(bool, ()),
        'is_last': elements.Space(bool, ()),
        'is_terminal': elements.Space(bool, ()),
        'reward': elements.Space(np.float32, ()),
        'is_collision': elements.Space(np.float32, ()),
        'is_success': elements.Space(np.float32, ()),
    }

    act_space = {
        'action': elements.Space(np.float32, (2,), -1.0, 1.0),
    }

    # Agent構築
    print("  Creating DreamerV3 agent...")
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

    # チェックポイントのロード
    checkpoint_dir = logdir / 'agent'
    if checkpoint_step == 0:
        checkpoint_path = checkpoint_dir / 'checkpoint_final.pkl'
    else:
        checkpoint_path = checkpoint_dir / f'checkpoint_{checkpoint_step}.pkl'

    if not checkpoint_path.exists():
        available = sorted(checkpoint_dir.glob('checkpoint_*.pkl'))
        available_str = ', '.join([p.stem for p in available])
        raise FileNotFoundError(
            f"Checkpoint not found: {checkpoint_path}\n"
            f"Available: {available_str}"
        )

    print(f"  Loading checkpoint: {checkpoint_path.name}")
    with open(str(checkpoint_path), 'rb') as f:
        data = pickle.load(f)
        agent.load(data)

    print("  Agent loaded successfully!")
    state = agent.init_policy(batch_size=1)
    return agent, state


# ─────────────────────────────────────────────
# Environment creation
# ─────────────────────────────────────────────
def create_env(task: str, max_linear_vel: Optional[float], max_angular_vel: Optional[float]):
    """タスクに応じた環境を作成する。"""
    from awsim_dreamerv3.embodied_adapter import FromGymnasium

    if task in ('obstacle_avoidance', 'obstacle_avoidance_v2'):
        from awsim_dreamerv3.obstacle_avoidance_env import ObstacleAvoidanceEnv
        from awsim_dreamerv3 import obstacle_avoidance_config as oa_config

        lin_vel = max_linear_vel if max_linear_vel is not None else oa_config.MAX_LINEAR_VEL
        ang_vel = max_angular_vel if max_angular_vel is not None else oa_config.MAX_ANGULAR_VEL

        gym_env = ObstacleAvoidanceEnv(
            render_mode="rgb_array",
            image_size=oa_config.IMAGE_SIZE,
            max_linear_vel=lin_vel,
            max_angular_vel=ang_vel,
            episode_timeout=oa_config.EPISODE_TIMEOUT,
            step_duration=oa_config.STEP_DURATION,
        )
        print(f"  [Task] {task}: vel={lin_vel}m/s, ang={ang_vel}rad/s")
    else:
        from awsim_dreamerv3.env import AWSIMWheelchairEnv

        lin_vel = max_linear_vel if max_linear_vel is not None else awsim_config.MAX_LINEAR_VEL
        ang_vel = max_angular_vel if max_angular_vel is not None else awsim_config.MAX_ANGULAR_VEL

        gym_env = AWSIMWheelchairEnv(
            render_mode="rgb_array",
            image_size=awsim_config.IMAGE_SIZE,
            max_linear_vel=lin_vel,
            max_angular_vel=ang_vel,
            goal_distance=awsim_config.GOAL_DISTANCE,
            episode_timeout=awsim_config.EPISODE_TIMEOUT,
            step_duration=awsim_config.STEP_DURATION,
        )
        print(f"  [Task] original: vel={lin_vel}m/s, ang={ang_vel}rad/s, "
              f"goal={awsim_config.GOAL_DISTANCE}m")

    env = FromGymnasium(gym_env, obs_key='image', act_key='action')
    return env, gym_env, lin_vel, ang_vel


# ─────────────────────────────────────────────
# Trajectory plotting
# ─────────────────────────────────────────────
def plot_trajectory(records: List[StepRecord], outcome: str, output_path: Path):
    """軌道をプロットしてPNG保存。"""
    if not records:
        print("  Warning: No trajectory data to plot.")
        return

    xs = [r.x for r in records]
    ys = [r.y for r in records]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ── Left: XY trajectory ──
    ax = axes[0]
    # Color gradient by time
    colors = np.linspace(0, 1, len(xs))
    scatter = ax.scatter(xs, ys, c=colors, cmap='viridis', s=8, alpha=0.8)
    ax.plot(xs, ys, 'k-', alpha=0.2, linewidth=0.5)

    # Start / End markers
    ax.plot(xs[0], ys[0], 'go', markersize=12, label='Start', zorder=5)
    ax.plot(xs[-1], ys[-1], 'r^' if outcome == 'COLLISION' else 'bs',
            markersize=12,
            label=f'End ({outcome})', zorder=5)

    ax.set_xlabel('X (m) [ROS Forward]', fontsize=12)
    ax.set_ylabel('Y (m) [ROS Lateral]', fontsize=12)
    ax.set_title('Trajectory (XY Plane)', fontsize=14)
    ax.set_aspect('equal')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax, label='Time progress')

    # ── Right: Velocity over time ──
    ax2 = axes[1]
    times = [r.time_sec for r in records]
    lin_vels = [r.linear_vel for r in records]
    ang_vels = [r.angular_vel for r in records]

    ax2.plot(times, lin_vels, 'b-', label='Linear vel (m/s)', alpha=0.8)
    ax2.plot(times, ang_vels, 'r-', label='Angular vel (rad/s)', alpha=0.8)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Velocity', fontsize=12)
    ax2.set_title('Velocity Profile', fontsize=14)
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)

    fig.suptitle(f'Simulation Trajectory — {outcome}', fontsize=16, fontweight='bold')
    plt.tight_layout()
    fig.savefig(str(output_path), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Trajectory plot saved: {output_path}")


# ─────────────────────────────────────────────
# CSV saving
# ─────────────────────────────────────────────
def save_csv(records: List[StepRecord], output_path: Path):
    """軌道データをCSVに保存。"""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'step', 'time', 'x', 'y',
            'linear_vel', 'angular_vel',
            'action_0', 'action_1', 'reward',
        ])
        for r in records:
            writer.writerow([
                r.step,
                f"{r.time_sec:.4f}",
                f"{r.x:.6f}",
                f"{r.y:.6f}",
                f"{r.linear_vel:.4f}",
                f"{r.angular_vel:.4f}",
                f"{r.action_0:.4f}",
                f"{r.action_1:.4f}",
                f"{r.reward:.4f}",
            ])
    print(f"  Trajectory CSV saved: {output_path}")


# ─────────────────────────────────────────────
# Main simulation loop
# ─────────────────────────────────────────────
def run_single_simulation(agent, agent_state, env, gym_env, lin_vel, ang_vel, duration=20.0):
    """
    1回のシミュレーションを実行し、ステップごとの記録を返す。
    ゴール・衝突判定を無視し、duration秒間走り続ける。

    Args:
        duration: 走行時間（秒）

    Returns:
        (records, outcome, total_reward, elapsed_time)
    """
    records: List[StepRecord] = []
    events: List[str] = []  # ゴールや衝突のイベントログ

    # ── Reset ──
    print(f"\n  Resetting environment...")
    act = {k: v.sample() for k, v in env.act_space.items()}
    act['reset'] = True
    obs = env.step(act)

    state = agent_state  # Use the initial state from load_agent
    episode_start = time.time()
    total_reward = 0.0
    step_count = 0
    env_episode_ended = False  # 環境がエピソード終了を返したか

    print(f"  Running simulation for {duration:.0f} seconds (ignoring goal/collision)...\n")

    while True:
        elapsed = time.time() - episode_start

        # ── 時間制限チェック ──
        if elapsed >= duration:
            break

        if not env_episode_ended:
            # ── 環境を通じた通常フロー ──
            obs_batch = {k: np.asarray(v)[None] for k, v in obs.items()}
            state, action, _ = agent.policy(state, obs_batch, mode='eval')

            action_dict = {k: v[0] for k, v in action.items()}
            action_dict['reset'] = False

            next_obs = env.step(action_dict)
            reward = next_obs['reward']
            is_last = next_obs['is_last'] or next_obs['is_terminal']

            # イベント記録（ゴール/衝突が起きたが無視して続行）
            if is_last:
                if next_obs.get('is_success', False):
                    events.append(f"  [Event] Step {step_count} ({elapsed:.1f}s): GOAL reached (ignored, continuing)")
                elif next_obs.get('is_collision', False):
                    events.append(f"  [Event] Step {step_count} ({elapsed:.1f}s): COLLISION detected (ignored, continuing)")
                else:
                    events.append(f"  [Event] Step {step_count} ({elapsed:.1f}s): ENV TIMEOUT (ignored, continuing)")
                # 環境はエピソード終了した → 以降はROS2直接制御に切り替え
                env_episode_ended = True

            raw_action = action_dict.get('action', np.array([0.0, 0.0]))
            obs = next_obs
        else:
            # ── 環境エピソード終了後: ROS2直接制御で推論を続行 ──
            # カメラ画像を直接取得
            image = gym_env._ros2.get_image()
            if image is None:
                time.sleep(0.01)
                continue
            import cv2
            image = cv2.resize(image, (64, 64), interpolation=cv2.INTER_AREA)
            image = np.asarray(image, dtype=np.uint8)

            obs_batch = {
                'image': image[None],
                'is_first': np.array([False]),
                'is_last': np.array([False]),
                'is_terminal': np.array([False]),
                'reward': np.array([0.0], dtype=np.float32),
                'is_collision': np.array([0.0], dtype=np.float32),
                'is_success': np.array([0.0], dtype=np.float32),
            }
            state, action, _ = agent.policy(state, obs_batch, mode='eval')

            raw_action = action['action'][0]
            reward = 0.0

            # 速度指令を直接送信
            act_0 = float(raw_action[0])
            act_1 = float(raw_action[1])
            actual_linear = float(np.clip(act_0 * lin_vel, 0.0, lin_vel))
            actual_angular = float(np.clip(act_1 * ang_vel, -ang_vel, ang_vel))
            gym_env._ros2.send_velocity_command(actual_linear, actual_angular)

            # 新しい画像を待機（env.stepと同じタイミング制御）
            time.sleep(0.1)  # STEP_DURATION相当

        # ── Record step data ──
        try:
            pos3d = gym_env._ros2.get_position_3d()
            x_pos, y_pos = pos3d[0], pos3d[1]
        except Exception:
            x_pos, y_pos = 0.0, 0.0

        if not env_episode_ended:
            raw_action = action_dict.get('action', np.array([0.0, 0.0]))
        act_0 = float(raw_action[0])
        act_1 = float(raw_action[1])
        actual_linear = float(np.clip(act_0 * lin_vel, 0.0, lin_vel))
        actual_angular = float(np.clip(act_1 * ang_vel, -ang_vel, ang_vel))

        record = StepRecord(
            step=step_count,
            time_sec=elapsed,
            x=x_pos,
            y=y_pos,
            linear_vel=actual_linear,
            angular_vel=actual_angular,
            action_0=act_0,
            action_1=act_1,
            reward=reward,
        )
        records.append(record)

        total_reward += reward
        step_count += 1

        # ── Progress log (every 10 steps) ──
        if step_count % 10 == 0:
            mode_str = "ENV" if not env_episode_ended else "ROS2"
            print(f"    Step {step_count:4d} | "
                  f"pos=({x_pos:.3f}, {y_pos:.3f}) | "
                  f"vel={actual_linear:.3f}m/s | "
                  f"ang={actual_angular:.3f}rad/s | "
                  f"[{mode_str}]")

    # 停止指令を送信
    gym_env._ros2.send_velocity_command(0.0, 0.0)

    elapsed_total = time.time() - episode_start
    outcome = f'COMPLETED_{duration:.0f}s'

    # イベント表示
    if events:
        print("\n  Events during simulation:")
        for ev in events:
            print(ev)

    return records, outcome, total_reward, elapsed_total


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='シミュレータで1回のシミュレーションを実行し、軌道を記録する',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # デフォルト設定 (finalチェックポイント)
  python3 scripts/sim_single_run.py

  # チェックポイント指定
  python3 scripts/sim_single_run.py --checkpoint 30000

  # タスク指定 + 出力先指定
  python3 scripts/sim_single_run.py \\
    --logdir /path/to/logs \\
    --checkpoint 30000 \\
    --task original \\
    --output-dir ./results
        """
    )
    parser.add_argument(
        '--logdir', type=Path,
        default=str(REPO_ROOT / 'logs' / 'dreamer' / 'awsim_wheelchair_v3'),
        help='学習ログディレクトリ'
    )
    parser.add_argument(
        '--checkpoint', type=int, default=0,
        help='チェックポイントのステップ番号 (0=final)'
    )
    parser.add_argument(
        '--task', type=str, default='original',
        choices=['original', 'obstacle_avoidance', 'obstacle_avoidance_v2'],
        help='タスク選択: original=既存環境, obstacle_avoidance=ダンボール回避, obstacle_avoidance_v2=v2モデル'
    )
    parser.add_argument(
        '--output-dir', type=Path, default=Path('.'),
        help='出力ファイルの保存先ディレクトリ'
    )
    parser.add_argument(
        '--max-linear-vel', type=float, default=None,
        help='最大線速度 (m/s). 未指定時はconfig値を使用'
    )
    parser.add_argument(
        '--max-angular-vel', type=float, default=None,
        help='最大角速度 (rad/s). 未指定時はconfig値を使用'
    )
    parser.add_argument(
        '--duration', type=float, default=20.0,
        help='走行時間 (秒). デフォルト20秒 (実機と同じ)'
    )
    args = parser.parse_args()

    # タスクに応じたデフォルトlogdirの自動設定
    LOGDIR_MAP = {
        'original': str(REPO_ROOT / 'logs' / 'dreamer' / 'awsim_wheelchair_v3'),
        'obstacle_avoidance': str(REPO_ROOT / 'logs' / 'dreamer' / 'obstacle_avoidance'),
        'obstacle_avoidance_v2': str(REPO_ROOT / 'logs' / 'dreamer' / 'obstacle_avoidance_dr_v2'),
    }
    # --logdir が明示的に指定されていない場合、タスクに合わせて自動設定
    default_logdir = REPO_ROOT / 'logs' / 'dreamer' / 'awsim_wheelchair_v3'
    if args.logdir == default_logdir and args.task in LOGDIR_MAP:
        args.logdir = Path(LOGDIR_MAP[args.task])

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    print("=" * 60)
    print("DreamerV3 Single Simulation Run + Trajectory Recording")
    print("=" * 60)
    print(f"  Log directory:  {args.logdir}")
    print(f"  Checkpoint:     {args.checkpoint} {'(final)' if args.checkpoint == 0 else ''}")
    print(f"  Task:           {args.task}")
    print(f"  Duration:       {args.duration}s")
    print(f"  Output dir:     {args.output_dir}")
    print(f"  Mode:           ゴール・衝突を無視して走り続ける")
    print("=" * 60)

    # ── 1. Load Agent ──
    print("\n[1/3] Loading DreamerV3 Agent...")
    agent, agent_state = load_agent(args.logdir, args.checkpoint)

    # ── 2. Create Environment ──
    print("\n[2/3] Creating environment...")
    env, gym_env, lin_vel, ang_vel = create_env(
        args.task, args.max_linear_vel, args.max_angular_vel
    )

    # ── 3. Run Simulation ──
    print("\n[3/3] Running single simulation...")
    try:
        records, outcome, total_reward, elapsed_time = run_single_simulation(
            agent, agent_state, env, gym_env, lin_vel, ang_vel,
            duration=args.duration
        )
    except KeyboardInterrupt:
        print("\n  Simulation interrupted by user.")
        records, outcome, total_reward, elapsed_time = [], 'INTERRUPTED', 0.0, 0.0
    finally:
        env.close()

    # ── Summary ──
    print("\n" + "=" * 60)
    print("SIMULATION RESULT")
    print("=" * 60)
    print(f"  Outcome:        {outcome}")
    print(f"  Steps:          {len(records)}")
    print(f"  Time:           {elapsed_time:.2f} seconds")
    print(f"  Total reward:   {total_reward:.2f}")
    if records:
        xs = [r.x for r in records]
        ys = [r.y for r in records]
        traveled = sum(
            np.sqrt((xs[i] - xs[i-1])**2 + (ys[i] - ys[i-1])**2)
            for i in range(1, len(xs))
        )
        print(f"  Distance:       {traveled:.3f} m")
        print(f"  Start pos:      ({xs[0]:.3f}, {ys[0]:.3f})")
        print(f"  End pos:        ({xs[-1]:.3f}, {ys[-1]:.3f})")
    print("=" * 60)

    # ── Save outputs ──
    if records:
        args.output_dir.mkdir(parents=True, exist_ok=True)

        csv_path = args.output_dir / f"sim_trajectory_{timestamp}.csv"
        png_path = args.output_dir / f"sim_trajectory_{timestamp}.png"

        save_csv(records, csv_path)
        plot_trajectory(records, outcome, png_path)

    print("\nDone!")


if __name__ == "__main__":
    main()
