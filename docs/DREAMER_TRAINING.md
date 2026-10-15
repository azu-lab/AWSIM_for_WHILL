# Reinforcement learning with DreamerV3

Camera-based obstacle avoidance of the WHILL in the corridor scene, trained with
[DreamerV3](https://github.com/danijar/dreamerv3) (JAX). The environment and the training loop
come from soturon (`world_model/`, `e91b33f`) and were adapted so that they run from a fresh
clone (see §8).

> Scope: this is the **vehicle-camera** task of soturon. It is unrelated to infrastructure-LiDAR
> based planning: the infrastructure LiDAR of the scene is not part of the observation.

## 1. What exists

| Item | Status | Where |
|---|---|---|
| DreamerV3 implementation | DreamerV3 by D. Hafner, commit `b65cf81` (2025-09-23), git submodule | `third_party/dreamerv3` |
| Gymnasium environments | `ObstacleAvoidanceEnv` (task `obstacle_avoidance`, main task), `AWSIMWheelchairEnv` (task `original`) | `rl/awsim_dreamerv3/` |
| AWSIM side | collision (`CollisionPublisher`), reset + randomized obstacle + goal (`EnvironmentReset`), camera | corridor scene |
| Training loop | custom loop around `dreamerv3.agent.Agent` (not `dreamerv3/main.py`) | `rl/scripts/train.py` |
| Evaluation | batch evaluation of checkpoints; single run with trajectory plot; real-WHILL inference (needs a real WHILL, not tested) | `rl/scripts/eval.py`, `sim_single_run.py`, `real_whill_inference.py` |
| Environment test without DreamerV3 | random / constant actions | `rl/scripts/random_agent.py`, `rl/scripts/test_env.py` |
| Trained checkpoints of soturon | **not available** (not in the soturon repository) | – |

## 2. Task definition (`obstacle_avoidance`, defaults of `rl/configs/env/obstacle_avoidance.yaml`)

**Scene.** Corridor (`roukamap`, walls tagged `Obstacle`), one cardboard box (`danbo-ru`) whose
position (x ∈ [−1, −0.5] m, z ∈ [−0.5, 0.5] m around its scene position) and yaw (0–360°) are
randomized at every reset, the sign-board field `shougaikaihi` (62 boards tagged `Obstacle`), and
the goal object `goal_2` 4.2 m ahead of base_link at the start pose (map (−0.562, 4.463)). Randomization ranges are
Inspector values of `DreamerV3Manager/EnvironmentReset`.

**Observation** (`Dict`):

| Key | Space | Source |
|---|---|---|
| `image` | uint8 (64, 64, 3) RGB | `/sensing/camera/traffic_light/image_raw` (1920×1080) resized with `INTER_AREA` |
| `is_success` | float32 () | 1.0 on the step the goal is reached |
| `is_collision` | float32 () | 1.0 on the step of a collision |

`is_success` / `is_collision` were part of the observation of the soturon checkpoints and are kept
for compatibility. DreamerV3 encodes them with its MLP encoder; they are non-zero only on the
terminal step.

**Action**: `Box([-1, 1]², float32)`; `linear.x = clip(a0·0.3, 0, 0.3)` m/s (forward only),
`angular.z = clip(a1·0.6, −0.6, 0.6)` rad/s, sent as `Twist` on `/whill/controller/cmd_vel`
(gear DRIVE is sent at start and after every reset).

**Step**: publish the action, then wait for the next camera frame (max 0.5 s, warning on
timeout). With the camera at 7.9 Hz this is **≈ 7.7 steps/s** (measured).

**Reward** per step: `r = 100·[goal] − 50·[collision] + 30·(d_prev − d) − 0.05`,
d = 2D distance from base_link to the goal (goal from `/awsim/goal_position`).

**Termination**

| Condition | Result |
|---|---|
| `/collision_sensor` true (object tagged `Obstacle`) | terminated, `is_collision = 1` |
| d ≤ 0.5 m | terminated, `is_success = 1` |
| 30 s of wall-clock time since reset | truncated |
| no new camera frame within 0.5 s, reset not acknowledged | **not detected as failure**: warning only, the episode continues (see §9) |

**Reset**: `/reset_command` → AWSIM moves the WHILL to the start pose (zero velocity, gear DRIVE),
re-places the obstacle, re-publishes the goal → `/reset_response`. The environment then waits
until the vehicle is within 2 m of its first observed position.

Start pose and goal can be set from the YAML (`episode.start_pose`, `goal.position`); they are
sent on `/awsim/episode/start_pose` and `/awsim/episode/goal` before every reset.

The `original` task (`rl/configs/env/original.yaml`): 1.0 m/s, 1.0 rad/s, goal radius 0.75 m,
15 s, additional steering-change penalty `−1.0·|Δa1|`.

## 3. Requirements

- NVIDIA GPU with a driver for CUDA 12 (tested: RTX 5090, 32 GB). With the defaults the agent
  has 38.2 M parameters (batch 16 × 64).
- Docker with the NVIDIA Container Toolkit (`docker run --gpus all` must work).
- AWSIM for WHILL player (corridor scene) on the same machine.

Python packages (pinned): `rl/requirements.txt` — `jax[cuda12]==0.4.33` (DreamerV3 pin),
`nvidia-cuda-nvcc-cu12==12.9.86` (see §8), `tensorflow-cpu==2.18.0` + `tensorboard==2.18.0`
(TensorBoard writer of `elements`), `numpy==1.26.4`, `gymnasium==1.4.0`, the DreamerV3
dependencies, and ROS 2 Humble (`rclpy`, `cv_bridge`, `autoware_vehicle_msgs`) from the base image.

## 4. Setup

```bash
git submodule update --init third_party/dreamerv3
docker compose -f docker/compose.yaml build ros2
docker compose -f docker/compose.yaml build dreamer
scripts/build_player.sh          # once; corridor scene
```

## 5. Run

Terminal 1 — AWSIM:

```bash
scripts/run_awsim.sh
```

Terminal 2 — check the environment without learning (2 episodes of random actions):

```bash
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 rl/scripts/random_agent.py --episodes 2
```

Training (short test, then the full run):

```bash
# 400 steps, checkpoint every 200 steps
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 -u rl/scripts/train.py \
    --task obstacle_avoidance --logdir logs/dreamer/smoke_test \
    --steps 400 --save-every 200 --log-every 50 --report-every 200

# full run (soturon default: 1e6 steps)
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 -u rl/scripts/train.py \
    --task obstacle_avoidance --logdir logs/dreamer/obstacle_avoidance --steps 1000000
```

Running `train.py` again with the same `--logdir` **resumes**: it loads the newest
`agent/checkpoint_*.pkl`, the replay buffer in `replay/` and the step count of `metrics.jsonl`.

TensorBoard:

```bash
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh tensorboard --logdir logs/dreamer --bind_all
```

Evaluation of checkpoints (no learning, 5 episodes each, CSV in the current directory):

```bash
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 rl/scripts/eval.py \
    --task obstacle_avoidance --logdir logs/dreamer/smoke_test --checkpoints 400 --episodes 5
```

## 6. Hyper-parameters

DreamerV3 defaults (`third_party/dreamerv3/dreamerv3/configs.yaml`) overridden by
`rl/configs/awsim_obstacle_avoidance.yaml` (soturon values): batch 16 × length 64, replay 1e6,
RSSM deter 512 / stoch 64 / classes 32, encoder/decoder depth 32, policy and value 4 × 256,
`model_lr 1e-4`, `actor_lr`/`critic_lr 3e-5`, `compute_dtype float16` (see §8).
`run.log_every 100`, `report_every 1e4`, `save_every 5e3` (overridable on the command line).

Training frequency: the loop trains **once every 2 environment steps** (hard-coded in `train.py`,
as in soturon); `run.train_ratio` of the YAML is not used by this loop.

## 7. Outputs (`--logdir`)

| File | Content |
|---|---|
| `config.yaml` | full DreamerV3 config of the run |
| `env_config.yaml` | copy of the environment YAML (used by `eval.py` by default) |
| `metrics.jsonl`, `events.out.tfevents.*` | scalars (losses, `episode/reward`, `eval/success_rate`, `eval/collision_rate`), report images/videos, trajectory plots |
| `agent/checkpoint_<step>.pkl`, `checkpoint_final.pkl` | pickled agent state, **≈ 462 MB each** |
| `replay/` | replay buffer chunks |

`logs/` is ignored by Git.

## 8. Differences from soturon and why

| Change | Reason |
|---|---|
| `FromGymnasium` adapter instead of `embodied.envs.from_gym.FromGym` | `from_gym.py` imports the old `gym` and expects the old API (`reset() → obs`, 4-tuple `step`); the environments are `gymnasium` (`reset() → (obs, info)`, 5-tuple). How soturon connected them is not recorded (the `dreamerv3` directory was a gitlink without `.gitmodules`; local modifications are lost). `is_last = terminated or truncated`, `is_terminal = terminated` |
| `compute_dtype` → float32 on GPUs with compute capability ≥ 12 (`jax_compat.py`) | jax 0.4.33 fails on Blackwell with `Unsupported conversion from f16 to f16 / LLVM ERROR` (float16) and `bf16 to f16` (bfloat16); float32 compiles. Checked on an RTX 5090. Older GPUs keep the configured float16. Override with `--compute-dtype` |
| `nvidia-cuda-nvcc-cu12==12.9.86` instead of `<=12.2` | ptxas 12.2 does not know sm_120 ("ptxas too old") |
| `tensorflow-cpu==2.18.0` | needed by the TensorBoard writer of `elements`; TF 2.21 crashes together with jaxlib 0.4.33 (protobuf "File already exists in database") |
| Environment YAML (`--env-config`) | reward / action / topics were Python constants or literals |
| Goal re-published by AWSIM | the goal was published once at start; a training started later used a fallback goal 0.64 m away from the goal object |
| Trajectory plot from `env.info` | the episode trajectory is in `info`, not in the observation |
| Per-step debug output behind `--verbose` | it printed two lines per step |

## 9. Tested and known limitations

Tested (see TEST_REPORT.md): environment reset/step/reward/termination with random and constant
actions, 400-step training with checkpoints at 200 and 400, resuming to 500 steps, image report.
**No long training was run**; there is no evidence yet that the policy learns the task in this
repository (soturon's results/checkpoints are not available).

- Episode timeout and step pacing use wall-clock time, not `/clock`.
- A stalled AWSIM (no camera frames) is not detected as failure.
- Step rate is bound to the camera rate (≈7.7 steps/s); 1e6 steps take ≈ 36 h plus training time.
- One AWSIM instance per training process (topics are global, see ROS2_INTERFACE.md §5).
- Checkpoints of soturon trained with float16 should load (parameters are float32), not tested
  (none available).
- soturon trained with the vehicle parameters of its scene; the default of this repository is the
  complete optimized set (two lateral-friction values differ, VEHICLE_MODEL.md §4). To train
  under the soturon conditions, apply `config/vehicle/whill_model_cr_soturon_scene.yaml` with
  `whill_optimizer apply-params` after starting AWSIM.
