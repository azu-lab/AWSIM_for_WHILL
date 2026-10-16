# Reinforcement learning (DreamerV3 and your own models)

Camera-based end-to-end learning of the WHILL in AWSIM: 64×64 RGB image of the front camera →
normalized action `[a_v, a_ω]` → `geometry_msgs/Twist`. By default the task is set **on a
sidewalk of West Shinjuku** (default scene and player); a task can be placed anywhere in the map
with a YAML file, obstacles are placed in the scene with the Unity Editor.

The code (`rl/`) follows Fig. 2 of the paper (*Sim-to-Real Aware End-to-End Learning Environment
for Micromobility*): the **learning environment** (`rl/whill_env/`) and the **model**
(`rl/agents/`) are separate. DreamerV3 is one agent; your own model is another one with the same
commands. Overview and file list: [rl/README.md](../rl/README.md).

> Scope: vehicle-camera policies. The infrastructure LiDAR of the scene is not part of the
> observation.

## 1. Structure

| Layer (paper Fig. 2) | Where | Content |
|---|---|---|
| AWSIM side | `Assets/Awsim/Scripts/DreamerV3/` | `EnvironmentReset` (reset, start pose / goal over ROS 2, ground search, obstacle randomization), `CollisionPublisher` (`/collision_sensor`), `GroundProbe` |
| Data management code | `rl/whill_env/data_manager.py` | aggregate data (camera, `/ground_truth/odom`, collision, goal; depth 1, latest value), send command (Twist, gear), reset the scenario |
| Learning environment management code | `rl/whill_env/env.py` | `WhillEnv` (Gymnasium): process data, calculate reward, determine end of episode, convert actions into commands |
| Task | `rl/configs/tasks/*.yaml`, `rl/whill_env/task.py` | start pose(s), goal, limits, reward weights, observation keys, topics |
| E2E policy | `rl/agents/` | `base.py` (interface), `dreamerv3/` (paper), `my_agent.py` (template), `random_agent.py` |
| Runner | `rl/train.py`, `rl/evaluate.py`, `rl/loop.py` | the same for every agent |

## 2. Default task (`rl/configs/tasks/shinjuku_box.yaml`)

![Task area](images/rl_shinjuku_box_area.png)

**Place.** Sidewalk of West Shinjuku, 14 m ahead (Unity +z) of the start pose of the parameter
optimization. Start: base_link at map (−126.0, 32.8), yaw 0 (= Unity (−32.8, z −126), facing
Unity +z). Goal: 4.5 m ahead (map (−121.5, 32.8)), success radius 1.0 m (paper). Obstacle: the
cardboard box `TaskObstacles/cardboard_box` of the default scene (0.28 × 0.54 × 0.5 m, tag
`Obstacle`) 2.5 m ahead; at every reset AWSIM moves it by ±0.4 m (Unity x, lateral) and ±0.5 m
(Unity z) and turns it (0–360°). A shrub planter is ≈ 3 m to the left of the path, the wall of
a building ≈ 5.5 m to the right (both are collisions).

**Observation** (`Dict`, keys from `observation.keys`):

| Key | Space | Source |
|---|---|---|
| `image` | uint8 (64, 64, 3) RGB | `/sensing/camera/front/image_raw` (1920×1080, 10 Hz) resized with `INTER_AREA` |
| `is_success` | float32 () | 1.0 on the step the goal is reached |
| `is_collision` | float32 () | 1.0 on the step of a collision |
| `goal` (optional) | float32 (2,) | distance [m] and bearing [rad] of the goal in base_link |
| `velocity` (optional) | float32 (2,) | measured v [m/s], ω [rad/s] |

`is_success` / `is_collision` are the observation of soturon and the paper (kept for compatibility).

**Action**: `Box([-1, 1]², float32)`; `linear.x = clip(a_v·0.3, 0, 0.3)` m/s (no reverse),
`angular.z = a_ω·0.6` rad/s (`action_to_command`). Gear DRIVE is sent at start and after every
reset.

**Step**: send the command, then wait for the next camera frame (decision step = frame arrival,
paper Sec. III-C; at most `frame_timeout_sec` 0.5 s, a warning and a counter in `info` on timeout).
All other values (odometry, collision flag, goal) are the latest at that moment.

**Reward** per step (paper, Sec. IV-C): `R = r_goal + r_dist + r_collision + r_time + r_smooth`
with `r_goal = 100` (goal reached), `r_dist = 30·(d_{t−1} − d_t)`, `r_collision = −50`,
`r_time = −0.05`, `r_smooth = −1.0·|a_ω,t − a_ω,t−1|` (not on the terminal step); d = 2D distance
base_link–goal.

**End of episode**

| Condition | Result |
|---|---|
| `/collision_sensor`: object tagged `Obstacle`, **or any contact of the vehicle body steeper than 45°** (buildings, curbs, shrubs, poles of the map) | terminated, `outcome = collision` |
| d ≤ 1.0 m | terminated, `outcome = success` |
| 150 steps (`episode.max_steps`, paper) | truncated, `outcome = timeout` |

**Communication** (`DataManager`): sensor data and commands over ROS 2, the episode reset over a
synchronous TCP request:

| Topic | Type | Rate | QoS (subscriber) | Stamp used | Queue |
|---|---|---|---|---|---|
| `/sensing/camera/front/image_raw` | `sensor_msgs/Image` | 10 Hz | BEST_EFFORT | yes (decision step) | 1 |
| `/ground_truth/odom` | `nav_msgs/Odometry` | 20 Hz | BEST_EFFORT | no | 1 |
| `/collision_sensor` | `std_msgs/Bool` | event | RELIABLE | no | 10 |
| `/whill/controller/cmd_vel` (published) | `geometry_msgs/Twist` | 10 Hz (one per step) | RELIABLE + TRANSIENT_LOCAL | no | 1 |

All with KEEP_LAST (drop oldest). The vehicle LiDAR (`/sensing/lidar/top/pointcloud_raw`, 10 Hz)
is not part of the observation and not subscribed. The goal position, the obstacle poses (plots
only) and the gear command are also used.

**Reset** (`DataManager.reset_scenario`): one TCP request `reset_episode` to AWSIM
(`OptimizerBridge`, 127.0.0.1:50051, `awsim` section of the task file) with the start pose and the
goal of the episode. AWSIM puts both **on the ground below (x, y)** (ray cast down, ignoring the
vehicle, the goal and the randomized obstacles; an optional `z` is a height hint for places
under bridges), moves the WHILL upright to the start pose (zero velocity, gear DRIVE),
re-places the obstacles, and answers when this is done, with the simulation time of the reset.
The environment then waits until the odometry is within 0.5 m of the start pose, and takes as
first observation a camera frame stamped after the reset time (frames still in the camera
pipeline would show the place before the reset). Contacts while being moved are
discarded.

Other tasks:

| Task file | Scene / player | Content |
|---|---|---|
| `shinjuku_box.yaml` (default) | default (`Builds/AWSIM_for_WHILL`) | above |
| `shinjuku_goal.yaml` | default | soturon's "original" task at the optimization start pose: goal 5 m ahead, no obstacle, 1.0 m/s, 1.0 rad/s, radius 0.75 m |
| `corridor_box.yaml` | `WhillCorridor.unity` (`Builds/AWSIM_for_WHILL_Corridor`) | soturon's obstacle-avoidance task: corridor, cardboard box, sign boards, goal 4.2 m ahead, radius 0.5 m, 30 s wall-clock limit, no smoothness term |

## 3. Requirements and setup

- NVIDIA GPU with a CUDA 12 driver for DreamerV3 (tested: RTX 5090, 32 GB). With the defaults
  the agent has 38.2 M parameters (batch 16 × 64).
- Docker with the NVIDIA Container Toolkit.
- The default AWSIM player (West Shinjuku), built by `scripts/setup.sh`.

```bash
WITH_DREAMER=1 scripts/setup.sh      # also builds the DreamerV3 image (awsim-for-whill/dreamer)
# or, after scripts/setup.sh:
git submodule update --init third_party/dreamerv3
docker compose -f docker/compose.yaml build dreamer
# corridor task only:
WITH_CORRIDOR=1 scripts/setup.sh     # builds Builds/AWSIM_for_WHILL_Corridor
```

Python packages of the image (pinned): `rl/requirements.txt` — `jax[cuda12]==0.4.33` (DreamerV3
pin), `nvidia-cuda-nvcc-cu12==12.9.86` (§9), `tensorflow-cpu==2.18.0` + `tensorboard==2.18.0`
(TensorBoard writer of `elements`), `numpy==1.26.4`, `gymnasium==1.4.0`, the DreamerV3
dependencies, ROS 2 Humble (`rclpy`, `cv_bridge`, `autoware_vehicle_msgs`) from the base image.

## 4. Run

Terminal 1 — AWSIM (default player, West Shinjuku):

```bash
scripts/run_awsim.sh
```

Terminal 2 (every command runs in the DreamerV3 image; `D` is only a shorthand):

```bash
D="env IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh"

# 1. check the task without a model: 2 episodes of random actions, then 1 straight ahead
$D python3 rl/evaluate.py --agent random --episodes 2 --verbose
$D python3 rl/evaluate.py --agent random --set mode=constant --set "constant=[1.0,0.0]" --episodes 1

# 2. train DreamerV3 (short test, then a long run)
$D python3 -u rl/train.py --agent dreamerv3 --task rl/configs/tasks/shinjuku_box.yaml \
    --logdir logs/rl/smoke --steps 400 --save-every 200 --set loop.report_every=200
$D python3 -u rl/train.py --agent dreamerv3 --task rl/configs/tasks/shinjuku_box.yaml \
    --logdir logs/rl/shinjuku_box --steps 200000

# 3. evaluate checkpoints (no learning): CSV + one trajectory plot per episode in <logdir>/eval/
$D python3 rl/evaluate.py --logdir logs/rl/shinjuku_box --checkpoints latest --episodes 10

# TensorBoard (DreamerV3 losses, episode reward, success / collision rate, trajectories, reconstructions)
$D tensorboard --logdir logs/rl --bind_all
```

Running `train.py` again with the same `--logdir` **resumes**: agent and configuration from
`run.yaml`, task from `task.yaml`, newest checkpoint from `progress.json`; DreamerV3 also reloads
its replay buffer (`replay/`). `--steps` is the total. An episode that is running when `--steps`
is reached is finished, so the last checkpoint can be a few steps later.

The corridor task: start the corridor player
(`AWSIM_BIN=$PWD/Builds/AWSIM_for_WHILL_Corridor/AWSIM_for_WHILL.x86_64 scripts/run_awsim.sh`) and
pass `--task rl/configs/tasks/corridor_box.yaml`.

## 5. Your own task in West Shinjuku

A task = a **place** (task YAML) + **obstacles** (scene).

**1. Pick the place.** Positions are in the map frame of `/ground_truth/odom`. In the Unity Editor
a point at Unity (X, Y, Z) is map (x = Z, y = −X); yaw 0 = facing Unity +z, positive =
counter-clockwise seen from above. Ways to find coordinates:

- `scripts/open_editor.sh`, move the scene view to the place, create an empty object there and
  read its Transform position (then delete it);
- drive the WHILL there (ROS 2 `cmd_vel`) and read `ros2 topic echo --once /ground_truth/odom`;
- top-down images of the map: `WhillSceneTools.CapturePreviewBatch -captureTopDown <half width>
  -captureCenter <x>,<z> -captureHeight <m>` (needs graphics, i.e. without `-nographics`).

Choose a wheelchair-accessible place (sidewalk); the WHILL is put on the ground below (x, y), so
no height is needed. Avoid places under bridges or roofs, or give `z` (map height) as a hint.

**2. Write the task file** (copy `rl/configs/tasks/shinjuku_box.yaml`):

```yaml
name: my_task
start:
  poses:                                  # one drawn per episode
    - {x: -126.0, y: 32.8, yaw: 0.0}
    - {x: -110.0, y: 33.5, yaw: 3.14}
  noise: {x: 0.2, y: 0.2, yaw: 0.1}       # uniform +- per episode
goal:
  ahead: 4.5                              # relative to the drawn start pose, or
  # position: {x: -121.5, y: 32.8}        # absolute
  radius: 1.0
action: {max_linear_vel: 0.3, max_angular_vel: 0.6}
episode: {max_steps: 150}
reward: {goal: 100.0, progress: 30.0, collision: -50.0, time: -0.05, smooth: 1.0}
```

Every key with its default and meaning: `DEFAULTS` in `rl/whill_env/task.py`; unknown keys are
rejected. No code change is needed.

**3. Place obstacles in the Unity Editor.**

```bash
scripts/open_editor.sh            # Unity Editor with the default scene, ROS 2 environment removed
```

- Duplicate `TaskObstacles/cardboard_box` (Ctrl+D) and move it, or add any object with a
  collider (GameObject > 3D Object > Cube, a prefab, ...). In the West Shinjuku scene every contact
  of the vehicle body steeper than 45° counts as a collision; set the tag to `Obstacle` to make
  every contact count (also flat ones).
- To move an obstacle randomly at every reset, add it to `DreamerV3Manager` > `EnvironmentReset` >
  `Obstacles`; the ranges (`Obstacle Random Range Min/Max`, offsets in Unity axes, and the yaw
  range) apply to all listed obstacles.
- Save the scene (Ctrl+S). Then either press **Play** in the Editor (rl/ connects to it like to
  the player, same ROS_DOMAIN_ID) or rebuild the player: `scripts/build_player.sh`.
- `scripts/setup_scenes.sh` regenerates the layouts of `WhillSceneTools.cs`: it re-creates
  `TaskObstacles` (and resets the WHILL placement, goal, randomization ranges); other objects you
  added are kept. To keep a change of `TaskObstacles` across regenerations, edit
  `ShinjukuLayout.TaskObstacles` in `WhillSceneTools.cs`.

**4. Check and train.**

```bash
# can the task be solved? (scripted driver with ground truth, see below)
$D python3 rl/tools/check_task.py --task rl/configs/tasks/my_task.yaml --episodes 10
$D python3 rl/evaluate.py --agent random --task rl/configs/tasks/my_task.yaml --episodes 3 --verbose
$D python3 -u rl/train.py --task rl/configs/tasks/my_task.yaml --logdir logs/rl/my_task --steps 200000
```

`rl/tools/check_task.py` drives every episode with a scripted controller that knows what the
agent does not: the odometry, the goal and the obstacle poses (`/awsim/episode/obstacles`). It
passes obstacles near the straight line on the far side (0.9 m from their center) and steers to
the goal with the velocity limits of the task, through `WhillEnv` (same reward, collision,
success and step limit as in training). If it does not succeed in most episodes, the task is not
learnable as defined (goal too far for `max_steps`, the way blocked, start in contact with
something, ...). It also saves, per episode, a plot (path, start, goal, obstacles) and the first
camera frame in full resolution and as the 64×64 observation, to check what the agent sees.
The plots of `evaluate.py` (`logs/rl/eval_random/`, `<logdir>/eval/`) also show the obstacles.

## 6. Your own model

Implement it in `rl/agents/` — nothing else changes:

1. `cp rl/agents/my_agent.py rl/agents/ppo_agent.py` (and `my_agent.yaml` for its settings).
2. Fill in the marked places: build the model (`__init__`), compute the action (`act`), learn
   (`observe` / `end_episode`), `save` / `load`.
3. Register it: `"ppo": "agents.ppo_agent:PPOAgent"` in `AGENTS` of `rl/agents/__init__.py` (or
   pass `--agent agents.ppo_agent:PPOAgent`).
4. `train.py --agent ppo ...`, `evaluate.py --logdir ...`, `tools/real_whill_inference.py
   --logdir ...` work as for DreamerV3.

The interface (`rl/agents/base.py`):

| Method | Called | Your model |
|---|---|---|
| `__init__(observation_space, action_space, config, logdir)` | once | build networks, optimizer; `config` = YAML + `--set` |
| `begin_episode(obs, training)` | after every reset | reset recurrent state |
| `act(obs, training) -> float32 (2,)` | every step | action in [−1, 1]² |
| `observe(action, reward, next_obs, terminated, truncated, info, training)` | every step | store / learn; `terminated` = goal or collision, `truncated` = step limit |
| `end_episode(stats, training)` | end of episode | e.g. on-policy update, logging (`stats`: outcome, reward, length, trajectory, goal, reward terms) |
| `save(dir)` / `load(dir)` | checkpoints, resume, evaluation | parameters |

`obs` contains the keys of the task (`image` uint8 (64, 64, 3) by default). The agent does not
talk to ROS 2 or AWSIM. Additional Python packages: add them pinned to `rl/requirements.txt` and
rebuild the image (`docker compose -f docker/compose.yaml build dreamer`).

## 7. DreamerV3 settings

`rl/agents/dreamerv3/config.yaml` overrides the DreamerV3 defaults
(`third_party/dreamerv3/dreamerv3/configs.yaml`) with the values of soturon: batch 16 × length 64,
replay 1e6, RSSM deter 512 / stoch 64 / classes 32, encoder/decoder depth 32, policy and value
4 × 256 units, `jax.compute_dtype float16` (float32 on RTX 50xx, §9). Learning rate: the DreamerV3
default `agent.opt.lr 4e-5` — soturon's file also set `opt.actor_lr`, `opt.critic_lr`,
`opt.model_lr` and `expl.expl_amount`, but these keys do not exist in this DreamerV3 version, so
they had no effect and were removed.

`loop` (this repository): one gradient step every `train_every: 2` environment steps (soturon;
`run.train_ratio` of DreamerV3 is not used), losses every `log_every: 100` steps, reconstructions
every `report_every: 10000` steps. Any key: `--set agent.opt.lr=1e-4`, `--set batch_size=8`.

## 8. Outputs (`--logdir`)

| File | Content |
|---|---|
| `run.yaml`, `task.yaml` | agent name and configuration, command line; copy of the task |
| `episodes.jsonl` | per episode: outcome, reward, length, reward terms, start, goal, trajectory, frame timeouts |
| `progress.json` | steps, episodes, newest checkpoint |
| `checkpoints/<step>/` | `dreamerv3.pkl` (≈ 440 MB) and `dreamerv3_state.json` (step); `--keep-checkpoints N` keeps the newest N |
| `config.yaml`, `metrics.jsonl`, `events.out.tfevents.*` | DreamerV3: full configuration, losses, `episode/*`, `eval/success_rate`, `eval/collision_rate` (moving rates of the training episodes), `eval/trajectory` (images), reconstructions (every `loop.report_every` steps) |
| `replay/` | DreamerV3 replay buffer chunks (written with every checkpoint) |
| `eval/` | `evaluate.py`: `results.csv`, `episodes.jsonl` (trajectories, obstacle poses), `<checkpoint>_epNNN.png` |

`logs/` is ignored by Git.

## 9. Differences from soturon and why

| Change | Reason |
|---|---|
| Learning environment (`whill_env`) and model (`agents`) separated, one runner for all agents | paper Fig. 2; a model can be replaced without touching the environment (soturon: one script per use, DreamerV3 calls inside the loop) |
| Task YAML (start poses, goal, limits, reward) instead of Python constants and the scene placement | tasks anywhere in West Shinjuku without code; the corridor task of soturon is `corridor_box.yaml` |
| Start pose / goal put on the ground by AWSIM (`GroundProbe`) | the map is not flat (several meters of height difference); before, the start kept the height of the scene placement |
| Collisions with the map (contact steeper than 45°) in the West Shinjuku scene | buildings, curbs and shrubs of the map are not tagged `Obstacle` (`CollisionPublisher._detectEnvironment`, off in the corridor scene, as in soturon) |
| Step limit 150 (default task) instead of 30 s wall-clock | paper; independent of the machine speed. `episode.timeout_sec` still exists (corridor task) |
| Goal radius 1.0 m (default task) | paper (soturon's code: 0.5 m in the corridor task, kept in `corridor_box.yaml`) |
| DreamerV3 replay buffer saved with each checkpoint and reloaded on resume | soturon's loop wrote it nowhere (`Replay.save()` was never called), so a resumed run started with an empty buffer |
| `FromGymnasium` conversion inside the DreamerV3 agent | `embodied/envs/from_gym.py` imports the old `gym` and expects the old API |
| `compute_dtype` → float32 on GPUs with compute capability ≥ 12 (`jax_compat.py`) | jax 0.4.33 fails on Blackwell with `Unsupported conversion from f16 to f16` (float16) and `bf16 to f16` (bfloat16); float32 compiles (RTX 5090). Override: `--set loop.compute_dtype=float16` |
| `nvidia-cuda-nvcc-cu12==12.9.86` instead of `<=12.2` | ptxas 12.2 does not know sm_120 ("ptxas too old") |
| `tensorflow-cpu==2.18.0` | TensorBoard writer of `elements`; TF 2.21 crashes together with jaxlib 0.4.33 |
| Real-WHILL inference uses `process_image` / `action_to_command` of the environment and any agent | the same input/output conversion in simulation and on the vehicle |
| First observation of an episode: a camera frame stamped after the reset | camera frames arrive with a delay; the frame received right after a reset could show the place before it (seen after a long teleport) |
| Trajectory images in TensorBoard | soturon passed a batched image to `elements.Logger.image`, which failed silently on every episode |
| soturon's `scripts/train.py`, `eval.py`, `random_agent.py`, `test_env.py`, `sim_single_run.py`, `awsim_dreamerv3/` replaced by `train.py`, `evaluate.py` (with `--agent random` for environment checks and a trajectory plot per episode), `whill_env/`, `agents/` | one runner for every agent; `sim_single_run.py` (drive 20 s ignoring goal/collision) has no direct replacement — use `evaluate.py` with a task whose goal is far away and whose `episode.max_steps` is larger (collisions still end the episode) |

## 10. Results on the default task and the pretrained model

Training: `rl/train.py --agent dreamerv3 --task rl/configs/tasks/shinjuku_box.yaml` with the
defaults of this repository (default player; 4-scenario vehicle parameters; camera and decision
steps at 10 Hz; episode reset over TCP), one seed, RTX 5090. 20 000 steps: 144 episodes, 35 min.

![Learning curve](images/rl_shinjuku_box_learning_curve.png)

| Training steps | Training episodes | Success | Collision | Timeout |
|---|---|---|---|---|
| 0–5 000 | 36 | 0 % | 14 % | 86 % |
| 5 000–10 000 | 38 | 37 % | 16 % | 47 % |
| 10 000–15 000 | 34 | 47 % | 3 % | 50 % |
| 15 000–20 000 | 36 | 86 % | 0 % | 14 % |

First success at step 6 677. Evaluation of the checkpoint of step 20 000 without learning
(`rl/evaluate.py`, 20 episodes, box randomized by AWSIM): **20/20 successes**, no collision,
mean reward 175.1, 128–146 steps (mean 136). The exported package gave 5/5 successes (129–148
steps).

![Evaluation trajectories](images/rl_shinjuku_box_eval.png)

The policy passes the box on its left side (map +y) and enters the goal circle at its edge
(final distance ≈ 1.0 m).

**Room for further learning.** The policy is not converged. A successful episode uses 128–146 of
the 150 steps, so a slightly longer path ends in a timeout, which gives no terminal signal. In the
same run, training on to 25 000 and 30 000 steps did not improve the result: the policy took wider
detours and the same evaluation gave 40 % (25 000; timeouts 1.0–1.1 m from the goal) and 0 %
(30 000; it passed beside the goal circle, 1.6–1.8 m away) successes. The checkpoint of 20 000
steps is therefore the published one. Not tried yet: a longer step limit (e.g. 200 steps; with
the earlier 7.7 Hz camera the same 150 steps were ≈ 20 s, and that training stayed at 100 %
from 25 000 to 30 000 steps), other seeds, other training ratios or learning rates.

The checkpoint of step 20 000 is published as **pretrained model** (GitHub Release asset, not in
the Git history, **CC BY-NC 4.0**: it was trained on images of the CC BY-NC West Shinjuku map):

```bash
scripts/fetch_pretrained.sh                     # -> pretrained/shinjuku_box_dreamerv3/ (144 MB download)
$D python3 rl/evaluate.py --logdir pretrained/shinjuku_box_dreamerv3 --episodes 10
$D python3 -u rl/train.py --logdir pretrained/shinjuku_box_dreamerv3 --steps 40000   # continue training
```

The package is a run directory without the optimizer state (156 MB instead of 462 MB; continued
training starts with a fresh optimizer and an empty replay buffer) with `MODEL_CARD.md` (including
the notes above) and `LICENSE.txt`. To publish your own model in the same form:

```bash
$D python3 rl/tools/export_checkpoint.py --logdir logs/rl/my_task --checkpoint latest \
    --name my_task_dreamerv3 --eval-dir logs/rl/my_task/eval --archive --note "limitations ..."
```

Earlier results with the settings before the alignment with the paper (camera at 7.7 Hz,
3-scenario vehicle parameters, ROS 2 reset): 30 000 steps, 100 % success in 40 evaluation
episodes; those settings are no longer the default (TEST_REPORT.md T-D-9).

## 11. Tested and known limitations

See [TEST_REPORT.md](TEST_REPORT.md) (T-D-*). Tested: unit tests without AWSIM; the scripted
driver (`check_task.py`) on the default task; random and straight-ahead episodes (collision with
the box); collisions with a building and a shrub planter of the map; DreamerV3 training to
30 057 steps with checkpoints and evaluation (§10), resume (agent, step count and replay buffer);
export and loading of the published package; `my_agent` through `train.py` / `evaluate.py`,
`random` through `evaluate.py`; the corridor task with the rebuilt corridor player.

- One AWSIM per training process (topics are global, ROS2_INTERFACE.md §5).
- The step rate is bound to the camera rate (10 Hz; ≈ 9.5 steps/s with training); 20 000 steps
  took 35 min.
- A stalled AWSIM is not detected as failure (warnings and `frame_timeouts` only).
- Obstacles are scene objects; they cannot be placed per episode from Python (only the
  randomization of `EnvironmentReset`).
- `rl/tools/real_whill_inference.py` needs a real WHILL and was not run (it was adapted to the
  agent interface; the processing functions are those tested in simulation).
- soturon trained with the vehicle parameters of its scene; the default of this repository is
  the 4-scenario optimum (VEHICLE_MODEL.md §4). To train under soturon's conditions, apply `config/vehicle/whill_model_cr_soturon_scene.yaml` with
  `whill_optimizer apply-params` after starting AWSIM.
