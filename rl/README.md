# rl/ — learning environment and models

End-to-end learning (camera → velocity command) of the WHILL in AWSIM. The code is split like
Fig. 2 of the paper (*Sim-to-Real Aware End-to-End Learning Environment for Micromobility*):
the **learning environment** and the **model** are separate, so a model can be replaced without
touching the environment, and the environment can be changed (task, place in the map, reward)
without touching the model.

```
AWSIM (Unity)                     rl/whill_env/  — learning environment (model-agnostic)       rl/agents/  — models
┌──────────────────────┐          ┌───────────────────────────┐   ┌──────────────────────────┐   ┌────────────────────────┐
│ West Shinjuku map     │ROS2+TCP │ data_manager.py           │   │ env.py  (WhillEnv, gym)  │   │ base.py   Agent API    │
│ WHILL Model CR        │ ──────▶ │  Data management code     │──▶│  Learning environment    │──▶│ dreamerv3/ (paper)     │
│ camera, odometry,     │         │  - aggregate data         │   │  management code         │   │ my_agent.py  ← YOUR    │
│ collision, reset      │ ◀────── │  - send command (Twist)   │◀──│  - process data (64x64)  │◀──│              MODEL     │
│ obstacles: placed in  │         │  - reset the scenario     │   │  - calculate reward      │   │ random_agent.py        │
│ the Unity Editor      │         └───────────────────────────┘   │  - determine end         │   └────────────────────────┘
└──────────────────────┘                                          │  - action → command      │
                                  task.py + configs/tasks/*.yaml  └──────────────────────────┘
                                  start pose, goal, limits, reward weights (no code)
```

| I want to … | Edit | Do not edit |
|---|---|---|
| use my own model (PPO, SAC, a transformer, …) | a new file in `agents/` (copy `my_agent.py`), one line in `agents/__init__.py` | `whill_env/` |
| change DreamerV3 hyper-parameters | `agents/dreamerv3/config.yaml`, or `--set key=value` | – |
| train somewhere else in West Shinjuku, change goal / limits / reward | a task YAML in `configs/tasks/` | Python |
| add / move obstacles | the scene in the Unity Editor (see `docs/DREAMER_TRAINING.md` §5) | Python |
| change what the agent observes, the reward formula, the termination | `whill_env/env.py` | `agents/` |
| talk to another simulator or the real vehicle | `whill_env/data_manager.py` (same methods) | `env.py`, `agents/` |

## Files

| Path | Role |
|---|---|
| `whill_env/data_manager.py` | **Data management code.** ROS 2 for sensor data and commands (camera 10 Hz, `/ground_truth/odom` 20 Hz, `/collision_sensor`, goal; latest value), `cmd_vel` / gear publishers; episode reset over a synchronous TCP request (`reset_episode` to AWSIM, 127.0.0.1:50051) |
| `whill_env/env.py` | **Learning environment management code.** `WhillEnv` (Gymnasium): `process_image`, `compute_reward`, termination, `action_to_command`. One step = one new camera frame |
| `whill_env/task.py` | Task YAML: start poses (+ noise), goal (absolute or `ahead` of the start), goal radius, velocity limits, step limit, reward weights, observation keys, topics |
| `whill_env/domain_randomization.py` | Optional image randomization (brightness, contrast, hue, saturation, noise) for sim-to-real |
| `configs/tasks/` | `shinjuku_box.yaml` (default: West Shinjuku sidewalk, cardboard box), `shinjuku_goal.yaml` (no obstacle), `corridor_box.yaml` (corridor scene of soturon) |
| `agents/base.py` | **`Agent` interface**: `begin_episode`, `act`, `observe`, `end_episode`, `save`, `load` |
| `agents/my_agent.py` | **Template for your model** (runs as it is) |
| `agents/dreamerv3/` | DreamerV3 of the paper (`third_party/dreamerv3`), `config.yaml` |
| `agents/random_agent.py` | random / constant actions |
| `train.py` | train any agent on any task; resumes from the newest checkpoint |
| `evaluate.py` | run without learning: success / collision / timeout, trajectory plots, CSV |
| `loop.py` | the episode loop shared by both |
| `tools/real_whill_inference.py` | run a trained agent on a real WHILL (same `process_image` / `action_to_command`) |
| `tools/check_task.py` | **check that a task can be solved** before training: a scripted driver with ground truth (odometry, goal, obstacle poses) drives each episode through `WhillEnv`; plots, first camera frame / 64×64 observation per episode |
| `tools/export_checkpoint.py` | package a trained DreamerV3 checkpoint for publication (without optimizer state, model card, CC BY-NC 4.0 notice, `.tar.gz` + SHA-256); `scripts/fetch_pretrained.sh` downloads the published one |
| `tools/play_rosbag_awsim.py`, `tools/correct_trajectory.py` | replay a rosbag in AWSIM; normalize recorded trajectories |
| `tests/` | `pytest` without AWSIM (fake data manager) |

## Commands

From the repository root, with AWSIM running (`scripts/run_awsim.sh`):

```bash
D="env IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh"
$D python3 rl/tools/check_task.py --episodes 10                            # is the task solvable?
$D python3 rl/evaluate.py --agent random --episodes 2 --verbose          # check the environment
$D python3 -u rl/train.py --agent dreamerv3 --logdir logs/rl/shinjuku_box --steps 200000
$D python3 rl/evaluate.py --logdir logs/rl/shinjuku_box --episodes 10
$D python3 -u rl/train.py --agent my_agent --logdir logs/rl/mine --steps 1000   # your model
$D python3 -m pytest rl/tests                                              # without AWSIM
```

Details, the task file format and how to place obstacles: [docs/DREAMER_TRAINING.md](../docs/DREAMER_TRAINING.md).
