# Differences from upstream AWSIM

## 1. Upstream

| | |
|---|---|
| Repository | [autowarefoundation/AWSIM](https://github.com/autowarefoundation/AWSIM) (formerly `tier4/AWSIM`) |
| Base commit | [`fa1dc6592175a7510b9bd9d429a32d69637242fa`](https://github.com/autowarefoundation/AWSIM/commit/fa1dc6592175a7510b9bd9d429a32d69637242fa) — "Merge pull request #433 from tier4/docs/format_usecase", 2025-09-22 |
| Release | between **v2.0.0** (2025-08-01, 235 commits before the base) and **v2.0.1** (2025-10-31, 71 commits after it); not a tagged release |
| Unity | 6000.0.34f1 (`ProjectSettings/ProjectVersion.txt`) |
| ROS 2 | Humble, Ros2ForUnity 1.3.0 standalone (bundled in `Assets/Ros2ForUnity`) |
| LiDAR | Robotec GPU Lidar (RGL) 0.18.0 (`Assets/RGLUnityPlugin`) |

How the base was identified: the first commit of shoumexi-03/soturon (`e2c2734`, 2025-10-21) has
`AWSIM` as a git submodule pointing at `fa1dc65`; the next commit (`ff23fb9`) replaced the
submodule by the files. A file-by-file comparison (`git ls-tree` blob hashes) of `ff23fb9:AWSIM`
with upstream `fa1dc65` shows 0 deleted files, 5 modified files and 13 added files (the WHILL
model), i.e. the copy was taken from that commit.

History of the WHILL changes: shoumexi-03/soturon, branch `optimize`, 30 commits from
2025-10-21 to 2026-03-05 (`e91b33f`). That repository is private and contains assets that cannot
be published, so this repository has a **new history**:

| Commit (this repo) | Content |
|---|---|
| `Import AWSIM fa1dc65 as the base` | upstream files at `fa1dc65`, without `docs/`, `mkdocs.yml`, `README*` (AWSIM License, see ASSETS_AND_LICENSES.md) |
| `Bring in the WHILL changes of soturon/optimize (code only)` | soturon's code changes |
| later commits | placeholder model, infrastructure LiDAR, tools, fixes, Python, docs |

So `git diff <first commit> HEAD` is exactly "everything this fork changed in upstream AWSIM".

## 2. Overview

Upstream AWSIM simulates an Autoware car (Lexus RX450h) in the Shinjuku map. This fork:

- replaces the ego vehicle by a **WHILL Model CR** (differential drive, velocity command over
  `geometry_msgs/Twist`), with physics parameters fitted to a real WHILL;
- drives in a **corridor scene with obstacles** built from primitives, used for reinforcement
  learning with **DreamerV3**;
- adds a **TCP bridge for parameter optimization** (Optuna) against real-WHILL recordings;
- adds an **infrastructure LiDAR** (this repository) and ground-truth / collision / reset topics.

Unity project files changed against `fa1dc65` (without `.meta`): **23 added, 13 modified,
0 deleted** (`git diff --name-status <base> HEAD -- Assets ProjectSettings Packages`).

## 3. Added features

| Feature | Files | Origin |
|---|---|---|
| WHILL Model CR prefab (physics, sensors, placeholder visual) | `Assets/Awsim/Prefabs/Entity/EgoVehicle/WhillModelCR/` | soturon prefab, visual replaced here |
| Ground-truth odometry `/ground_truth/odom` | `Assets/Awsim/Models/Entity/Sensor/GroundTruthPublisher.cs` | soturon |
| Wheel speeds `/whill/wheel_velocities_sim` | `Assets/Awsim/Models/Entity/Sensor/WheelSpeedPublisher.cs` | soturon |
| Collision events `/collision_sensor` (tag `Obstacle`) | `Assets/Awsim/Scripts/DreamerV3/CollisionPublisher.cs`, `ProjectSettings/TagManager.asset` | soturon |
| Episode reset, randomized obstacle, goal publication | `Assets/Awsim/Scripts/DreamerV3/EnvironmentReset.cs` | soturon; goal re-publication and start-pose/goal topics added here |
| Optimizer bridge (TCP 50051, JSON) | `Assets/Awsim/Scripts/Optimizer/OptimizerBridge.cs`, `UnityMainThreadDispatcher.cs` | soturon; loopback binding and `get_parameters` added here |
| Center-of-mass gizmo | `Assets/Awsim/Scripts/Debug/CoMVisualizer.cs` | soturon |
| Corridor scene: walls/floor (`roukamap`), cardboard box, sign boards, goals, `DreamerV3Manager` | `Assets/Awsim/Scenes/AutowareSimulationDemo.unity`, materials in `Assets/Awsim/Models/Entity/` | soturon (SLAM mesh and photo texture removed) |
| Infrastructure LiDAR (YAML-driven) + `/tf_static` | `config/infra_lidar.yaml`, `Assets/Awsim/Scripts/InfraLidar/`, `Assets/Awsim/Scripts/Editor/InfraLidar/` | this repository |
| Obstacle-free scene for the optimization | `Assets/Awsim/Scenes/WhillOpenField.unity`, `Assets/Awsim/Scripts/Editor/WhillModelCR/WhillSceneTools.cs` | this repository |
| Editor tools: placeholder builder, parameter export, scene validator, player builder | `Assets/Awsim/Scripts/Editor/WhillModelCR/`, `Assets/Awsim/Scripts/Editor/Build/` | this repository |
| Python: optimizer, DreamerV3 environment, ROS 2 examples | `optimizer/`, `rl/`, `ros2_examples/` | soturon (adapted) / this repository |

## 4. Modified upstream files

| File | Change |
|---|---|
| `Scripts/Entity/Vehicle/AccelVehicle/AccelVehicle.cs` | Ackermann acceleration model → differential drive: inverse kinematics, per-drive-wheel PID, per-wheel lateral damping, rolling/air drag, speed limiter; steering fixed to 0; Rigidbody mass forced to 55 kg; `MaxAcceleration` constant 3.0; `SidewaySlipMultiplier` disabled; angular velocity from the Rigidbody |
| `Scripts/Entity/Vehicle/AccelVehicle/Input/AccelVehicleRos2Input.cs` | Subscribes `geometry_msgs/Twist` `/whill/controller/cmd_vel` |
| `.../Input/IAccelVehicleInput.cs` | `TargetLinearVelocity`, `TargetYawRate` |
| `.../Input/AccelVehicleControlModeBasedInputter.cs` | Forwards the velocity targets; sets the gear only when it changes |
| `.../Input/AccelVehicleKeyboardInput.cs`, `AccelVehicleLogitechG29Input.cs` | Implement the new interface members (0); keyboard no longer reads back the vehicle state |
| `Scenes/AutowareSimulationDemo/EgoVehicle.cs` | IMU, GNSS and visual effects commented out |
| `Scenes/AutowareSimulationDemo.unity` | WHILL instead of the Lexus (Lexus kept inactive), corridor, obstacles, RL manager, infrastructure LiDAR; Shinjuku environment set inactive |
| `Scripts/UI/UIWindow/Entity/AccelVehicleInformationUIWindow.cs` | Shows the yaw rate |
| `Inputactions/Awsim.cs` | Regenerated by Unity (namespace `Awsim.EgoVehicle`, header removed) |
| `RGLUnityPlugin/Scripts/SceneManager.cs`, `LowLevelWrappers/RGLNativeAPI.cs` | `FindObjectsOfType` → `FindObjectsByType` (deprecated API) |
| `ProjectSettings/TagManager.asset` | Tag `Obstacle` |

## 5. Removed / disabled upstream features

Nothing was deleted from the upstream project. What is not used:

| Feature | Status | Where / how to re-enable |
|---|---|---|
| Lexus RX450h ego vehicle | **Present, inactive** in the scene | activate `AutowreSimulationDemo/EgoVehicle/Lexus RX450h 2015` and point `AutowareSimulationDemo._egoVehicle` back to it (the modified AccelVehicle is a differential drive for both) |
| Shinjuku map | **Inactive**; its meshes need the AWSIM Externals (not included) | download the AWSIM externals, import, activate `Environments` |
| NPC vehicles (TrafficSimulator) | **Code present, effectively disabled by scene settings**: no NPC prefabs and no spawnable lanes in `TrafficSimulator` | configure `_randomTrafficSims` in the scene (needs the Shinjuku lanes) |
| Pedestrians | `humanElegant` instances present under `TrafficSimulator/NPCPedestrians`; **no walker controllers** (`_simplePedestrianWalkerControllers` empty), so they do not move | add controllers in `AutowareSimulationDemo` |
| V2I | **Disabled by setting** (`_useV2i = 0`) | set `_useV2i` |
| IMU / GNSS of the ego | **Disabled in code** (`EgoVehicle.cs`) | uncomment |
| Autoware connection | Upstream topics are still served (`/control/command/*`, `/vehicle/status/*`); acceleration/steering commands no longer drive the vehicle | – |
| Scenario simulator / other demo scenes (`IntegrateScenarioSimulatorDemo`, `PcdGenerationDemo`, `AutowareSimulationURPDemo`) | **Unchanged upstream scenes**; not checked with the modified AccelVehicle; need the Shinjuku externals | – |
| Upstream documentation | **Removed from this repository** (license) | https://autowarefoundation.github.io/AWSIM/ |

## 6. Using the fork-specific features

| Task | Document |
|---|---|
| Drive the WHILL, read odometry / LiDAR | [ROS2_INTERFACE.md](ROS2_INTERFACE.md), [ROS2_EXAMPLES.md](ROS2_EXAMPLES.md) |
| Change the vehicle parameters, re-run the optimization | [PARAMETER_OPTIMIZATION.md](PARAMETER_OPTIMIZATION.md) |
| Train / evaluate DreamerV3 | [DREAMER_TRAINING.md](DREAMER_TRAINING.md) |
| Vehicle model, placeholder | [VEHICLE_MODEL.md](VEHICLE_MODEL.md) |
| Infrastructure LiDAR | `config/infra_lidar.yaml`, README |
