# Test report

Status values: **PASS** = executed and succeeded; **FAIL** = executed and failed;
**BLOCKED** = could not be executed in this environment; **NOT IMPLEMENTED** = feature does not
exist; **NOT TESTED** = exists but was not executed. Reading the code alone never counts as PASS.

## 1. Environment

| | |
|---|---|
| Date | 2026-10-15 (clean clone, corridor default at that time), 2026-10-16 (West Shinjuku default, RL rework, fresh setup) |
| Machine | Ubuntu 22.04.5 LTS, AMD Ryzen Threadripper 9960X, 256 GB RAM, NVIDIA RTX 5090 32 GB (driver 580.178.04), X display :1 |
| Unity | 6000.0.34f1, batchmode (`scripts/unity_lib.sh`), Unity Hub signed in |
| AWSIM | standalone Linux player built by `scripts/build_player.sh` (window 1280×720) |
| ROS 2 | Humble, `rmw_cyclonedds_cpp`, `config/cyclonedds.xml`, `ROS_DOMAIN_ID=77`, Docker images `awsim-for-whill/ros2`, `awsim-for-whill/dreamer` |
| Clean environment | `git clone --recurse-submodules` of the committed repository into a new directory (`clean_clone/`), **no `Library/` copied** (fresh import), Docker images built from that clone with `docker compose`. Neither the soturon working copy nor any private asset is reachable from it |

Three working trees were used:

- *clean*: `git clone --recurse-submodules` of commit `c879702` (2026-10-15) with a fresh
  `Library/`. At that time the default scene was the corridor; the items marked *clean* below were
  run in that state.
- *dev*: the development tree with all current (uncommitted) changes: West Shinjuku default scene,
  task obstacle, collisions with the map, the reworked `rl/` (2026-10-16).
- *fresh*: a copy of the development tree of 2026-10-16 morning **without `Library/`,
  `Assets/Awsim/Externals/` (map) and `Builds/`**, on which `scripts/setup.sh` was run as after a
  clone (marked *fresh*). Docker layer caches and Unity's global caches of the machine were
  present, so the measured duration (4 min) is not representative of a new machine.

## 2. Results (section 9.1 of the task)

| ID | Item | Status | Evidence |
|---|---|---|---|
| T-S-1 | Setup after clone: map download/import, Docker image, player build | **PASS** *fresh* | `SHINJUKU_PACKAGE=<cached package> scripts/setup.sh`: exit 0; `Assets/Awsim/Externals/Shinjuku` imported (2 min), `awsim-for-whill/ros2` built, player built. The player of this tree: `drive_test.py` 8/8 in West Shinjuku (forward +1.201 m) |
| T-S-2 | Map download from the official URL | **PASS** *dev* | `curl -L https://github.com/tier4/AWSIM/releases/download/v2.0.0/Shinjuku.unitypackage`: see §7 (the setup itself used a cached copy of the same file) |
| T-U-1 | Unity project opens (import, compile) | **PASS** *clean* | `scripts/validate_project.sh` in the clean clone: fresh import, 0 compile errors (`error CS` absent), exit 0 |
| T-U-2 | No missing script | **PASS** *clean* | validator: 0 missing scripts on 1 712 active objects; `WhillModelCR.prefab`: missing scripts = 0 |
| T-U-3 | No missing asset | **PASS** for active objects *clean*; see note | active objects: 0 broken references, 0 renderers without mesh/material, 0 missing prefabs. Inactive `Environments/Shinjuku` (upstream, disabled): 6 741 broken references, 1 967 meshes/materials missing — they belong to the AWSIM Externals (Shinjuku map, not redistributable) and were in the same state upstream without externals |
| T-U-4 | Scene loads | **PASS** *clean* | validator opens `AutowareSimulationDemo.unity`; player loads it |
| T-U-5 | AWSIM starts | **PASS** *clean* | `scripts/build_player.sh` exit 0; `BG=1 scripts/run_awsim.sh` → "Ready." (OptimizerBridge on 127.0.0.1:50051) |
| T-U-7 | Current scenes valid (default West Shinjuku, corridor) | **PASS** *dev* | `scripts/validate_project.sh`: default scene 5 274 objects (4 940 active), corridor scene 5 272 (1 712 active): 0 problems on active and inactive objects; prefab 0 missing |
| T-U-6 | WHILL is displayed | **PASS** *dev* | `docs/images/whill_model_cr_shinjuku.png`, `whill_model_cr_corridor.png` rendered by `WhillSceneTools.CapturePreviewBatch` (placeholder + VLP16 visible) |
| T-V-1 | WHILL moves forward | **PASS** *clean* | `drive_test.py`: +0.40 m/s × 3 s → +1.164 m along the heading, lateral −0.002 m, v_end 0.399 m/s |
| T-V-2 | WHILL turns | **PASS** *clean* | ±0.5 rad/s × 3 s → +83.2° / −83.1° (commanded 86°), translation < 0.01 m |
| T-V-3 | WHILL stops | **PASS** *clean* | after each stop phase \|v\| = 0.000 m/s, \|ω\| = 0.000 rad/s |
| T-V-4 | WHILL moves backward | **PASS** *clean* | −0.30 m/s × 3 s → −0.866 m |
| T-V-6 | Driving in West Shinjuku (default scene) | **PASS** *dev* + *fresh* | `drive_test.py` 8/8: forward +1.201 m, backward −0.844 m, turns +81.9° / −81.5° (−81.1° in the last run), stops \|v\| = 0.000 m/s |
| T-V-5 | Same with the final default parameters | **PASS** *dev* | player rebuilt at `86a234c`+; `get-params`: skidding_cancel_rate 33.5545, front_wheel_skidding_rate 1.10522; `drive_test.py` 8/8 (forward +1.171 m, turns +81.1° / −80.7°) |
| T-R-1 | ROS 2 velocity command received | **PASS** *clean* | T-V-1..4; topic info: subscriber node `AWSIM`, TRANSIENT_LOCAL |
| T-R-2 | QoS: VOLATILE publisher not matched, TRANSIENT_LOCAL matched | **PASS** *dev* | rclpy VOLATILE publisher: "requesting incompatible QoS … DURABILITY", 0.000 m moved; TRANSIENT_LOCAL: 0.866 m. `ros2 topic pub` without flags also moves the WHILL (Humble default transient_local) |
| T-R-3 | Odometry obtained | **PASS** *clean* | `/ground_truth/odom` 23.35 Hz, `map → base_link` |
| T-R-4 | Infrastructure LiDAR point cloud obtained | **PASS** *clean* | `/infrastructure/lidar_0/points` 10.0 Hz, 43 148 points, 1 035 552 B; `/tf_static` map → infrastructure_lidar_0 |
| T-R-5 | Infrastructure LiDAR sees the WHILL | **PASS** *clean* (corridor) / *dev* (West Shinjuku) | `infra_lidar_check.py`: corridor median 275 points at 6.0 m; West Shinjuku median 56 points at 11.9 m (threshold 20); cloud 60 082 points, 1 441 968 B, 10 Hz |
| T-R-6 | All interfaces | **PASS** *clean* | `check_interfaces.py` table (§3) |
| T-R-7 | `/clock`, stamps on the same time base | **PASS** *clean* | /clock 99.2 Hz; stamp − /clock: odom 0.021 s, image 0.021 s, clouds 0.000 s |
| T-R-8 | Pause / resume behaviour of time | **NOT TESTED** | described from the code in ROS2_INTERFACE.md §4 |
| T-E-1 | Collision with obstacle detected | **PASS** *clean*, *dev* | `reset_collision_test.py` (corridor player, rebuilt with the current code): `/collision_sensor` after 6.2 s, 2.45 m, exit 0; West Shinjuku: collision with `cardboard_box` (T-D-2) |
| T-E-5 | Collision with the map (West Shinjuku) | **PASS** *dev* | task with the WHILL facing the building / the shrub planter, straight ahead: `Collision detected with SJK01_P06_Building` after 5.5 m, `… with ShrubZenrin_…` after 3.0 m; driving 20 m along the sidewalk (`reset_collision_test.py` in West Shinjuku) raised no collision (no false positive from the ground) |
| T-E-6 | Start pose / goal anywhere, put on the ground | **PASS** *dev* | start map (−126.0, 32.8) → root Unity y −7.79 (ground of the scene placement −7.37 m 14 m away); goal Unity y −7.92; corridor task with height hints → root (−0.60, −0.68, −0.36) vs scene placement (−0.603, −0.704, −0.363) |
| T-E-2 | Reset | **PASS** *clean* | `/reset_response` received, distance to start pose 0.000 m |
| T-E-3 | Start pose / goal over ROS 2 | **PASS** *clean* | commanded (−0.030, 0.106, 124.9°) → odom error 0.000 m / 0.00°; goal echo on `/awsim/goal_position` |
| T-E-4 | Goal position available to late subscribers | **PASS** *clean* | `/awsim/goal_position` 1.0 Hz (was published once before the fix) |
| T-D-1 | Learning environment initializes (`whill_env`) | **PASS** *dev* | `evaluate.py --agent random`: observation `image (64,64,3) uint8`, `is_success`, `is_collision`; task `shinjuku_box` |
| T-D-2 | Episodes in West Shinjuku (default task) | **PASS** *dev* | straight ahead (`--agent random --set mode=constant`): collision with the box after 35 / 43 steps; `shinjuku_goal` straight ahead: success after 36 steps (reward 237.3); random actions: timeouts after 150 steps |
| T-D-3 | DreamerV3 training (new `rl/train.py`) | **PASS** *dev* | `train.py --agent dreamerv3 --task shinjuku_box --steps 400 --save-every 200`: JIT (float32 on RTX 5090), 3 episodes, reports at 200/400, checkpoints 200/400/450 (441 MB each), replay saved (3 chunks), exit 0 |
| T-D-4 | Resume | **PASS** *dev* | same `--logdir`, `--steps 600`: "Loading checkpoint 000000450", "replay buffer: 390 items loaded", trained to 600, exit 0 |
| T-D-5 | Evaluation of checkpoints | **PASS** *dev* | `evaluate.py --checkpoints 000000200,000000600 --episodes 2`: 200: collision + timeout, 600: 2 timeouts (expected after a few hundred steps), CSV + 4 plots |
| T-D-6 | Other agents through the same runner | **PASS** *dev* | `train.py --agent my_agent --steps 300 --save-every 150` (5 episodes, checkpoints 300/336), `evaluate.py --logdir` of it; `random` in all `evaluate.py` runs |
| T-D-7 | Corridor task (`corridor_box`) | **PASS** *dev* | corridor player: straight ahead → collision with `danbo-ru` (2 episodes), random → 30 s timeouts (≈ 240 steps) |
| T-D-8 | Unit tests of the environment / agents without AWSIM | **PASS** | `pytest rl/tests`: task files, goal relative to the start yaw, noise bounds, unknown keys, action scaling, reward terms, success / collision / step limit with a fake data manager, optional observation keys, agents save/load |
| T-D-9b | Learning performance with the current defaults (10 Hz, TCP reset, 4-scenario vehicle parameters) | **PASS** *dev* (at 20 000 steps) | DreamerV3, `shinjuku_box`, one seed: first success at step 6 677; training episodes 15–20 k: 86 % success / 0 % collision; evaluation, 20 episodes each: **20 000 steps 100 %** (128–146 steps), 25 000 steps 40 % (60 % timeouts), 30 057 steps 0 % (95 % timeouts) — not converged / unstable after 20 000; the checkpoint of 20 000 steps is published (package: 5/5 successes) |
| T-D-9 | Learning performance (before the alignment with the paper: camera 7.7 Hz, ROS 2 reset, 3-scenario parameters) | **PASS** *dev* | DreamerV3, task `shinjuku_box`, 30 009 steps / 312 episodes / 71 min: training success 2 % (0–5 k) → 31 % → 78 % → 92 % → 96 % → 100 % (25–30 k); evaluation without learning, 20 episodes each: 10 k 65 % success / 35 % collision, 20 k 95 % / 5 %, 30 009 100 % / 0 % (twice: 40/40). One seed; DREAMER_TRAINING.md §10 |
| T-D-11 | Task solvable / placement (`rl/tools/check_task.py`) | **PASS** *dev* | scripted driver with ground truth: 13/13 successes in 94–102 steps (limit 150); box poses from `/awsim/episode/obstacles` at map x −123.96…−123.04, y 32.48…33.09 (= 2.0–3.0 m ahead, ±0.4 m), yaw random; start (−126.0, 32.8), goal (−121.5, 32.8); box and red goal marker visible in the 64×64 observation |
| T-D-12 | First observation after a reset is current | **PASS** *dev* (after a fix) | before: after a 14 m teleport the first 64×64 observation still showed the old place (frame from the camera pipeline); after waiting for an image stamped after the reset: correct in 3/3 episodes after the same teleport |
| T-D-13 | Published model package | **PASS** *dev* | `export_checkpoint.py`: 462 → 156 MB (optimizer removed), archive 144 MB + SHA-256; `fetch_pretrained.sh` with a local archive (checksum verified); `evaluate.py --logdir pretrained/shinjuku_box_dreamerv3`: "244 parameters not in the checkpoint (opt): initialized", 5/5 successes. Download from the GitHub Release: **BLOCKED** until the release exists |
| T-D-14 | TensorBoard | **PASS** *dev* | `tensorboard --logdir logs/rl` in the dreamer image: 37 scalar tags incl. `eval/success_rate`; `eval/trajectory` images were not written (batched image, fixed; checked with `elements.Logger` directly, not in a training run) |
| T-D-10 | `rl/tools/real_whill_inference.py` | **BLOCKED** | needs a real WHILL; adapted to the agent interface, compiles |
| T-K-1 | Terminal keyboard driving (`teleop_keyboard.py`) | **PASS** *dev* | keys sent through a pseudo terminal: w×3 → cmd 0.3 m/s, odom 0.29 m/s; a×3 → ω 0.30 rad/s; d×5 → −0.20; space → 0; s×3 → −0.31 m/s; q → "stopped" (a crash at exit, "terminate called without an active exception", was fixed in this script and the other examples). Not tried by a person at a keyboard |
| T-O-1 | Optimizer: connection, rosbag loading | **PASS** *dev* | `test-grpc` connected; `test-rosbag`: 5 scenarios loaded (local copies of the real-WHILL recordings, not in the repository) |
| T-O-2 | Optimizer: runtime parameters | **PASS** *dev* | `get-params`; `apply-params` optimized YAML → `get-params` equal within float32 (`diff-params` no difference); restored |
| T-O-3 | Optimizer: optimization run | **PASS** *dev* | `optimize --trials 3 --scenarios straight` (open field): Optuna DB, results JSON, 3 plots; trials 0, 1 pruned, trial 2 score 2.32 (3 trials only — not a meaningful optimum) |
| T-OPT-4 | Re-evaluation and optimization in West Shinjuku | **PASS** *dev* | `batch-eval` (§4b); `optimize --trials 3` in West Shinjuku: exit 0, best 2.7209 (trials 0 and 2 pruned); `batch-eval` with one parameter set: exit 0 (segfault at exit fixed) |
| T-OPT-3 | Reproduction of the original study | **PASS** *dev* | `batch-eval` (§4): initial parameters 1.404 m vs original trial 0 1.411 m; optimized 0.106 m vs original best 0.090 m |
| T-P-1 | Physics unchanged by the placeholder | **PASS** | `WhillModelCRTools.ExportVehicleParameters` before (original prefab, private copy) / after: all AccelVehicle, Rigidbody, WheelCollider, BoxCollider and sensor-pose values identical; only the collider parent name differs. Dynamic confirmation: T-OPT-3 initial parameters reproduce the original trial 0 within 0.5 % |
| T-PY-1 | Python unit tests | **PASS** | `python3 -m pytest rl/tests optimizer/tests` in the `awsim-for-whill/dreamer` image: 31 passed |
| T-M-1 | Several WHILLs | **NOT IMPLEMENTED** | ROS2_INTERFACE.md §5 |
| T-N-1 | ns-3 integration | **NOT IMPLEMENTED** | design in NS3_INTEGRATION.md |
| T-X-1 | Unity Editor GUI opened by command (`scripts/open_editor.sh`) | **PASS** *dev* | Editor window "Unity - AWSIM_for_WHILL - AutowareSimulationDemo", hierarchy with `TaskObstacles`, West Shinjuku in the scene view; console "[EditorStartupScene] Opened Assets/Awsim/Scenes/AutowareSimulationDemo.unity" (Unity's `-openfile` left an "Untitled" scene, hence `EditorStartupScene.cs`) |
| T-X-4 | Play in the Editor, placing obstacles by hand | **NOT TESTED** | needs interaction with the GUI |
| T-X-2 | Keyboard driving in the AWSIM window | **PASS** *dev* | after the change of `AccelVehicleKeyboardInput` (velocity targets), key events sent to the player window (§8): Up 0.519 m/s, Up+Left 0.514 m/s / 0.502 rad/s, Down −0.492 m/s, Shift+Up 1.008 m/s, Right −0.493 rad/s, 0.000 after every release; `cmd_vel` ignored in Manual, C → 0.296 m/s with `cmd_vel` 0.3; `drive_test.py` 8/8 afterwards (after a reset: the keyboard run had left the WHILL against a shrub planter) |
| T-X-3b | G29 steering wheel | **NOT TESTED** | no device; same input path as the keyboard (acceleration/steering, ignored by the differential drive) |
| T-X-3 | Other upstream scenes (URP demo, scenario simulator, PCD generation) | **NOT TESTED** | unchanged upstream scenes, need the Shinjuku externals |

## 3. Measured interfaces

West Shinjuku (dev, default scene, 10 s, `check_interfaces.py`): `/clock` 99.1 Hz,
`/ground_truth/odom` 23.1 Hz, camera 7.73 Hz (1920×1080 bgr8), infrastructure LiDAR 10.0 Hz /
60 082 points / 1 441 968 B, vehicle LiDAR 10.0 Hz / 27 648 points, wheel speeds 39.0 Hz,
`/awsim/goal_position` 1.0 Hz (−135.0, 30.0, −7.499), `/tf_static` map → infrastructure_lidar_0
t = (−128.00, 30.00, −4.37).

Corridor scene (clean clone, 10 s):

| topic | type | msgs | rate [Hz] | frame_id | size [B] | points | stamp − /clock [s] |
|---|---|---|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | 993 | 99.16 | | | | |
| `/ground_truth/odom` | nav_msgs/Odometry | 234 | 23.35 | map → base_link | | | 0.021 |
| `/infrastructure/lidar_0/points` | sensor_msgs/PointCloud2 | 101 | 10.0 | infrastructure_lidar_0 | 1 035 552 | 43 148 | 0.0 |
| `/sensing/lidar/top/pointcloud_raw` | sensor_msgs/PointCloud2 | 101 | 10.0 | velodyne_top | 691 200 | 28 800 | 0.0 |
| `/sensing/lidar/top/pointcloud_raw_ex` | sensor_msgs/PointCloud2 | 101 | 10.0 | velodyne_top | 921 600 | 28 800 | 0.0 |
| `/sensing/camera/traffic_light/image_raw` | sensor_msgs/Image | 78 | 7.82 | traffic_light_left_camera/camera_link | 6 220 800 (1920×1080 bgr8) | | 0.021 |
| `/sensing/camera/traffic_light/camera_info` | sensor_msgs/CameraInfo | 78 | 7.82 | same | | | 0.0 |
| `/whill/wheel_velocities_sim` | std_msgs/Float32MultiArray | 383 | 38.23 | | | | |
| `/awsim/goal_position` | geometry_msgs/Point | 10 | 1.0 | | | | (−0.562, 4.463, −0.749) |
| `/tf_static` | tf2_msgs/TFMessage | 1 | latched | map → infrastructure_lidar_0, t = (−0.20, 6.30, 1.65) | | | |

## 4. Parameter re-evaluation (dev, WhillOpenField, real-WHILL recordings, 2026-10-15)

RMSE of the 2D position [m], one run each:

| Parameters | straight | turn | circle | sum (3) | s_curve | lemniscate |
|---|---|---|---|---|---|---|
| initial | 0.1583 | 0.4038 | 0.8422 | 1.404 | 0.4421 | 0.8528 |
| optimized 3scenario_v2 (new default) | 0.0494 | 0.0030 | 0.0537 | 0.106 | 0.1093 | 0.0943 |
| soturon scene | 0.0611 | 0.0062 | 0.0704 | 0.138 | 0.1299 | 0.1240 |
| original study, trial 0 / best | 0.1593 / – | 0.4058 / – | 0.8459 / – | 1.411 / 0.0899 | | |

## 4b. Parameter re-evaluation in West Shinjuku (dev, default scene, 2026-10-16)

RMSE of the 2D position [m], start pose of the original optimization (sidewalk with a
longitudinal slope of 2.1° and a cross slope of 0.9°):

| Parameters | straight | turn | circle | sum (3) | s_curve | lemniscate |
|---|---|---|---|---|---|---|
| initial | 1.548 | 0.7975 | 1.7207 | 4.066 | 0.6645 | 2.8478 |
| optimized 3scenario_v2 (default) | 0.0876 | 0.1185 | 0.1113 | 0.317 | 0.1166 | 0.1643 |
| soturon scene | 0.0815 | 0.2163 | 0.2206 | 0.518 | 0.1328 | 0.2617 |

The initial parameters drift sideways on the cross slope (3.07 m in the circle scenario); this is
the state before optimization.

Repeated after the cardboard box of the RL task was added to the scene (optimized parameters
only): straight 0.0998, turn 0.1197, circle 0.1118, s_curve 0.1101, lemniscate 0.1648 — within
0.012 m of the run above (real-time replay is not bit-exact); the straight run covers 10.0 m and
does not reach the box (16.5 m ahead, 2.8 m to the side).

## 5. Publication checks

| Check | Status | Evidence |
|---|---|---|
| Private WHILL 3D model not in the tree | **PASS** | no file of `Model Ci - FBX/` (list in ASSETS_AND_LICENSES.md §3); scan of all files for the 15 GUIDs of the private files: 0 hits |
| … nor in any commit | **PASS** | `git log --all --name-only` contains no private path; `git grep` of the private GUIDs over every commit: 0 hits; the history was created from scratch (first commit = upstream `fa1dc65`) |
| SLAM mesh, cardboard photos not included | **PASS** | same scans (paths, GUIDs, names `slam_map`, `IMG_676`) |
| No credentials | **PASS** | regex scan of all added lines of every commit (GitHub/OpenAI/AWS tokens, private keys, `password=`, `api_key=`, `secret=`, `token=`): 0 hits |
| No local paths / personal data | **PASS** with note | 0 hits for `/home/<user>` and `/media/` paths of the author, e-mail addresses or IPs in the added code; remaining: "Azumi Lab" in two copyright headers (to be confirmed by the author, ASSETS_AND_LICENSES.md §4) and an upstream `/home/takatokimakino/...` debug path in the upstream scenes |
| Upstream `docs/` not included | **PASS** | not in the tree nor in the history |
| Commit author e-mail | note | commits use the GitHub no-reply address of the account |

## 6. How to reproduce

```bash
scripts/setup.sh                                                     # T-S-1 (after a clone)
scripts/validate_project.sh                                          # T-U-*
BG=1 scripts/run_awsim.sh
scripts/ros2_shell.sh python3 ros2_examples/drive_test.py            # T-V-*
scripts/ros2_shell.sh python3 ros2_examples/check_interfaces.py      # T-R-*
scripts/ros2_shell.sh python3 ros2_examples/infra_lidar_check.py     # T-R-5
D="env IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh"
$D python3 rl/evaluate.py --agent random --set mode=constant --set "constant=[1.0,0.0]" --episodes 2   # T-D-2
$D python3 -u rl/train.py --agent dreamerv3 --logdir logs/rl/test --steps 400 --save-every 200 \
    --set loop.report_every=200                                      # T-D-3 (again for T-D-4 with --steps 600)
$D python3 rl/evaluate.py --logdir logs/rl/test --checkpoints all --episodes 2                   # T-D-5
$D python3 -m pytest -q rl/tests optimizer/tests                     # T-PY-1
# corridor player (WITH_CORRIDOR=1 scripts/setup.sh):
AWSIM_BIN=$PWD/Builds/AWSIM_for_WHILL_Corridor/AWSIM_for_WHILL.x86_64 BG=1 scripts/run_awsim.sh
scripts/ros2_shell.sh python3 ros2_examples/reset_collision_test.py  # T-E-1..4
```

## 7. Map download

2026-10-16, development machine: `curl -fL https://github.com/tier4/AWSIM/releases/download/v2.0.0/Shinjuku.unitypackage`
→ exit 0 in 113 s, 1 474 063 052 bytes, SHA-256 `fa1be2383899505ccfb4f966e10ce391b0f6ff122fa2cc13ded33874e7d359c9`, identical to the package imported in
T-S-1. The same URL is used by `scripts/fetch_shinjuku_map.sh`.

## 8. Keyboard of the AWSIM window (2026-10-16)

### Before the change (upstream keyboard input: acceleration / steering)

`cmd_vel` 0.3 m/s published at 10 Hz for 30 s (gear DRIVE), `/ground_truth/odom` logged; key
events sent to the player window (`XTestFakeKeyEvent` after `XSetInputFocus`) at fixed times.
Mean linear velocity per phase:

| Phase [s] | Input | Run 2 | Run 4 (fresh player) |
|---|---|---|---|
| 2–5 | none | 0.310 | 0.310 |
| 5–7 | Up arrow held | 0.010 | 0.000 |
| 8–12 | released | 0.000 | 0.000 |
| 13–17 | after C | 0.310 | 0.310 |
| 17–19 | Left arrow held | 0.010 | 0.007 |
| 20–24 | released | 0.000 | 0.000 |
| 25–30 | after C | 0.309 | 0.310 |

Explanation (code): `AccelVehicleControlModeBasedInputter` switches to `ControlMode.Manual` when a
manual input reports an override (arrow keys); in Manual it forwards the keyboard input, whose
`TargetLinearVelocity` / `TargetYawRate` are 0, and only `C` (`SwitchAutonomous`) switches back.
The Up arrow sets `AccelerationInput = 1`, which the differential drive does not use for
propulsion. In the first run (run 1, right after a player start), the WHILL did not move in the
first 5 s although no key had been sent, and the first C did not restore the control. This was
not reproduced in runs 3 (fresh start, no keys: 0.30 m/s from the first second) and 4; the cause
is not known.

### After the change (arrow keys → velocity targets)

No `cmd_vel` publisher; keys sent to the window of the rebuilt player; mean values per phase:

| Phase [s] | Keys held | v [m/s] | ω [rad/s] |
|---|---|---|---|
| 0.5–3 | none | 0.000 | 0.000 |
| 4–6 | Up | +0.519 | −0.002 |
| 6.8–8 | none | 0.000 | 0.000 |
| 9–11 | Up + Left | +0.514 | +0.502 |
| 11.8–13 | none | 0.000 | 0.000 |
| 14–16 | Down | −0.492 | +0.007 |
| 16.8–18 | none | 0.000 | 0.000 |
| 19–21 | Left Shift + Up | +1.008 | +0.005 |
| 21.8–23 | none | 0.000 | 0.000 |
| 24–26 | Right | +0.010 | −0.493 |
| 27–30 | none | 0.000 | 0.000 |

With `cmd_vel` 0.3 m/s published (AWSIM still in Manual from the run above): 0.000 m/s
(`cmd_vel` ignored), Left held → ω +0.497 rad/s, released → 0.000, C → v 0.296 m/s.

Gear keys in Manual: P, then Up held → 0.000 m/s; D, then Up held → 0.513 m/s; released → 0.000.

## 9. ROS 2 settings aligned with the paper (Table III, Sec. III-C), 2026-10-16

| Item | Before | After | Evidence |
|---|---|---|---|
| Camera topic | `/sensing/camera/traffic_light/image_raw` | `/sensing/camera/front/image_raw` (+ `camera_info`) | `check_interfaces.py` |
| Camera rate | 7.7 Hz (render, then wait 1/10 s) | **10.0 Hz** (absolute schedule in `CameraSensorScheduler`) | 100 messages in 10 s |
| Odometry rate | 23.1 Hz (setting 30, remainder dropped) | **20.0 Hz** (setting 20, remainder kept) | 201 messages in 10 s |
| Collision subscriber queue | 1 | 10 | `data_manager.py` |
| Odometry stamp | used to find the first frame after a reset | not used; the reset answer carries the simulation time | `data_manager.py` |
| `cmd_vel` | one per step at 7.7 Hz | one per step at 10 Hz | step = camera frame |
| Episode reset | ROS 2 topics `/reset_command` → `/reset_response` | synchronous TCP request `reset_episode` (OptimizerBridge) | player log: 10 resets, 0 `/reset_command` |

Checks after the change (dev): `drive_test.py` 8/8; `check_task.py` 10/10 successes, now 120–129
steps (94–102 at 7.7 Hz; limit 150); `batch-eval` of the optimized parameters with 20 Hz odometry:
straight 0.1036, turn 0.1192, circle 0.1090, s_curve 0.1155, lemniscate 0.1647 m (within the
run-to-run variation of §4b); the model trained at 7.7 Hz evaluated at 10 Hz: 8/10 successes,
2 timeouts (100 % at 7.7 Hz) → retrained (DREAMER_TRAINING.md §10).

## 10. Default vehicle parameters = 4-scenario optimum (paper Table II), 2026-10-16

`WhillModelCR.prefab` set to `config/vehicle/whill_model_cr_optimized_4scenario_v2.yaml` (values
of Table II of the paper); `get-params` from the rebuilt player returns them (float32). The
initial values (`whill_model_cr_initial.yaml`, search-space defaults) use `rolling_resistance`
0.002 as in Table II (before: 0.02, as in trial 0 of the studies).

`batch-eval`, WhillOpenField, one run each — trajectory RMSE [m]:

| Parameters | straight | turn | circle | s_curve | figure-eight |
|---|---|---|---|---|---|
| 4-scenario (default) | 0.0631 | 0.0025 | 0.0369 | 0.1095 | 0.1092 |
| paper Table IV, optimized | 0.058 | 0.002 | 0.037 | 0.112 | 0.111 |
| initial, μ_r 0.002 | 0.0614 | 0.2468 | 1.1089 | 0.5849 | 1.3204 |
| initial, μ_r 0.02 (§4) | 0.1583 | 0.4038 | 0.8422 | 0.4421 | 0.8528 |
| paper Table IV, initial | 0.166 | 0.406 | 0.846 | 0.446 | 0.853 |
| 3-scenario | 0.0543 | 0.0026 | 0.0486 | 0.1081 | 0.0909 |
| soturon scene | 0.0609 | 0.0063 | 0.0678 | 0.1308 | 0.1293 |

West Shinjuku with the new default: `drive_test.py` 8/8 (forward +1.205 m, turn +82.7°),
`check_task.py` 5/5 successes (120–128 steps).

## 11. Resource measurement (2026-10-16)

`scripts/bench_resources.sh`, 9 conditions (1/2/4/8/16 infrastructure LiDARs with 1 WHILL, 1/2/4/8/16
WHILLs with 1 LiDAR), vehicle LiDAR off, exit 0 (~25 min). Every LiDAR at 10 Hz, every camera
published (≥ 9.0 Hz), every WHILL turning in every condition. Results: RESOURCE_USAGE.md.
