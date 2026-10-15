# Test report

Status values: **PASS** = executed and succeeded; **FAIL** = executed and failed;
**BLOCKED** = could not be executed in this environment; **NOT IMPLEMENTED** = feature does not
exist; **NOT TESTED** = exists but was not executed. Reading the code alone never counts as PASS.

## 1. Environment

| | |
|---|---|
| Date | 2026-10-15 |
| Machine | Ubuntu 22.04.5 LTS, AMD Ryzen Threadripper 9960X, 256 GB RAM, NVIDIA RTX 5090 32 GB (driver 580.178.04), X display :1 |
| Unity | 6000.0.34f1, batchmode (`scripts/unity_lib.sh`), Unity Hub signed in |
| AWSIM | standalone Linux player built by `scripts/build_player.sh` (window 1280×720) |
| ROS 2 | Humble, `rmw_cyclonedds_cpp`, `config/cyclonedds.xml`, `ROS_DOMAIN_ID=77`, Docker images `awsim-for-whill/ros2`, `awsim-for-whill/dreamer` |
| Clean environment | `git clone --recurse-submodules` of the committed repository into a new directory (`clean_clone/`), **no `Library/` copied** (fresh import), Docker images built from that clone with `docker compose`. Neither the soturon working copy nor any private asset is reachable from it |

Two working trees were used: the development tree (tests marked *dev*) and the clean clone
(tests marked *clean*). The clean clone was at commit `c879702`; later commits change only
documentation, tests, the default of two vehicle parameters (re-tested *dev*, T-OPT-3/T-V-5) and
tools.

## 2. Results (section 9.1 of the task)

| ID | Item | Status | Evidence |
|---|---|---|---|
| T-U-1 | Unity project opens (import, compile) | **PASS** *clean* | `scripts/validate_project.sh` in the clean clone: fresh import, 0 compile errors (`error CS` absent), exit 0 |
| T-U-2 | No missing script | **PASS** *clean* | validator: 0 missing scripts on 1 712 active objects; `WhillModelCR.prefab`: missing scripts = 0 |
| T-U-3 | No missing asset | **PASS** for active objects *clean*; see note | active objects: 0 broken references, 0 renderers without mesh/material, 0 missing prefabs. Inactive `Environments/Shinjuku` (upstream, disabled): 6 741 broken references, 1 967 meshes/materials missing — they belong to the AWSIM Externals (Shinjuku map, not redistributable) and were in the same state upstream without externals |
| T-U-4 | Scene loads | **PASS** *clean* | validator opens `AutowareSimulationDemo.unity`; player loads it |
| T-U-5 | AWSIM starts | **PASS** *clean* | `scripts/build_player.sh` exit 0; `BG=1 scripts/run_awsim.sh` → "Ready." (OptimizerBridge on 127.0.0.1:50051) |
| T-U-6 | WHILL is displayed | **PASS** *dev* | `docs/images/whill_model_cr_placeholder.png` rendered by `WhillSceneTools.CapturePreviewBatch` (placeholder + VLP16 visible). In the player window the follow camera is behind the corridor wall (known limitation) |
| T-V-1 | WHILL moves forward | **PASS** *clean* | `drive_test.py`: +0.40 m/s × 3 s → +1.164 m along the heading, lateral −0.002 m, v_end 0.399 m/s |
| T-V-2 | WHILL turns | **PASS** *clean* | ±0.5 rad/s × 3 s → +83.2° / −83.1° (commanded 86°), translation < 0.01 m |
| T-V-3 | WHILL stops | **PASS** *clean* | after each stop phase \|v\| = 0.000 m/s, \|ω\| = 0.000 rad/s |
| T-V-4 | WHILL moves backward | **PASS** *clean* | −0.30 m/s × 3 s → −0.866 m |
| T-V-5 | Same with the final default parameters | **PASS** *dev* | player rebuilt at `86a234c`+; `get-params`: skidding_cancel_rate 33.5545, front_wheel_skidding_rate 1.10522; `drive_test.py` 8/8 (forward +1.171 m, turns +81.1° / −80.7°) |
| T-R-1 | ROS 2 velocity command received | **PASS** *clean* | T-V-1..4; topic info: subscriber node `AWSIM`, TRANSIENT_LOCAL |
| T-R-2 | QoS: VOLATILE publisher not matched, TRANSIENT_LOCAL matched | **PASS** *dev* | rclpy VOLATILE publisher: "requesting incompatible QoS … DURABILITY", 0.000 m moved; TRANSIENT_LOCAL: 0.866 m. `ros2 topic pub` without flags also moves the WHILL (Humble default transient_local) |
| T-R-3 | Odometry obtained | **PASS** *clean* | `/ground_truth/odom` 23.35 Hz, `map → base_link` |
| T-R-4 | Infrastructure LiDAR point cloud obtained | **PASS** *clean* | `/infrastructure/lidar_0/points` 10.0 Hz, 43 148 points, 1 035 552 B; `/tf_static` map → infrastructure_lidar_0 |
| T-R-5 | Infrastructure LiDAR sees the WHILL | **PASS** *clean* | `infra_lidar_check.py`: median 275 points in the box around the WHILL at 6.0 m (threshold 20) |
| T-R-6 | All interfaces | **PASS** *clean* | `check_interfaces.py` table (§3) |
| T-R-7 | `/clock`, stamps on the same time base | **PASS** *clean* | /clock 99.2 Hz; stamp − /clock: odom 0.021 s, image 0.021 s, clouds 0.000 s |
| T-R-8 | Pause / resume behaviour of time | **NOT TESTED** | described from the code in ROS2_INTERFACE.md §4 |
| T-E-1 | Collision with obstacle detected | **PASS** *clean* | `reset_collision_test.py`: `/collision_sensor` after 5.5 s, 2.18 m |
| T-E-2 | Reset | **PASS** *clean* | `/reset_response` received, distance to start pose 0.000 m |
| T-E-3 | Start pose / goal over ROS 2 | **PASS** *clean* | commanded (−0.030, 0.106, 124.9°) → odom error 0.000 m / 0.00°; goal echo on `/awsim/goal_position` |
| T-E-4 | Goal position available to late subscribers | **PASS** *clean* | `/awsim/goal_position` 1.0 Hz (was published once before the fix) |
| T-D-1 | DreamerV3 environment initializes | **PASS** *clean* | `random_agent.py`: observation `image (64,64,3) uint8`, goal received |
| T-D-2 | At least one episode | **PASS** *clean* | random actions: 238 steps, return 34.16, truncated at 30.4 s; *dev* (final): constant forward → collision after 62 / 70 steps, "Vehicle returned to start (dist: 0.000m)" after each reset |
| T-D-3 | DreamerV3 training step | **PASS** *clean* | `train.py --steps 100`: JIT compile (float32 on RTX 5090), training loop, checkpoint at step 100 |
| T-D-4 | Short training, checkpoint save | **PASS** *dev* | 400 steps, checkpoints 200 / 400 / final (≈462 MB each), TensorBoard events, metrics.jsonl |
| T-D-5 | Checkpoint restore / resume | **PASS** *dev* | second run with the same logdir: "Loading existing checkpoint … checkpoint_final.pkl", "Resuming from step 400", trained to 500, report generated |
| T-D-6 | Evaluation of a checkpoint (`eval.py`) | **PASS** *clean* | `eval.py --checkpoints 100 --episodes 1`: checkpoint loaded, 1 episode (collision after 123 steps, reward −26.7 — expected after 100 training steps), summary table and CSV written |
| T-D-7 | Learning performance (policy reaches the goal) | **NOT TESTED** | requires ≥ 10⁵–10⁶ steps (days); no long run was made |
| T-D-8 | `real_whill_inference.py` | **BLOCKED** | needs a real WHILL |
| T-O-1 | Optimizer: connection, rosbag loading | **PASS** *dev* | `test-grpc` connected; `test-rosbag`: 5 scenarios loaded (local copies of the real-WHILL recordings, not in the repository) |
| T-O-2 | Optimizer: runtime parameters | **PASS** *dev* | `get-params`; `apply-params` optimized YAML → `get-params` equal within float32 (`diff-params` no difference); restored |
| T-O-3 | Optimizer: optimization run | **PASS** *dev* | `optimize --trials 3 --scenarios straight` (open field): Optuna DB, results JSON, 3 plots; trials 0, 1 pruned, trial 2 score 2.32 (3 trials only — not a meaningful optimum) |
| T-OPT-3 | Reproduction of the original study | **PASS** *dev* | `batch-eval` (§4): initial parameters 1.404 m vs original trial 0 1.411 m; optimized 0.106 m vs original best 0.090 m |
| T-P-1 | Physics unchanged by the placeholder | **PASS** | `WhillModelCRTools.ExportVehicleParameters` before (original prefab, private copy) / after: all AccelVehicle, Rigidbody, WheelCollider, BoxCollider and sensor-pose values identical; only the collider parent name differs. Dynamic confirmation: T-OPT-3 initial parameters reproduce the original trial 0 within 0.5 % |
| T-PY-1 | Python unit tests | **PASS** | `python3 -m pytest rl/tests optimizer/tests`: 18 passed |
| T-M-1 | Several WHILLs | **NOT IMPLEMENTED** | ROS2_INTERFACE.md §5 |
| T-N-1 | ns-3 integration | **NOT IMPLEMENTED** | design in NS3_INTEGRATION.md |
| T-X-1 | Unity Editor GUI (open + Play button) | **NOT TESTED** | batchmode import/validation and the standalone player were used instead |
| T-X-2 | Keyboard / G29 control | **NOT TESTED** | keyboard does not drive the WHILL by design of the modified AccelVehicle (code) |
| T-X-3 | Other upstream scenes (URP demo, scenario simulator, PCD generation) | **NOT TESTED** | unchanged upstream scenes, need the Shinjuku externals |

## 3. Measured interfaces (clean clone, corridor scene, 10 s)

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
scripts/validate_project.sh                       # T-U-1..4
scripts/build_player.sh && BG=1 scripts/run_awsim.sh
scripts/ros2_shell.sh python3 ros2_examples/drive_test.py            # T-V-*
scripts/ros2_shell.sh python3 ros2_examples/check_interfaces.py      # T-R-*
scripts/ros2_shell.sh python3 ros2_examples/infra_lidar_check.py     # T-R-5
scripts/ros2_shell.sh python3 ros2_examples/reset_collision_test.py  # T-E-*
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 rl/scripts/random_agent.py --episodes 1
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 -u rl/scripts/train.py \
    --task obstacle_avoidance --logdir logs/dreamer/test --steps 100 --save-every 100
IMAGE=awsim-for-whill/dreamer scripts/ros2_shell.sh python3 -m pytest -q rl/tests optimizer/tests
```
