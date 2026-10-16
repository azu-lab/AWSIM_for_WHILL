# WHILL Model CR vehicle model

This document describes the simulated WHILL Model CR: the prefab, the drive and physics
model, the parameters actually in effect, and the public placeholder that replaces the
original 3D model.

All values below were read from the project (prefab / scene YAML and C# source) or exported
with `WhillModelCRTools.ExportVehicleParameters`; nothing is estimated.

## 1. Prefab

`Assets/Awsim/Prefabs/Entity/EgoVehicle/WhillModelCR/WhillModelCR.prefab`
(GUID `eae4e7b9df617648cbb0b9cbbe115bca`, inherited from the original `ModelCi.prefab` so that
every scene reference stays valid).

```
WhillModelCR                      Rigidbody, AccelVehicle, AccelVehicleReportRos2Publisher,
│                                 AccelVehicleControlModeSrvServer, EgoVehicle,
│                                 OptimizerBridge, CollisionPublisher, WheelSpeedPublisher
├── Inputs                        ControlModeBasedInputter, Keyboard / ROS 2 / G29 inputs, PlayerInput
├── CameraSensorScheduler
│   └── CameraSensor              CameraSensor, CameraRos2Publisher
│       └── Camera
├── Colliders                     ── physics only ──
│   ├── WheelFL, WheelFR          WheelCollider (front casters)
│   ├── WheelRL, WheelRR          WheelCollider (drive wheels)
│   └── BodyCollider              BoxCollider
├── VisualRoot                    ── visual only, no colliders ──
│   ├── WheelVisualFL/FR/RL/RR    moved every frame by AccelVehicle
│   │   └── Tire                  Unity Cylinder
│   └── PublicPlaceholderModel    Unity Cubes (chassis, seat, backrest, armrests, controller, footrest)
├── VelodyneVLP16                 LidarSensor, RglLidarPublisher (vehicle-mounted LiDAR)
├── tamagawa/imu_link/ImuSensor   ImuSensor, ImuRos2Publisher (not initialized, see §6)
└── RealAxle_center               GroundTruthPublisher  (= base_link)
```

Mapping to the structure recommended for this project:

| Recommended | In the prefab |
|---|---|
| `PhysicsRoot` + `Rigidbody` | prefab root `WhillModelCR` (AccelVehicle requires the Rigidbody on the same GameObject) |
| `Colliders` | `Colliders` |
| `DriveController` | `AccelVehicle` on the root, inputs under `Inputs` |
| `Sensors` | `CameraSensorScheduler`, `VelodyneVLP16`, `tamagawa/imu_link`, `RealAxle_center` (kept at their original paths so that the transforms are unchanged) |
| `VisualRoot/PublicPlaceholderModel` | `VisualRoot/PublicPlaceholderModel` |

Changing anything under `VisualRoot` cannot change the physics: the visual nodes have no
collider, and AccelVehicle only writes the pose of the `WheelVisual*` transforms.

## 2. Geometry (vehicle frame, Unity axes: x right, y up, z forward; origin = prefab root)

| Item | Value | Source |
|---|---|---|
| Drive wheels (rear) | WheelRL / WheelRR at (∓0.248, 0.19, −0.30) m, radius **0.1345 m** | WheelCollider |
| Casters (front) | WheelFL / WheelFR at (∓0.248, 0.19, +0.30) m, radius **0.13 m** | WheelCollider |
| Track width (wheel distance) | **0.496 m** (geometry and `AccelVehicle._treadWidth`) | |
| Wheelbase | 0.60 m | |
| Suspension | spring 5000 N/m, damper 500 N·s/m, distance 0.1 m, target 0.5 | WheelCollider |
| Body collider | box 0.55 × 0.80 × 0.985 m (W × H × L), center y = 0.444 m | BoxCollider |
| base_link | `RealAxle_center` at (0, 0.19, −0.30): center of the drive axle, at wheel-center height | GroundTruthPublisher |
| Camera | (0.326, 0.916, 0.287) m, pitch 15.7° down, 1920×1080 | CameraSensor |
| Vehicle LiDAR (VLP16) | (−0.014, 0.383, 0.361) m | LidarSensor |

## 3. Drive model (`Assets/Awsim/Scripts/Entity/Vehicle/AccelVehicle/AccelVehicle.cs`)

Upstream AWSIM models an Ackermann car driven by an acceleration command. soturon replaced it
by a differential drive driven by a velocity command:

1. **Command.** `geometry_msgs/Twist` on `/whill/controller/cmd_vel`:
   `linear.x` = v [m/s], `angular.z` = ω [rad/s] (ROS convention: positive = counter-clockwise =
   left turn; verified by `ros2_examples/drive_test.py`). Other fields are ignored.
2. **Gear.** Velocity control is active only in gear DRIVE, REVERSE or NEUTRAL. The vehicle starts in
   PARKING; send `autoware_vehicle_msgs/GearCommand` `DRIVE (2)` on `/control/command/gear_cmd`
   (the sign of v selects forward/backward, REVERSE is not needed).
   `EnvironmentReset` also sets DRIVE on every reset.
3. **Inverse kinematics** (every FixedUpdate):
   `v_L = v − (d/2)·ω`, `v_R = v + (d/2)·ω`, d = `_treadWidth` = 0.496 m.
4. **Per-wheel PID** on the drive wheels: error e = target − measured ground speed of the wheel
   contact point; `a = Kp·e + Ki·∫e + Kd·de/dt` with ∫e clamped to ±`_wheelIntegralClamp` and
   de/dt clamped to ±50; drive force `F = a · m/2`, clamped to ±200 N
   (`_maxForcePerWheel`, a constant), applied at the contact point along the wheel forward
   direction multiplied by `_forwardSlipMultiplier`.
5. **Lateral resistance** on every wheel: `F_lat = −c · v_lat · sprungMass`, with
   c = `_skiddingCancelRate` (drive wheels) or `_frontWheelSkiddingRate` (casters).
6. **Drag**: rolling resistance `Crr·m·g` and air drag `½·ρ·Cd·A·v²` against the velocity
   (`_rollingResistance`, `_airDrag`, `_frontalArea`, ρ = 1.225 kg/m³), plus a strong
   restoring force above 1.1 × `_maxSpeed`.
7. **Steering** is fixed to 0 (`SteerTireAngleInput` is ignored).

`AccelVehicle.Initialize()` overwrites three Rigidbody values: **mass = 55 kg**
(the serialized Rigidbody mass of 50 kg is never used), `maxAngularVelocity = 30`,
`angularDamping = 0`.

Note: a tooltip in AccelVehicle says "TargetYawRate: + = right turn". The measured behaviour is
the ROS convention (+ = left turn); the tooltip is wrong (kept unchanged, as it is upstream code
of soturon).

## 4. Parameters in effect (default prefab)

Exported from the prefab used by `Assets/Awsim/Scenes/AutowareSimulationDemo.unity`.
Runtime-tunable values can be read from a running AWSIM with
`python3 -m whill_optimizer get-params` and changed with `apply-params`
(see [PARAMETER_OPTIMIZATION.md](PARAMETER_OPTIMIZATION.md)).

| Parameter (OptimizerBridge name) | AccelVehicle field | Value | Origin |
|---|---|---|---|
| skidding_cancel_rate | `_skiddingCancelRate` | 34.790068 | 4scenario_v2 (3scenario_v2: 33.554504; soturon scene: 10.24398) |
| front_wheel_skidding_rate | `_frontWheelSkiddingRate` | 0.40560326 | 4scenario_v2 (3scenario_v2: 1.1052157; soturon scene: 0.14087) |
| forward_slip_multiplier | `_forwardSlipMultiplier` | 0.84004636 | 4scenario_v2 |
| wheel_kp | `_wheelKp` | 27.699972 | 4scenario_v2 |
| wheel_ki | `_wheelKi` | 0.40250112 | 4scenario_v2 |
| wheel_kd | `_wheelKd` | 0.73839092 | 4scenario_v2 |
| wheel_integral_clamp | `_wheelIntegralClamp` | 1.4313239 | 4scenario_v2 |
| air_drag | `_airDrag` | 1.1324312 | 4scenario_v2 |
| rolling_resistance | `_rollingResistance` | 0.0033877346 | 4scenario_v2 |
| frontal_area | `_frontalArea` | 1.0614818 | 4scenario_v2 |
| com_offset_x/y/z | `Rigidbody.centerOfMass` (Unity axes: x right, y up, z forward) | (0.0044574, 0.1215417, −0.2654788) | 4scenario_v2 |
| – | `_treadWidth` | 0.496 m | measured geometry |
| – | `_maxSpeed` | 1.667 m/s (6 km/h) | |
| – | mass | 55 kg (code) | |

The default is **exactly the best result of the 4-scenario optimization** (straight, turn,
circle, S-curve; `config/vehicle/whill_model_cr_optimized_4scenario_v2.yaml`), the optimized
values of Table II of the paper (arXiv:2609.28969). The best result of the 3-scenario
optimization is kept as `whill_model_cr_optimized_3scenario_v2.yaml`. The paper lists the center
of mass in the vehicle frame of ROS (CoM_x front = Unity z, CoM_y left = Unity −x, CoM_z up =
Unity y); its CoM_y (+0.0044 m, "left") is the value of Unity x (+0.0045 m, i.e. right).

The scene of soturon differed from both: in its
commit `d72b3a3` (2026-02-10) only 11 of the 13 optimized values were entered in the scene;
`skidding_cancel_rate` and `front_wheel_skidding_rate` kept the values of an older study
(`saitekika.yaml`: 10.24398 / 0.14087), and that state was used for every later experiment of
soturon (including the DreamerV3 training). Re-evaluated against the real-WHILL recordings, the
complete optimized set is closer to the real WHILL in all five scenarios
(PARAMETER_OPTIMIZATION.md §6), so it is the default here. To reproduce soturon:

```bash
python3 -m whill_optimizer apply-params --yaml ../config/vehicle/whill_model_cr_soturon_scene.yaml
```

Serialized values that have **no effect** (left by older versions of AccelVehicle, removed from
the prefab when the scene overrides were applied): `_vehicleMass`, `_maxForcePerWheel`
(now a constant 200 N), `_lateralDampingFactor`, `_sidewaySlipMultiplier`,
`_frontWheelSideslipMultiplier`.

## 5. Public placeholder model

The 3D model of the WHILL Model CR used in soturon (`Model Ci - FBX/`: FBX meshes and
textures) cannot be published. It was replaced by Unity primitives generated from the physics
components:

- `PublicPlaceholderModel`: chassis, seat post, seat cushion, backrest, armrests, joystick box,
  footrest and a front marker, sized from `BodyCollider`;
- one `Tire` cylinder per wheel, sized from the WheelCollider radius;
- four materials in `WhillModelCR/Materials/` (HDRP Lit, plain colors).

Rebuild it (e.g. after changing the colliders) with the menu
**AWSIM for WHILL > Rebuild WHILL Model CR placeholder model**, or in batchmode
`-executeMethod Awsim.WhillModelCR.Editor.WhillModelCRTools.RebuildPlaceholderBatch`.
Every primitive has its collider removed by the tool.

What changes with the placeholder:

- **Physics: nothing.** Exported parameters before (original model) and after (placeholder)
  are identical; the only difference is the name of the collider parent
  (`Collider` → `Colliders`). See TEST_REPORT.md.
- **Sensors that see the WHILL itself:** the infrastructure LiDAR and other cameras now see
  boxes instead of the WHILL mesh. RGL ray-casts against renderers, so the shape of the point
  cloud of the WHILL differs from soturon. The vehicle camera looks forward and down
  (15.7°) from 0.92 m and does not see the placeholder.

How it was derived (one-off, in a private working copy, not reproducible without the original
model): the original prefab was moved (GUID preserved), the scene overrides of the instance were
applied to the prefab, the original mesh nodes were deleted and the placeholder was built.
The original FBX, textures and prefab never entered the history of this repository.

## 6. Components disabled in soturon

`Assets/Awsim/Scenes/AutowareSimulationDemo/EgoVehicle.cs` comments out the initialization
of the IMU, the GNSS and the vehicle visual effects. The IMU publisher component is still in the
prefab but is never initialized, so `/sensing/imu/tamagawa/imu_raw` is **not** published.
