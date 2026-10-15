# Vehicle parameter optimization

## 1. Purpose

Make the simulated WHILL Model CR move like a real one: the same velocity commands sent to a real
WHILL and to AWSIM should give the same trajectories. The parameters of the differential-drive
model (friction, PID, drag, center of mass) are searched automatically against recordings of a
real WHILL. Origin: soturon `Optimaizer/` (`e91b33f`), ported to `optimizer/whill_optimizer`.

## 2. Parameters

13 parameters, all applied at runtime by `OptimizerBridge.set_parameters`
(`Assets/Awsim/Scripts/Optimizer/OptimizerBridge.cs`):

| Name | AccelVehicle / Rigidbody | Search range | Meaning |
|---|---|---|---|
| `skidding_cancel_rate` | `_skiddingCancelRate` | 0 – 50 | lateral damping of the drive wheels |
| `front_wheel_skidding_rate` | `_frontWheelSkiddingRate` | 0 – 10 | lateral damping of the casters |
| `forward_slip_multiplier` | `_forwardSlipMultiplier` | 0.5 – 1.0 | scale of the drive force |
| `wheel_kp`, `wheel_ki`, `wheel_kd` | `_wheelKp`, `_wheelKi`, `_wheelKd` | 0–30, 0–1, 0–2 | per-wheel speed PID |
| `wheel_integral_clamp` | `_wheelIntegralClamp` | 0.1 – 3.0 | anti-windup |
| `air_drag`, `rolling_resistance`, `frontal_area` | `_airDrag`, `_rollingResistance`, `_frontalArea` | 0.1–1.5, 0–0.05, 0.4–1.2 | drag |
| `com_offset_x/y/z` | `Rigidbody.centerOfMass` | ±0.25, ±0.3, ±0.3 m | center of mass (Unity axes) |

Not optimized (fixed): mass 55 kg (set in code), track width 0.496 m, wheel radii (0.1345 m drive,
0.13 m caster), max. force 200 N per wheel (constant), max. speed 1.667 m/s.
Older soturon studies also searched `vehicle_mass`, `max_force_per_wheel`,
`lateral_damping_factor`, `acceleration_time_constant` …; these no longer exist in AccelVehicle
and are not part of the current search space.

Search space file: [`config/optimizer/search_space.yaml`](../config/optimizer/search_space.yaml).
Its ranges are the ones **stored in the Optuna storage of the study that produced the published
optimum** (`trial_params.distribution_json` of `3scenario_v2.db`). The code default
(`DEFAULT_PARAMETER_RANGES`, unchanged from soturon `e91b33f`) differs in the center of mass
(`com_offset_x` ±0.2, `com_offset_y` 0.1–0.6, default 0.3): it was changed after that study.

## 3. Method

- **Algorithm**: Optuna, TPE sampler (Optuna default), minimize, SQLite storage (resumable).
  Trial 0 is the `default` column (= `config/vehicle/whill_model_cr_initial.yaml`).
- **Input data**: rosbag2 recordings of a real WHILL driven by `cmd_vel`
  (`/whill/controller/cmd_vel` Twist and `/whill/odom` Odometry; recorded with
  `optimizer/control_whill.py`). Scenario names are mapped to directories in
  `whill_optimizer/rosbag_loader.py`: `straight`, `turn`, `circle`, `s_curve`, `lemniscate`.
- **One evaluation**: `OptimizerBridge.run_scenario` applies the parameters and resets the WHILL to
  its start pose; the recorded `cmd_vel` sequence is replayed over ROS 2 in real time (gear DRIVE,
  2 s settle, 1 s before start); `/ground_truth/odom` is recorded.
- **Objective**: per scenario, both trajectories are expressed in the frame of their own initial pose,
  time-aligned at their first sample, resampled to the lower rate over the common duration, and the
  RMSE of the 2D position is computed (`compare_bags.compute_rmse_metrics`). Score = sum over the
  scenarios. A failed scenario costs 10; a trial is **pruned** when the running sum exceeds 3.0.
- Simulation runs at time scale 1.0 (the replay is in wall-clock time).

## 4. Environment

AWSIM must run a scene **without obstacles**: the scenarios need several metres of free space
(straight: 4.6 m). Use `Assets/Awsim/Scenes/WhillOpenField.unity` (generated from the corridor scene
by `scripts/setup_open_field_scene.sh`: corridor and obstacles removed, flat 200 m × 200 m ground at
the same height). In the corridor scene a straight run stopped at the cardboard box after 2.3 m.
soturon ran its studies before the corridor existed, on the Shinjuku map (not redistributable).

Python: the ROS 2 tools image (`docker/ros2`, `optimizer/requirements.txt`: optuna 5.0.0,
rosbags 0.11.7, numpy 1.26.4, …).

## 5. Commands

```bash
# AWSIM with the open field
AWSIM_BUILD_SCENE=Assets/Awsim/Scenes/WhillOpenField.unity \
AWSIM_BUILD_OUTPUT=$PWD/Builds/AWSIM_for_WHILL_OpenField/AWSIM_for_WHILL.x86_64 scripts/build_player.sh
AWSIM_BIN=$PWD/Builds/AWSIM_for_WHILL_OpenField/AWSIM_for_WHILL.x86_64 scripts/run_awsim.sh

# recordings of a real WHILL (directory names as in rosbag_loader.py)
export WHILL_ROSBAG_DIR=$PWD/data/real_whill
scripts/ros2_shell.sh bash -c "cd optimizer && python3 -m whill_optimizer test-rosbag"
scripts/ros2_shell.sh bash -c "cd optimizer && python3 -m whill_optimizer test-grpc"

# optimization (results in outputs/, ignored by Git)
scripts/ros2_shell.sh bash -c "cd optimizer && python3 -m whill_optimizer optimize --trials 500 \
    --scenarios straight,turn,circle \
    --db-path /workspace/outputs/opt/study.db --results-json /workspace/outputs/opt/results.json \
    --output-dir /workspace/outputs/opt/visualization"
scripts/ros2_shell.sh bash -c "cd optimizer && python3 -m whill_optimizer dashboard \
    --db-path /workspace/outputs/opt/study.db --results-json /workspace/outputs/opt/results.json"

# evaluation of parameter files (every YAML of a directory x every scenario, CSV/PDF/JSON)
scripts/ros2_shell.sh bash -c "cd optimizer && python3 -m whill_optimizer batch-eval \
    --param-dir /workspace/config/vehicle --output-prefix /workspace/outputs/batch_eval/run"

# statistical comparison (interactive, n runs each, t-test, PDF)
scripts/ros2_shell.sh bash -c "cd optimizer && python3 -m whill_optimizer ttest"
```

`WHILL_ROSBAG_DIR` must point to a path inside the repository when the Docker shell is used
(the repository is the only mounted directory).

## 6. Results

### Original study (soturon, January 2026)

| Study | Scenarios | Trials | Best | Score (sum of RMSE) |
|---|---|---|---|---|
| `3scenario_v2` | straight, turn, circle | 500 (488 complete, 12 pruned) | #426 | **0.0899 m** (trial 0: 1.411 m) |
| `4scenario_v2` | + s_curve | 500 (496 / 4) | #392 | 0.1997 m (trial 0: 1.852 m) |
| `straight_v2` | straight | 500 | #435 | 0.0346 m (trial 0: 0.164 m) |

The best of `3scenario_v2` is [`config/vehicle/whill_model_cr_optimized_3scenario_v2.yaml`](../config/vehicle/whill_model_cr_optimized_3scenario_v2.yaml)
(checked to be identical to `best_params` of the study). The other runs are in
`config/vehicle/optimization_runs/`. The study databases are not published (they contain the
real-WHILL trajectories).

### Re-evaluation in this repository (placeholder model, WhillOpenField, 2026-10-15)

`batch-eval` with the original real-WHILL recordings (kept locally, not published), one run per
parameter set and scenario, standalone player, RTX 5090 (log: TEST_REPORT.md T-OPT-3):

| Parameter set | straight | turn | circle | **sum (3 scenarios)** | s_curve | lemniscate |
|---|---|---|---|---|---|---|
| initial (`whill_model_cr_initial.yaml`) | 0.1583 | 0.4038 | 0.8422 | **1.404** | 0.4421 | 0.8528 |
| original trial 0 of `3scenario_v2` (soturon) | 0.1593 | 0.4058 | 0.8459 | 1.411 | – | – |
| optimized (`..._optimized_3scenario_v2.yaml`) | 0.0494 | 0.0030 | 0.0537 | **0.106** | 0.1093 | 0.0943 |
| original best of `3scenario_v2` (soturon) | – | – | – | 0.0899 | – | – |
| soturon scene (`..._soturon_scene.yaml`) | 0.0611 | 0.0062 | 0.0704 | **0.138** | 0.1299 | 0.1240 |

RMSE of the 2D position in metres. The initial parameters reproduce the original trial 0 within
≈ 0.5 %; the optimized parameters reduce the error by 92 % (1.404 → 0.106 m), close to the original
0.090 m. The parameters of the soturon scene (two lateral-friction values from an older study) are
in between (0.138 m): the optimized set fits the real WHILL better in every scenario, including
the two scenarios that were not used in the optimization (s_curve, lemniscate). **Therefore the
optimized set is the default of `WhillModelCR.prefab` in this repository.** The remaining difference is expected: different ground (flat box instead of the Shinjuku
road mesh), single runs, real-time replay jitter.

## 7. Applying parameters to AWSIM

- **At runtime** (lost when AWSIM restarts):
  `python3 -m whill_optimizer apply-params --yaml <file>`; verify with `get-params`,
  compare files with `diff-params a.yaml b.yaml`.
- **Permanently**: set the AccelVehicle fields / Rigidbody center of mass of
  `WhillModelCR.prefab` in the Inspector (names in §2), or apply the YAML at runtime from your
  start-up script. The prefab default is
  [`whill_model_cr_optimized_3scenario_v2.yaml`](../config/vehicle/whill_model_cr_optimized_3scenario_v2.yaml);
  the state of soturon is [`whill_model_cr_soturon_scene.yaml`](../config/vehicle/whill_model_cr_soturon_scene.yaml).

## 8. Reproducibility check

1. `python3 -m pytest optimizer/tests` — configuration files are consistent (search space vs
   runtime parameters, initial = defaults, optimum inside the space).
2. Run `batch-eval` on `config/vehicle/` with your recordings in the open field and compare with §6.
3. Re-run a study with `--trials` ≥ 300; TPE is stochastic, so expect a different optimum with a
   similar score, not the same numbers.
