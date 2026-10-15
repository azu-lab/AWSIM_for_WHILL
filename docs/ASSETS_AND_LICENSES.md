# Assets and licenses

This repository is a derivative of [AWSIM](https://github.com/autowarefoundation/AWSIM)
(commit `fa1dc65`, see [UPSTREAM_DIFF.md](UPSTREAM_DIFF.md)) plus code and assets written for
the WHILL Model CR. Code and assets are licensed differently; check the table for the file you
want to reuse.

> This document records what was checked while preparing the repository. It is not legal
> advice. Items marked "not verified" were inherited unchanged from upstream AWSIM and were not
> re-checked file by file.

## 1. AWSIM License (upstream, kept in [`LICENSE`](../LICENSE))

The `LICENSE` file is the unmodified AWSIM License. It states:

- AWSIM specific **code** (`*.cs`, `*.compute`, `*.xml`) — **Apache License 2.0**.
- AWSIM specific **assets** (`*.fbx`, `*.pcd`, `*.osm`, `*.png`, `*.anim`, `*.unitypackage`,
  `*.x86_64`) — **CC BY-NC** (non-commercial).
- **AWSIM documents (`docs/`) may not be hosted or published except on the official site.**

Consequences for this repository:

- The upstream `docs/` directory, `mkdocs.yml`, the documentation workflow and `README_img/`
  are **not included**. The `docs/` directory of this repository contains only documents
  written for this project.
- Upstream AWSIM assets that are included (models of the Lexus, NPCs, pedestrians, sign boards,
  textures, ...) remain under **CC BY-NC**: this repository as a whole cannot be used
  commercially.
- Prebuilt players (`*.x86_64`) are **not** distributed; build them with
  `scripts/build_player.sh`.
- The Shinjuku map (AWSIM "Externals", 1.4 GB, CC BY-NC) is not included. The upstream scene
  keeps an inactive `Environments/Shinjuku` node whose meshes reference it; this project does not
  use it.

## 2. What is in this repository

| Path | Origin | License |
|---|---|---|
| `Assets/Awsim/**` (except the rows below), `ProjectSettings/`, `Packages/manifest.json` | AWSIM `fa1dc65` (TIER IV) | Code: Apache 2.0. Assets: CC BY-NC (AWSIM License) |
| `Assets/RGLUnityPlugin/` | Robotec GPU Lidar Unity plugin, shipped by AWSIM (`libRobotecGPULidar.so` 0.18.0) | Apache 2.0 (Robotec.ai); two `.cs` files modified by soturon (API update) |
| `Assets/Ros2ForUnity/` | Ros2ForUnity 1.3.0 "standalone" with ROS 2 Humble libraries and message assemblies, shipped by AWSIM | Apache 2.0 (Robotec.ai) for Ros2ForUnity; the bundled ROS 2 libraries keep their own licenses (mostly Apache 2.0 / BSD) — not verified individually |
| `Assets/TextMesh Pro/` | Unity TextMesh Pro resources, shipped by AWSIM | Unity Companion License — not verified |
| `Assets/Awsim/Plugins/` | Third-party .NET / native libraries shipped by AWSIM: `GeographicLib.dll`, `Google.Protobuf.dll` (© Google), `YamlDotNet.dll` (© Antoine Aubry and contributors), `System.*.dll`, `ZeroMQ.dll`, `LogitechG29Linux/libG29Linux.so` | Licenses of the respective projects — not verified; redistributed exactly as upstream AWSIM does |
| `Assets/Awsim/Fonts/` | Oswald, Ubuntu Mono, shipped by AWSIM | Font licenses of the respective projects (OFL / Ubuntu Font Licence) — not verified |
| Unity packages (HDRP, URP, Input System, ...) | Downloaded by the Unity Package Manager from `Packages/manifest.json`; not stored in the repository | Unity licenses |
| WHILL changes of soturon (AccelVehicle differential drive, ROS 2 Twist input, GroundTruthPublisher, WheelSpeedPublisher, CollisionPublisher, EnvironmentReset, OptimizerBridge, CoMVisualizer) | shoumexi-03/soturon (`optimize`, `e91b33f`) | Apache 2.0, as AWSIM code (see §4 for the copyright notices) |
| `Assets/Awsim/Prefabs/Entity/EgoVehicle/WhillModelCR/` (prefab, placeholder materials) | This repository (derived from the soturon prefab, visual model replaced) | Apache 2.0 |
| `Assets/Awsim/Models/Entity/Cardboard.mat`, `New Material*.mat`, `Misc/redgoal.mat` | soturon (plain colors; the cardboard photo texture was removed) | Apache 2.0 |
| `Assets/Awsim/Scripts/InfraLidar/`, `Assets/Awsim/Scripts/Editor/{InfraLidar,WhillModelCR,Build}/` | This repository | Apache 2.0 |
| `optimizer/`, `rl/`, `ros2_examples/`, `scripts/`, `config/`, `docker/`, `docs/`, `README*.md` | soturon Python tools (adapted) and this repository | Apache 2.0 |
| `third_party/dreamerv3` (git submodule, not copied) | [danijar/dreamerv3](https://github.com/danijar/dreamerv3) @ `b65cf81` | MIT |

## 3. Excluded from this repository

Identified in soturon/optimize (`e91b33f`) and deliberately not copied. The Git history of this
repository was created from scratch, so these files are **not in any commit** either
(`git log --all -- <path>` returns nothing; the check is part of the publication checklist in
[TEST_REPORT.md](TEST_REPORT.md)).

| soturon path | Content | Reason |
|---|---|---|
| `AWSIM/Assets/Awsim/Models/Entity/Vehicle/EgoVehicle/Model Ci - FBX/` (`Model Ci.FBX`, `Untitled.fbx`, `untitled(1).fbx`, `Model Ci Part Back_Mask.jpg`, `Tech Leather_Bump.jpg`, `Tech Leather_Ref.jpg`, `ModelCi.prefab`) | 3D model of the WHILL Model CR (named "Model Ci") | Not publishable (no redistribution right). Replaced by the placeholder model |
| `AWSIM/Assets/Awsim/Environment/Mesh[slam_map_20260206_221614 - Cloud] (level 8).obj`, `wall_material.mat` | Mesh reconstructed from a SLAM point cloud of a real building | Information about a real (laboratory) building |
| `AWSIM/Assets/Awsim/Models/Entity/IMG_6763 (1).jpeg`, `IMG_6764 (1).jpeg`, `Misc/IMG_6763 (1).jpeg` | Photos of a cardboard box used as texture | Third-party logos and a shipping label |
| `AWSIM/Assets/NewBehaviourScript.cs`, `.../Input/AccelVehicleInputSystemJoystick.cs` | Empty template / empty file | Unused |
| `whill_cmd_*`, `awsim_replay_*`, `rosbags/` (rosbag2) | Recordings of a real WHILL (cmd_vel, odometry, battery) and of AWSIM | Publication right not confirmed; required only to re-run the optimization (see PARAMETER_OPTIMIZATION.md) |
| `*.db`, `*_results.json`, `batch_eval_*`, `ttest_*`, `visualization/`, `world_model/sim_trajectory_*`, `world_model/eval_*.csv` | Optimization studies and evaluation results | Derived from the real-WHILL recordings (same reason). The resulting parameter sets are published in `config/vehicle/` |
| `specification.md`, `system_model.md`, `toukei_siyousho.md`, `AWSIM_DreamerV3_Spec.md`, `AWSIM/パラメータ調整ガイド.md`, `README.md` | Internal design notes | Partly outdated (e.g. "gRPC" for what is a TCP JSON protocol); replaced by the documents in `docs/` |
| `sadounirin.cs`, `yonrin.cs` | Old snapshots of AccelVehicle.cs | Superseded by the current file |
| upstream `docs/`, `mkdocs.yml`, `README_img/` | AWSIM documentation | AWSIM License forbids hosting it elsewhere |

## 4. Copyright notices to confirm before publication

- `Assets/Awsim/Scripts/DreamerV3/CollisionPublisher.cs` and `EnvironmentReset.cs` carry
  `// Copyright 2025 Azumi Lab.`
- `Assets/Awsim/Scripts/Optimizer/OptimizerBridge.cs` carries the TIER IV Apache header,
  although the file was written for soturon (it does not exist upstream).

These headers were kept unchanged: who holds the copyright of the code written during the
bachelor thesis is for the author to decide. New files of this repository carry
`Copyright 2026 shoumexi-03` with the Apache 2.0 header.
