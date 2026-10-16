# Recordings of a real WHILL Model CR

rosbag2 (sqlite3) recordings of a real WHILL Model CR driven by velocity commands. They are the
reference data of the vehicle parameter optimization (`optimizer/whill_optimizer`, see
[docs/PARAMETER_OPTIMIZATION.md](../../docs/PARAMETER_OPTIMIZATION.md)) and were recorded with
`optimizer/control_whill.py` for the bachelor thesis (soturon), December 2025 – January 2026.

| Directory | Scenario name | Duration | Messages (cmd_vel / odom) |
|---|---|---|---|
| `whill_cmd_straight_20251216_125820` | `straight` | 14.6 s | 155 / 1 327 |
| `whill_cmd_turn_20251216_131910` | `turn` | 19.6 s | 214 / 1 778 |
| `whill_cmd_circle_20251216_131510` | `circle` | 16.6 s | 179 / 1 512 |
| `whill_cmd_s_curve_20251217_172829` | `s_curve` | 19.9 s | 214 / 1 810 |
| `whill_cmd_lemniscate_20260108_113110` | `lemniscate` | 24.5 s | 273 / 2 229 |

Topics of every recording:

| Topic | Type | Use |
|---|---|---|
| `/whill/controller/cmd_vel` | `geometry_msgs/msg/Twist` | commands replayed to AWSIM |
| `/whill/odom` | `nav_msgs/msg/Odometry` | reference trajectory (wheel odometry of the WHILL driver) |
| `/whill/modelc_state` | `ros2_whill_interfaces/msg/WhillModelC` | raw WHILL state (not used by the optimizer) |
| `/whill/states/batttery_state` | `sensor_msgs/msg/BatteryState` | battery (not used) |

The scenario names are mapped to these directories in
`optimizer/whill_optimizer/rosbag_loader.py`. `ros2_whill_interfaces` is the message package of the
WHILL ROS 2 driver; it is only needed to read `/whill/modelc_state`, which the optimizer does not do.

License: same as the code of this repository (Apache 2.0).
