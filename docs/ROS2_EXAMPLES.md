# ROS 2 examples

All commands assume AWSIM is running (`scripts/run_awsim.sh`) and use the ROS 2 tools image
(`scripts/ros2_shell.sh`, which mounts the repository at `/workspace` and sets the same DDS
settings as AWSIM). With a native ROS 2 Humble installation, run the same commands after

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-autoware-vehicle-msgs
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
export ROS_DOMAIN_ID=0          # same as AWSIM
```

> Do not `source /opt/ros/humble/setup.bash` in the shell that starts AWSIM; `scripts/run_awsim.sh`
> removes it (AWSIM ships its own ROS 2 libraries).

## 1. Drive with `ros2 topic pub`

```bash
scripts/ros2_shell.sh            # interactive shell in the container

# 1) gear DRIVE (once; the WHILL starts in PARKING)
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{command: 2}"

# 2) forward 0.3 m/s
ros2 topic pub -r 10 --qos-reliability reliable --qos-durability transient_local \
  /whill/controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
# backward: {linear: {x: -0.3}}   turn left in place: {angular: {z: 0.5}}   right: {angular: {z: -0.5}}

# 3) stop (AWSIM keeps the last command, so send zero)
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /whill/controller/cmd_vel geometry_msgs/msg/Twist "{}"
```

AWSIM subscribes with TRANSIENT_LOCAL and does not match a VOLATILE publisher (the default QoS of
rclpy/rclcpp publishers). `ros2 topic pub` of Humble defaults to transient_local, so the flags are
optional there; in your own nodes set the durability as in §4.

## 2. Read the state

```bash
ros2 topic echo --once /ground_truth/odom                       # pose (map) and twist (base_link)
ros2 topic hz /ground_truth/odom
ros2 topic echo --once /whill/wheel_velocities_sim              # [L, R, L target, R target] m/s
ros2 topic echo --once --qos-durability transient_local /tf_static
ros2 topic hz /infrastructure/lidar_0/points
```

RViz (with a display): `rviz2`, fixed frame `map`, add PointCloud2 `/infrastructure/lidar_0/points`
(reliability Best Effort) and Odometry `/ground_truth/odom`. The infrastructure cloud is placed by
`/tf_static`; the vehicle LiDAR (`velodyne_top`) has no TF (see ROS2_INTERFACE.md §3).

## 3. Test scripts (`ros2_examples/`)

| Script | What it checks | Pass criterion |
|---|---|---|
| `teleop_keyboard.py` | interactive driving from the terminal: w/s speed ±0.1 m/s, a/d turn ±0.1 rad/s, space stop, q quit; shows the odometry | – |
| `drive_test.py [--csv out.csv]` | gear DRIVE, then forward 0.4 m/s 3 s, stop 2 s, backward −0.3 m/s 3 s, stop, turn left 0.5 rad/s 3 s, stop, turn right, stop | displacement along the heading > 50 % of the commanded one, yaw change > 50 %, \|v\|,\|ω\| < 0.05 at the end of each stop |
| `check_interfaces.py [--duration 10] [--json out.json]` | rate, frame_id, size, points and stamp − /clock of every output topic | prints a Markdown table (no pass/fail) |
| `infra_lidar_check.py [--topic ...]` | the infrastructure LiDAR sees the WHILL: cloud transformed with `/tf_static`, points counted in a box around base_link | median ≥ 20 points |
| `reset_collision_test.py` | `/reset_command` → `/reset_response` and back at the start pose; drive until `/collision_sensor`; start pose and goal over `/awsim/episode/*` | response received, pose error < 0.1 m (reset) / < 0.05 m, 2° (start pose), collision within 25 s, goal echoed |

Run them in the container:

```bash
scripts/ros2_shell.sh python3 ros2_examples/drive_test.py
scripts/ros2_shell.sh python3 ros2_examples/check_interfaces.py
scripts/ros2_shell.sh python3 ros2_examples/infra_lidar_check.py
scripts/ros2_shell.sh python3 ros2_examples/reset_collision_test.py   # corridor scene only (needs Obstacle-tagged objects)
```

Output of `drive_test.py` on the reference machine (RTX 5090, default West Shinjuku scene, sidewalk with a 2° slope):

```
Gear DRIVE: OK
PASS  forward    cmd=(+0.40 m/s, +0.00 rad/s, 3 s)  along=+1.201 m lateral=+0.012 m dyaw=-0.5 deg  v_end=+0.410 w_end=-0.002
PASS  stop_1     ...  v_end=+0.000 w_end=-0.000
PASS  backward   cmd=(-0.30 m/s, +0.00 rad/s, 3 s)  along=-0.844 m lateral=+0.017 m dyaw=-0.4 deg  v_end=-0.287
PASS  stop_2     ...
PASS  turn_left  cmd=(+0.00 m/s, +0.50 rad/s, 3 s)  along=+0.032 m lateral=+0.016 m dyaw=+81.9 deg  w_end=+0.501
PASS  stop_3     ...
PASS  turn_right cmd=(+0.00 m/s, -0.50 rad/s, 3 s)  along=+0.025 m lateral=-0.025 m dyaw=-81.5 deg  w_end=-0.497
PASS  stop_4     ...
8/8 phases passed
```

## 4. Python snippet

```python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from autoware_vehicle_msgs.msg import GearCommand

rclpy.init()
node = rclpy.create_node("whill_teleop")
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
gear = node.create_publisher(GearCommand, "/control/command/gear_cmd", qos)
cmd = node.create_publisher(Twist, "/whill/controller/cmd_vel", qos)
gear.publish(GearCommand(command=GearCommand.DRIVE))
twist = Twist()
twist.linear.x, twist.angular.z = 0.3, 0.2      # m/s, rad/s (left)
cmd.publish(twist)
```

## 5. Use simulation time

```bash
ros2 run <pkg> <node> --ros-args -p use_sim_time:=true
```

Stamps are seconds since AWSIM started (`/clock`), not wall-clock time.
