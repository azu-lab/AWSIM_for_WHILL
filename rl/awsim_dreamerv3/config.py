# AWSIM DreamerV3 Configuration

# Image settings
IMAGE_WIDTH = 64
IMAGE_HEIGHT = 64
IMAGE_SIZE = (IMAGE_HEIGHT, IMAGE_WIDTH)

# Action settings
MAX_LINEAR_VEL = 1.0   # m/s (forward velocity)
MAX_ANGULAR_VEL = 1.0  # rad/s (yaw rate)

# Episode settings
GOAL_DISTANCE = 5.0     # meters (distance to goal point in Forward/X direction)
GOAL_RADIUS = 0.75       # meters (radius around goal point to consider as reached)
EPISODE_TIMEOUT = 15.0  # seconds

# Reward settings ("original" task, AWSIMWheelchairEnv._calculate_reward)
# These values are the ones that were hard-coded in env.py (soturon e91b33f).
# They can be overridden with an environment YAML (rl/configs/env/*.yaml).
COLLISION_PENALTY = -50.0        # Large penalty for collision (safety first!)
GOAL_REWARD = 100.0              # Reward for reaching goal
PROGRESS_REWARD_SCALE = 30.0     # reward = scale * (prev_distance - current_distance) [per m]
TIME_PENALTY = -0.05             # per step
STEERING_CHANGE_PENALTY = 1.0    # reward -= scale * |angular action(t) - angular action(t-1)|

# ROS 2 Topics
CAMERA_TOPIC = "/sensing/camera/traffic_light/image_raw"
ODOM_TOPIC = "/ground_truth/odom"
COLLISION_TOPIC = "/collision_sensor"
CMD_VEL_TOPIC = "/whill/controller/cmd_vel"
GEAR_CMD_TOPIC = "/control/command/gear_cmd"
RESET_COMMAND_TOPIC = "/reset_command"
RESET_RESPONSE_TOPIC = "/reset_response"
INITIAL_POSE_TOPIC = "/initialpose"
GOAL_POSITION_TOPIC = "/awsim/goal_position"

# Episode settings sent to AWSIM before every reset (None = keep the scene placement).
# START_POSE: {"x", "y", "yaw"} of base_link in the map frame ([m], [m], [rad])
# GOAL_POSITION: {"x", "y"} of the goal object in the map frame [m]
START_POSE = None
GOAL_POSITION = None
START_POSE_TOPIC = "/awsim/episode/start_pose"
GOAL_TOPIC = "/awsim/episode/goal"

# AWSIM TCP Connection
AWSIM_HOST = "localhost"
AWSIM_PORT = 50051

# Timing
STEP_DURATION = 0.1  # seconds per step (10 Hz control)
ROS_SPIN_TIMEOUT = 0.01  # seconds for rclpy spin_once

# Logging
LOG_IMAGES_EVERY = 10000  # steps (frequency of image logging)
LOG_SCALARS_EVERY = 100   # steps (frequency of scalar logging)
IMAGE_SIZE = (64, 64)     # (height, width)
