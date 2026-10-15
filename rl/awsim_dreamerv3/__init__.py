"""
AWSIM DreamerV3 Package

Heavy modules (rclpy, gymnasium) are lazily imported to speed up startup
when only lightweight submodules like config are needed.
"""

from . import config
from . import obstacle_avoidance_config

__all__ = [
    'AWSIMWheelchairEnv',
    'ObstacleAvoidanceEnv',
    'ROS2Interface',
    'config',
    'obstacle_avoidance_config',
]

# Lazy imports for heavy modules (ROS2, gymnasium, etc.)
def __getattr__(name):
    if name == 'AWSIMWheelchairEnv':
        from .env import AWSIMWheelchairEnv
        return AWSIMWheelchairEnv
    elif name == 'ObstacleAvoidanceEnv':
        from .obstacle_avoidance_env import ObstacleAvoidanceEnv
        return ObstacleAvoidanceEnv
    elif name == 'ROS2Interface':
        from .ros2_interface import ROS2Interface
        return ROS2Interface
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

