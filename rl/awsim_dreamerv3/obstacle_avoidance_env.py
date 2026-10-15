"""
AWSIM Obstacle Avoidance (Cardboard Box) Gym Environment for DreamerV3

ダンボール回避前進タスク用のGymnasium環境。
既存の AWSIMWheelchairEnv を継承し、以下を変更:

1. 行動空間: [-0.3, 0.3] m/s (前進のみ), [-0.3, 0.3] rad/s
2. ゴール判定: Unity X方向に-3m (ROS Y方向に+3m)
3. 報酬関数:
   - 障害物/壁にぶつかったら → 負の報酬 (-50)
   - 前進していたら → 正の報酬 (距離ベース)
   - ゴールに到達したら → 正の報酬 (+100)
"""

import numpy as np
import time
from typing import Optional, Tuple, Dict, Any, List

from .env import AWSIMWheelchairEnv
from . import obstacle_avoidance_config as oa_config


class ObstacleAvoidanceEnv(AWSIMWheelchairEnv):
    """
    Gymnasium environment for cardboard box avoidance in AWSIM.
    
    Inherits from AWSIMWheelchairEnv and overrides:
    - Action scaling (0.3 m/s, 0.3 rad/s)
    - Goal computation (Unity X = -3m → ROS Y = +3m)
    - Reward function (collision penalty, forward reward, goal reward)
    
    Observation:
        - image: RGB camera image (64x64x3)
        - is_success: Whether goal was reached (float32)
        - is_collision: Whether collision occurred (float32)
        
    Action:
        - Continuous 2D: [linear_velocity, angular_velocity]
        - Range: [-1, 1] normalized → scaled to [0, 0.3] m/s and [-0.3, 0.3] rad/s
    """
    
    def __init__(
        self,
        render_mode: Optional[str] = None,
        image_size: Tuple[int, int] = oa_config.IMAGE_SIZE,
        max_linear_vel: float = oa_config.MAX_LINEAR_VEL,
        max_angular_vel: float = oa_config.MAX_ANGULAR_VEL,
        episode_timeout: float = oa_config.EPISODE_TIMEOUT,
        step_duration: float = oa_config.STEP_DURATION,
        enable_domain_randomization: bool = False,
    ):
        """
        Initialize the obstacle avoidance environment.
        
        Args:
            render_mode: "rgb_array" for rendering
            image_size: Size to resize camera images to
            max_linear_vel: Maximum forward velocity (m/s) - default 0.3
            max_angular_vel: Maximum angular velocity (rad/s) - default 0.3
            episode_timeout: Maximum episode duration (s) - default 30
            step_duration: Duration of each step (s)
        """
        # Call parent __init__ with obstacle avoidance specific parameters
        # goal_distance is not used directly (we compute our own),
        # but pass a dummy value for the parent
        super().__init__(
            render_mode=render_mode,
            image_size=image_size,
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            goal_distance=oa_config.GOAL_DISTANCE_ROS_Y,  # Not used directly
            episode_timeout=episode_timeout,
            step_duration=step_duration,
            enable_domain_randomization=enable_domain_randomization,
        )
        
        # Override goal settings for obstacle avoidance task
        self.goal_radius = oa_config.GOAL_RADIUS
        self._goal_ros_y_offset = oa_config.GOAL_DISTANCE_ROS_Y  # +3m in ROS Y
    
    def _get_goal_center(self) -> Tuple[float, float]:
        """
        Get goal center position in ROS XY plane.
        
        Uses goal_2 object position from Unity if available,
        otherwise falls back to initial_position + offset.
        
        Returns:
            (goal_x, goal_y) in ROS frame.
        """
        # Try to use goal_2 object position from Unity
        goal_pos = self._ros2.get_goal_position()
        if goal_pos is not None:
            return goal_pos[0], goal_pos[1]  # ROS X, Y
        
        # Fallback: initial position + offset
        initial_pos = self._ros2.get_initial_position()
        if initial_pos is not None:
            return initial_pos[0], initial_pos[1] + self._goal_ros_y_offset
        
        return 0.0, 0.0
    
    def _compute_distance_to_goal(self) -> float:
        """
        Compute 2D distance to goal center (for reward calculation).
        
        Returns:
            2D distance to goal center, or inf if position unknown.
        """
        goal_x, goal_y = self._get_goal_center()
        current_x, current_y, _ = self._ros2.get_position_3d()
        
        distance = np.sqrt(
            (current_x - goal_x) ** 2 +
            (current_y - goal_y) ** 2
        )
        return distance
    
    def _check_goal_reached(self) -> bool:
        """
        Check if wheelchair is within the circular goal area.
        
        Goal area is a circle of radius GOAL_RADIUS (0.5m)
        centered on the goal object position.
        
        Returns:
            True if within goal area.
        """
        distance = self._compute_distance_to_goal()
        return distance <= oa_config.GOAL_RADIUS
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """
        Reset the environment for a new episode.
        Calls parent reset, then re-initializes distance tracking for the
        obstacle avoidance goal axis (ROS Y / Unity -X).
        """
        obs, info = super().reset(seed=seed, options=options)
        
        # Override distance tracking for obstacle avoidance goal axis
        self._prev_distance_to_goal = self._compute_distance_to_goal()
        
        return obs, info
    
    def step(
        self,
        action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """
        Execute one step in the environment.
        
        Overrides parent to use:
        - Obstacle avoidance goal (Unity X = -3m)
        - Custom reward function
        
        Args:
            action: [linear_vel_normalized, angular_vel_normalized]
            
        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        self._step_count += 1
        
        # Convert normalized action to actual velocities
        linear_vel = float(action[0]) * self.max_linear_vel
        angular_vel = float(action[1]) * self.max_angular_vel
        
        # Clamp: forward only (no backward), angular both directions
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Send velocity command
        self._ros2.send_velocity_command(linear_vel, angular_vel)
        
        # Wait for new observation (timestamp blocking)
        current_timestamp = self._last_observation_timestamp
        timeout = 0.5
        start_wait = time.time()
        
        while True:
            new_timestamp = self._ros2.get_latest_image_timestamp()
            if new_timestamp > current_timestamp + 1e-6:
                self._last_observation_timestamp = new_timestamp
                break
            if time.time() - start_wait > timeout:
                print(f"Warning: Simulation step timed out ({timeout}s). AWSIM might be lagging.")
                break
            time.sleep(0.001)
        
        # Check for collision
        collision = self._ros2.check_collision()
        
        # Get current state
        forward_velocity = self._ros2.get_forward_velocity()
        
        # Compute distance to goal (for reward calculation)
        distance_to_goal = self._compute_distance_to_goal()
        
        # Check if goal reached (0.5m x 0.5m square area)
        goal_reached = self._check_goal_reached()
        
        # Calculate reward (obstacle avoidance specific)
        reward = self._calculate_reward(
            collision=collision,
            goal=goal_reached,
            prev_distance_to_goal=self._prev_distance_to_goal,
            distance_to_goal=distance_to_goal,
            current_velocity=forward_velocity,
            action=action,
            prev_action=self._prev_action,
        )
        
        # Update state for next step
        self._prev_distance_to_goal = distance_to_goal
        self._prev_action = action.copy()
        
        # Track trajectory
        try:
            current_pos = self._ros2.get_current_position()
            self._trajectory.append(current_pos)
        except Exception as e:
            print(f"Warning: Failed to track trajectory: {e}")
        
        # Check termination conditions
        terminated = False
        truncated = False
        is_collision = False
        is_success = False
        
        # Collision -> terminate
        if collision:
            terminated = True
            is_collision = True
        
        # Goal reached -> terminate
        if goal_reached:
            terminated = True
            is_success = True
        
        # Timeout -> truncate
        elapsed_time = time.time() - self._episode_start_time
        if elapsed_time >= self.episode_timeout:
            truncated = True
        
        # Get observation
        observation = self._get_observation(is_collision=is_collision, is_success=is_success)
        
        # Build info
        info = self._get_info()
        info.update({
            'collision': collision,
            'is_collision': is_collision,
            'is_success': is_success,
            'forward_velocity': forward_velocity,
            'distance_to_goal': distance_to_goal,
            'elapsed_time': elapsed_time,
        })
        
        # Return trajectory at the end of episode
        if terminated or truncated:
            info['trajectory'] = np.array(self._trajectory, dtype=np.float32)
        
        # Stop wheelchair if episode ends
        if terminated or truncated:
            self._ros2.send_velocity_command(0.0, 0.0)
        
        return observation, reward, terminated, truncated, info
    
    def _calculate_reward(
        self,
        collision: bool,
        goal: bool,
        prev_distance_to_goal: float,
        distance_to_goal: float,
        current_velocity: float,
        action: np.ndarray,
        prev_action: Optional[np.ndarray],
    ) -> float:
        """
        Calculate reward for obstacle avoidance task.
        
        報酬設計:
        - r_goal:      ゴール報酬 (+100)
        - r_collision:  衝突ペナルティ (-50) ※障害物・壁
        - r_forward:    前進報酬 (ゴール方向に近づいた分だけ正の報酬)
        - r_time:       時間ペナルティ (-0.05/step)
        
        Args:
            collision: Whether collision occurred (obstacle or wall)
            goal: Whether goal was reached
            prev_distance_to_goal: Previous distance to goal (m)
            distance_to_goal: Current distance to goal (m)
            current_velocity: Current forward velocity (m/s)
            action: Current action [linear_vel, angular_vel]
            prev_action: Previous action
            
        Returns:
            Total reward value
        """
        # 1. ゴール報酬
        r_goal = oa_config.GOAL_REWARD if goal else 0.0
        
        # 2. 衝突ペナルティ (障害物・壁)
        r_collision = oa_config.COLLISION_PENALTY if collision else 0.0
        
        # 3. 前進報酬 (ゴール方向に近づいた分だけ正)
        # diff > 0 なら近づいた, diff < 0 なら遠ざかった
        diff_dist = prev_distance_to_goal - distance_to_goal
        r_forward = oa_config.FORWARD_REWARD_SCALE * diff_dist
        
        # 4. 時間ペナルティ (早くゴールすることを促す)
        r_time = oa_config.TIME_PENALTY
        
        # 合計
        total_reward = r_goal + r_collision + r_forward + r_time
        
        return total_reward


# Register environment with Gymnasium
def register_obstacle_avoidance_env():
    """Register the obstacle avoidance environment with Gymnasium."""
    import gymnasium as gym
    gym.register(
        id='AWSIMObstacleAvoidance-v0',
        entry_point='awsim_dreamerv3.obstacle_avoidance_env:ObstacleAvoidanceEnv',
    )
