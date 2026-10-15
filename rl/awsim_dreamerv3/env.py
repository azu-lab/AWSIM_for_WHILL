"""
AWSIM Wheelchair Gym Environment for DreamerV3

A Gymnasium environment that interfaces with AWSIM simulation
for wheelchair obstacle avoidance training.
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import cv2
import time
from typing import Optional, Tuple, Dict, Any, List

from . import config
from .ros2_interface import ROS2Interface
from .domain_randomization import DomainRandomization, DRConfig


class AWSIMWheelchairEnv(gym.Env):
    """
    Gymnasium environment for wheelchair obstacle avoidance in AWSIM.
    
    Observation:
        - image: RGB camera image (64x64x3)
        
    Action:
        - Continuous 2D: [linear_velocity, angular_velocity]
        - Range: [-1, 1] normalized
        
    Reward:
        - Progress reward: proportional to forward velocity
        - Collision penalty: -10 (episode ends)
        - Goal reward: +10 (episode ends when 5m forward progress)
    """
    
    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}
    
    def __init__(
        self,
        render_mode: Optional[str] = None,
        image_size: Tuple[int, int] = config.IMAGE_SIZE,
        max_linear_vel: float = config.MAX_LINEAR_VEL,
        max_angular_vel: float = config.MAX_ANGULAR_VEL,
        goal_distance: float = config.GOAL_DISTANCE,
        episode_timeout: float = config.EPISODE_TIMEOUT,
        step_duration: float = config.STEP_DURATION,
        enable_domain_randomization: bool = False,
    ):
        """
        Initialize the environment.
        
        Args:
            render_mode: "rgb_array" for rendering
            image_size: Size to resize camera images to
            max_linear_vel: Maximum forward velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            goal_distance: Distance to goal (m)
            episode_timeout: Maximum episode duration (s)
            step_duration: Duration of each step (s)
            enable_domain_randomization: Whether to apply visual domain randomization
        """
        super().__init__()
        
        self.render_mode = render_mode
        self.image_size = image_size
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.goal_distance = goal_distance
        self.goal_radius = config.GOAL_RADIUS  # Radius around goal to consider as reached
        self.episode_timeout = episode_timeout
        self.step_duration = step_duration
        
        # Define observation space
        # NOTE: The checkpoints in awsim_wheelchair_v3 were trained WITH is_collision
        # and is_success in the observation space. Keep these for compatibility.
        self.observation_space = spaces.Dict({
            'image': spaces.Box(
                low=0,
                high=255,
                shape=(image_size[0], image_size[1], 3),
                dtype=np.uint8
            ),
            'is_success': spaces.Box(low=0.0, high=1.0, shape=(), dtype=np.float32),
            'is_collision': spaces.Box(low=0.0, high=1.0, shape=(), dtype=np.float32),
        })
        
        # Define action space (continuous)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )
        
        # ROS 2 interface (lazy initialization)
        self._ros2: Optional[ROS2Interface] = None
        
        # Episode state
        self._episode_start_time: float = 0.0
        self._step_count: int = 0
        self._last_image: Optional[np.ndarray] = None
        self._last_observation_timestamp: float = 0.0
        
        # Domain Randomization
        self._domain_randomization = DomainRandomization() if enable_domain_randomization else None
        if enable_domain_randomization:
            print("[Domain Randomization] ENABLED")
        
        # For reward calculation
        self._prev_distance_to_goal: float = float('inf')
        self._prev_action: Optional[np.ndarray] = None
        self._trajectory: List[Tuple[float, float]] = []  # List of (x, z) coordinates
    
    def _ensure_ros2_initialized(self):
        """Ensure ROS 2 interface is initialized."""
        if self._ros2 is None:
            self._ros2 = ROS2Interface()
            # Wait for initial data
            if not self._ros2.wait_for_data(timeout=10.0):
                raise RuntimeError("Failed to receive initial sensor data from AWSIM")
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """
        Reset the environment for a new episode.
        
        Args:
            seed: Random seed
            options: Additional options
            
        Returns:
            Tuple of (observation, info)
        """
        super().reset(seed=seed)
        
        # Initialize ROS 2 if needed
        self._ensure_ros2_initialized()
        
        # Start pose / goal from the environment config (None = scene placement)
        if config.START_POSE is not None or config.GOAL_POSITION is not None:
            self._ros2.set_episode(config.START_POSE, config.GOAL_POSITION)
        
        # Call reset service
        success = self._ros2.reset(timeout=5.0)
        if not success:
            print("Warning: Reset service call failed, continuing anyway")
        
        if not success:
            print("Warning: Reset service call failed, continuing anyway")
        
        # Wait for vehicle to return to start position (Synchronization)
        # This replaces fixed sleep
        self._ros2.wait_for_reset_completion(timeout=5.0)
        
        # Clear any pending collision events
        self._ros2.check_collision()
        
        # Reset episode state
        self._episode_start_time = time.time()
        self._step_count = 0
        self._step_count = 0
        self._prev_distance_to_goal = float('inf')  # Reset distance tracking
        self._prev_action = None  # Reset action tracking
        self._trajectory = []  # Reset trajectory
        
        # Wait for fresh data after reset
        self._ros2.wait_for_data(timeout=2.0)
        
        # Get initial observation
        observation = self._get_observation()
        info = self._get_info()
        self._last_observation_timestamp = self._ros2.get_latest_image_timestamp()
        
        # Initialize prev_distance_to_goal after getting observation
        self._prev_distance_to_goal = self._ros2.get_distance_to_goal(self.goal_distance)
        
        return observation, info
    
    def step(
        self,
        action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """
        Execute one step in the environment.
        
        Args:
            action: [linear_vel_normalized, angular_vel_normalized]
            
        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        self._step_count += 1
        
        # Convert normalized action to actual velocities
        linear_vel = float(action[0]) * self.max_linear_vel
        angular_vel = float(action[1]) * self.max_angular_vel
        
        # Clamp to valid range (only forward motion for safety)
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Send velocity command
        self._ros2.send_velocity_command(linear_vel, angular_vel)
        
        # Wait for new observation (timestamp blocking)
        # Instead of fixed sleep, wait until a new image arrives
        # This prevents using old data (observation lag) and handles processing delays
        current_timestamp = self._last_observation_timestamp
        timeout = 0.5  # Max wait time (seconds)
        start_wait = time.time()
        
        while True:
            new_timestamp = self._ros2.get_latest_image_timestamp()
            if new_timestamp > current_timestamp + 1e-6:  # Small epsilon for float comparison
                self._last_observation_timestamp = new_timestamp
                break
                
            if time.time() - start_wait > timeout:
                # Warning if simulation is too slow, but don't crash
                # Just proceed with whatever data we have
                print(f"Warning: Simulation step timed out ({timeout}s). AWSIM might be lagging.")
                break
                
            time.sleep(0.001)  # tiny sleep to prevent CPU hogging
        
        # Check for collision
        collision = self._ros2.check_collision()
        
        # Get current state
        forward_velocity = self._ros2.get_forward_velocity()
        forward_progress = self._ros2.get_distance_from_start()
        distance_to_goal = self._ros2.get_distance_to_goal(self.goal_distance)
        
        # Check if goal reached
        goal_reached = distance_to_goal <= self.goal_radius
        
        # Calculate reward
        reward = self._calculate_reward(
            collision=collision,
            goal=goal_reached,
            prev_distance_to_goal=self._prev_distance_to_goal,
            distance_to_goal=distance_to_goal,
            current_velocity=forward_velocity,
            action=action,
            prev_action=self._prev_action
        )
        
        # Update state for next step
        self._prev_distance_to_goal = distance_to_goal
        self._prev_action = action.copy()
        
        # Track trajectory (only X, Z as per current implementation)
        try:
            current_pos = self._ros2.get_current_position()
            self._trajectory.append(current_pos)
        except Exception as e:
            print(f"Warning: Failed to track trajectory: {e}")
        
        # Check termination conditions
        terminated = False
        truncated = False
        
        # Outcome flags for evaluation
        is_collision = False
        is_success = False
        
        # Collision -> terminate
        if collision:
            terminated = True
            is_collision = True
        
        # Goal reached -> terminate (within goal_radius of goal point)
        if distance_to_goal <= self.goal_radius:
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
            'forward_progress': forward_progress,
            'forward_velocity': forward_velocity,
            'elapsed_time': elapsed_time,
        })
        
        # Return trajectory at the end of episode
        if terminated or truncated:
            # Add trajectory as numpy array
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
        prev_action: Optional[np.ndarray]
    ) -> float:
        """
        Calculate reward for current step.
        
        Reward design:
        - r_goal: ゴール報酬 (+100)
        - r_collision: 衝突ペナルティ (-20)
        - r_dist: 距離報酬 (5.0 * distance_change)
        - r_forward: 前進ボーナス (速度に基づく)
        - r_time: 生存コスト (-0.05/step)
        - r_smooth: スムーズボーナス (移動中のみ)
        
        Args:
            collision: Whether collision occurred
            goal: Whether goal reached
            prev_distance_to_goal: Previous distance to goal (m)
            distance_to_goal: Current distance to goal (m)
            current_velocity: Current velocity (m/s)
            action: Current action [linear_vel, angular_vel]
            prev_action: Previous action [linear_vel, angular_vel]
            
        Returns:
            Reward value
        """
        # 1. ゴール報酬
        r_goal = config.GOAL_REWARD if goal else 0.0

        # 2. 衝突ペナルティ
        r_collision = config.COLLISION_PENALTY if collision else 0.0

        # 3. 距離報酬（差分ベース）
        # 「近づいた分だけ」プラス、「遠ざかったら」マイナス
        # その場にとどまると 0 になるため、ウロウロを防止できます。
        # Scale: 30.0 (1m近づくと +30点)
        # 5m完走で +150点程度
        diff_dist = prev_distance_to_goal - distance_to_goal
        r_dist = config.PROGRESS_REWARD_SCALE * diff_dist

        # 4. 時間ペナルティ (早くゴールすることを促す)
        r_time = config.TIME_PENALTY

        # 5. スムーズ報酬 (角速度の変化を抑制、移動中のみ)
        r_smooth = 0.0
        if prev_action is not None and not collision and not goal:
            # action[1] is angular velocity (normalized -1 to 1)
            # Penalize jerky steering
            ang_diff = abs(action[1] - prev_action[1])
            r_smooth = -config.STEERING_CHANGE_PENALTY * ang_diff

        # 合計
        total_reward = r_goal + r_collision + r_dist + r_time + r_smooth
        
        # Debug logging
        # print(f"Rew: Tot={total_reward:.2f} (Goal={r_goal:.1f}, Col={r_collision:.1f}, Dist={r_dist:.2f})")
        
        return total_reward
    
    def _get_observation(self, is_collision: bool = False, is_success: bool = False) -> Dict[str, np.ndarray]:
        """Get current observation.
        
        Args:
            is_collision: Whether a collision occurred
            is_success: Whether the goal was reached
        """
        image = self._ros2.get_image()
        
        if image is None:
            # Return black image if no data
            image = np.zeros(
                (self.image_size[0], self.image_size[1], 3),
                dtype=np.uint8
            )
        else:
            # Resize to target size (width, height) for cv2
            image = cv2.resize(image, (config.IMAGE_SIZE[1], config.IMAGE_SIZE[0]), interpolation=cv2.INTER_AREA)
            image = np.asarray(image, dtype=np.uint8)
        
        # Apply Domain Randomization
        if self._domain_randomization is not None and image is not None:
            image = self._domain_randomization.apply(image)
        
        self._last_image = image
        
        return {
            'image': image,
            'is_collision': np.float32(is_collision),
            'is_success': np.float32(is_success),
        }
    
    def _get_info(self) -> Dict[str, Any]:
        """Get info dictionary."""
        return {
            'step_count': self._step_count,
        }
    
    def render(self) -> Optional[np.ndarray]:
        """Render the environment."""
        if self.render_mode == "rgb_array":
            return self._last_image
        return None
    
    def close(self):
        """Clean up resources."""
        if self._ros2 is not None:
            # Stop wheelchair
            self._ros2.send_velocity_command(0.0, 0.0)
            self._ros2.destroy()
            self._ros2 = None


# Register environment with Gymnasium
def register_env():
    """Register the environment with Gymnasium."""
    gym.register(
        id='AWSIMWheelchair-v0',
        entry_point='awsim_dreamerv3.env:AWSIMWheelchairEnv',
    )
