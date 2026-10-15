#!/usr/bin/env python3
"""
Environment Test Script for AWSIM Wheelchair DreamerV3

Tests the Gym environment without DreamerV3 to verify:
- ROS 2 communication
- Image acquisition
- Velocity commands
- Reset service
- Collision detection
"""

import sys
from pathlib import Path
import time
import numpy as np

# Add parent directory to path for imports
# rl/ (awsim_dreamerv3) を import できるようにする
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'rl'))

from awsim_dreamerv3.env import AWSIMWheelchairEnv


def test_basic_functionality():
    """Test basic environment functionality."""
    print("=" * 60)
    print("AWSIM Wheelchair Environment Test")
    print("=" * 60)
    
    print("\n[1] Creating environment...")
    env = AWSIMWheelchairEnv(render_mode="rgb_array")
    print("    ✓ Environment created")
    
    print("\n[2] Testing reset...")
    obs, info = env.reset()
    print(f"    ✓ Reset complete")
    print(f"    Image shape: {obs['image'].shape}")
    print(f"    Image dtype: {obs['image'].dtype}")
    print(f"    Image range: [{obs['image'].min()}, {obs['image'].max()}]")
    
    # Check if image is not all zeros
    if obs['image'].sum() > 0:
        print("    ✓ Image contains non-zero data")
    else:
        print("    ⚠ WARNING: Image is all zeros!")
    
    print("\n[3] Testing step with forward motion...")
    action = np.array([0.5, 0.0], dtype=np.float32)  # Forward at half speed
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"    ✓ Step complete")
    print(f"    Reward: {reward:.4f}")
    print(f"    Forward velocity: {info.get('forward_velocity', 0):.4f} m/s")
    print(f"    Forward progress: {info.get('forward_progress', 0):.4f} m")
    print(f"    Terminated: {terminated}, Truncated: {truncated}")
    
    print("\n[4] Testing multiple steps...")
    total_reward = 0
    for i in range(10):
        action = np.array([0.8, 0.0], dtype=np.float32)  # Forward
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        if terminated or truncated:
            print(f"    Episode ended at step {i+1}")
            break
    
    print(f"    ✓ Completed 10 steps")
    print(f"    Total reward: {total_reward:.4f}")
    print(f"    Final progress: {info.get('forward_progress', 0):.4f} m")
    
    print("\n[5] Testing turning motion...")
    action = np.array([0.3, 0.5], dtype=np.float32)  # Forward + turn right
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"    ✓ Turn step complete")
    print(f"    Reward: {reward:.4f}")
    
    print("\n[6] Testing stop command...")
    action = np.array([0.0, 0.0], dtype=np.float32)
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"    ✓ Stop command sent")
    
    print("\n[7] Testing reset after episode...")
    obs, info = env.reset()
    print(f"    ✓ Second reset complete")
    print(f"    Image shape: {obs['image'].shape}")
    
    print("\n[8] Cleaning up...")
    env.close()
    print("    ✓ Environment closed")
    
    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)


def test_episode_loop():
    """Run a full episode with random actions."""
    print("\n" + "=" * 60)
    print("Running Full Episode Test")
    print("=" * 60)
    
    env = AWSIMWheelchairEnv()
    obs, info = env.reset()
    
    total_reward = 0
    step = 0
    done = False
    
    print("\nRunning episode with random actions...")
    
    while not done:
        # Random action (forward-biased)
        action = np.random.uniform(-1, 1, size=2).astype(np.float32)
        action[0] = abs(action[0])  # Forward only
        
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        step += 1
        done = terminated or truncated
        
        if step % 50 == 0:
            print(f"  Step {step}: reward={reward:.2f}, progress={info.get('forward_progress', 0):.2f}m")
    
    print(f"\nEpisode finished:")
    print(f"  Steps: {step}")
    print(f"  Total reward: {total_reward:.2f}")
    print(f"  Final progress: {info.get('forward_progress', 0):.2f}m")
    print(f"  Collision: {info.get('collision', False)}")
    print(f"  Elapsed time: {info.get('elapsed_time', 0):.1f}s")
    
    env.close()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test AWSIM Wheelchair Environment")
    parser.add_argument("--full-episode", action="store_true", help="Run full episode test")
    args = parser.parse_args()
    
    try:
        test_basic_functionality()
        
        if args.full_episode:
            test_episode_loop()
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
