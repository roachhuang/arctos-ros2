#!/usr/bin/env python3

import time
import numpy as np
from stable_baselines3 import PPO
from my_arm_rl.cld_robot_arm_env import RobotArmEnv
import rclpy

def main(args=None):
    rclpy.init(args=args)
    print("Testing trained RL model...")
    env = RobotArmEnv()
    state, _ = env.reset()  # Unpack obs, info for Gymnasium API
    time.sleep(2)  # Allow environment to stabilize
    print("Environment ready. Starting test...")
    # Load trained model
    model = PPO.load("./models/robot_arm_final")
    done = False

    while not done:
        action, _ = model.predict(state, deterministic=True)
        # Execute action
        state, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated

        if done:
            print(f"Episode finished. {done} reward: {reward:.2f}")
            break
    env.close()

if __name__ == "__main__":
    main()

"""
# Build package
cd ~/ros2_ws
colcon build --packages-select robot_arm_rl
source install/setup.bash

# Start Gazebo with your robot
ros2 launch your_robot_pkg robot_gazebo.launch.py

# In another terminal, start training
ros2 run robot_arm_rl train_rl

# After training, test the model
ros2 run robot_arm_rl test_rl
Step 8: Transfer to Real Robot
Once trained in Gazebo, use the same environment class but change the ROS2 topics to match your real robot's interface. The trained model will work directly!
"""
