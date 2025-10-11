#!/usr/bin/env python3
import rclpy
# from rclpy.node import Node
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from stable_baselines3 import PPO
# from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from my_arm_rl.cld_robot_arm_env import RobotArmEnv

def main(args=None):
    rclpy.init(args=args)
    print("Starting RL Training for Robot Arm...")
    
    # Create environment
    env = RobotArmEnv()
    # env = DummyVecEnv([lambda: env])
    # check_env(env)  # Verify environment compatibility
    logger = configure("./logs", ["tensorboard"])
    # Create model
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        tensorboard_log="./tensorboard_logs/"
    )
    
    # Callback to save model every 10k steps
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path="./models/",
        name_prefix="robot_arm_ppo"
    )
    
    try:
        # Train the model
        print("Training started...")
        model.set_logger(logger)
        model.learn(
            total_timesteps=100000,
            callback=checkpoint_callback,
            progress_bar=True
        )
        
        # Save final model
        model.save("./models/robot_arm_final")
        print("Training completed and model saved!")
        
    except KeyboardInterrupt:
        print("Training interrupted by user")
        model.save("./models/robot_arm_interrupted")
    
    finally:
        env.close()

if __name__ == "__main__":
    main()