import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node

# from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import threading
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from roboticstoolbox import DHRobot, RevoluteDH

"""
Main thread: Running your RL environment (step, reset, etc.)
ROS spin thread: Handling callbacks like joint_state_callback
"""

# Define constants (fill in your values)
a1, a2, a3 = 0.047, 0.11, 0.026
d1, d4, d6 = 0.133, 0.1175, 0.028

links = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),
    RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),
    RevoluteDH(d=d4, a=0, alpha=np.pi / 2),
    RevoluteDH(d=0, a=0, alpha=-np.pi / 2),
    RevoluteDH(d=d6, a=0, alpha=0),
]
robot = DHRobot(links, name="my_6dof_arm")

'''
class RobotArmEnv(gym.Env, Node):
    def __init__(self):
        gym.Env.__init__(self)
        Node.__init__(self, "robot_arm_rl_env")

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )
        obs_low = np.array([-3.14] * 3 + [-1.0] * 3 + [-1.0] * 3, dtype=np.float32)
        obs_high = np.array([3.14] * 3 + [1.0] * 3 + [1.0] * 3, dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32
        )

        self.joint_positions = np.zeros(6)
        self.end_effector_pos = np.zeros(3)
        self.target_position = np.zeros(3)
        self.prev_joints = np.zeros(6)
        self.previous_distance = float("inf")
        self.current_step = 0
        self.max_steps = 1000

        self.state_lock = False
        self.success_threshold = 0.05
        self.curriculum_stage = 1
        self.success_counter = 0
        self.marker_pub = self.create_publisher(Marker, "/target_marker", 10)

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

        # Start ROS2 spinning in a separate thread
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

    def joint_state_callback(self, msg):
        self.joint_positions = np.array(msg.position[:6])

    # call at the start of each episode
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Move to home position
        home_angles = [0.0] * 6
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [f"joint{i+1}" for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = home_angles
        point.time_from_start = rclpy.duration.Duration(seconds=1.5).to_msg()
        traj_msg.points = [point]
        self.joint_cmd_pub.publish(traj_msg)

        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        self.target_position = self._generate_target_position()
        self.end_effector_pos = self.fk(self.joint_positions)
        self.previous_distance = np.linalg.norm(
            self.end_effector_pos - self.target_position
        )
        self.current_step = 0
        self._publish_target_marker()
        return self._get_observation(), {}

    def step(self, action):
        delta_theta = action * 0.1
        target_angles = self.joint_positions.copy()
        target_angles[:3] += delta_theta
        target_angles[:3] = np.clip(target_angles[:3], -3.14, 3.14)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [f"joint{i+1}" for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = target_angles.tolist()
        point.time_from_start = rclpy.duration.Duration(seconds=0.2).to_msg()
        traj_msg.points = [point]
        self.joint_cmd_pub.publish(traj_msg)

        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.end_effector_pos = self.fk(self.joint_positions)

        distance_to_target = np.linalg.norm(
            self.end_effector_pos - self.target_position
        )
        reward = self._calculate_reward(distance_to_target)

        if distance_to_target < self.success_threshold:
            self.success_counter += 1
        if self.curriculum_stage == 1 and self.success_counter > 20:
            self.curriculum_stage = 2
        elif self.curriculum_stage == 2 and self.success_counter > 50:
            self.curriculum_stage = 3

        self.current_step += 1
        done = (
            self.current_step >= self.max_steps
            or distance_to_target < self.success_threshold
        )
        truncated = self.current_step >= self.max_steps
        self._publish_target_marker()
        return self._get_observation(), reward, done, truncated, {}

    def _calculate_reward(self, distance):
        rewards = {}
        rewards["distance"] = -distance * 5
        progress = (self.previous_distance - distance) * 100
        rewards["progress"] = np.clip(progress, -5, 5)
        self.previous_distance = distance
        rewards["success"] = 10.0 if distance < self.success_threshold else 0.0
        joint_violations = np.maximum(0, np.abs(self.joint_positions[:3]) - 2.8)
        rewards["joints"] = -np.sum(joint_violations) * 20
        if self.curriculum_stage >= 2:
            joint_change = np.linalg.norm(
                self.joint_positions[:3] - self.prev_joints[:3]
            )
            rewards["smoothness"] = -min(joint_change * 5, 2.0)
        else:
            rewards["smoothness"] = 0.0
        self.prev_joints = self.joint_positions.copy()
        return sum(rewards.values())

    def _generate_target_position(self):
        if self.curriculum_stage == 1:
            self.success_threshold = 0.1
            return np.array(
                [np.random.uniform(0.3, 0.4), np.random.uniform(-0.1, 0.1), 0.4]
            )
        elif self.curriculum_stage == 2:
            self.success_threshold = 0.05
            return np.array(
                [
                    np.random.uniform(0.2, 0.5),
                    np.random.uniform(-0.3, 0.3),
                    np.random.uniform(0.3, 0.6),
                ]
            )
        else:
            self.success_threshold = 0.03
            return np.array(
                [
                    np.random.uniform(0.1, 0.6),
                    np.random.uniform(-0.4, 0.4),
                    np.random.uniform(0.2, 0.7),
                ]
            )
    def _publish_target_marker(self):
        """Publish a red sphere marker at target_position"""
        marker = Marker()
        marker.header.frame_id = "base_link"  # Adjust to your fixed frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.target_position[0])
        marker.pose.position.y = float(self.target_position[1])
        marker.pose.position.z = float(self.target_position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Sphere size (meters)
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque
        self.marker_pub.publish(marker)

    def fk(self, joint_pos):
        T = robot.fkine(joint_pos)
        return np.array(T.t)

    def _get_observation(self):
        return np.concatenate(
            [self.joint_positions[:3], self.end_effector_pos, self.target_position]
        ).astype(np.float32)

    def close(self):
        self.destroy_node()
        rclpy.shutdown()
        # Wait for the spin thread to finish
        self.spin_thread.join()
'''

###############################################################################################################
class RobotArmEnv(gym.Env, Node):
    def __init__(self):
        gym.Env.__init__(self)
        Node.__init__(self, "my_arm_rl_env")

        # Action space: 3 joint postion(x,y,z)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )

        # Observation space: 3 joint positions + 3 end-effector position + 3 target position
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32
        )

        self.state_lock = threading.Lock()

        # ROS2 Publishers and Subscribers
        # self.joint_cmd_pub = self.create_publisher(
        #     Float64MultiArray, "/arm_controller/commands", 10
        # )

        self.marker_pub = self.create_publisher(Marker, "/target_marker", 10)

        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # State variables
        self.joint_positions = np.zeros(6)
        # self.joint_velocities = np.zeros(6)
        self.end_effector_pos = np.zeros(3)  # End-effector position in cartesian space
        # Simple target in cartesian space
        self.target_position = np.zeros(3)

        # Episode variables
        self.max_steps = 1000
        self.current_step = 0
        self.previous_distance = float("inf")

        # Start ROS2 spinning in separate thread
        self.spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True
        )
        self.spin_thread.start()

        # ros2 run tf2_tools view_frames
        # ros2 run tf2_tools tf2_monitor base_link ee_link

    def joint_state_callback(self, msg):
        """Update robot state from ROS2 topic
        :param msg: JointState message containing joint positions and velocities
        in joint space
        :type msg: sensor_msgs.msg.JointState
        """
        with self.state_lock:
            self.joint_positions = np.array(msg.position[:6])
            # self.joint_velocities = (
            #     np.array(msg.velocity[:6]) if msg.velocity else np.zeros(6)
            # )

    def fk(self, joint_pos):
        """
        Forward kinematics to get end-effector position
        :param joint_pos: Joint angles in radians
        :return: End-effector position in cartesian space
        """
        T = robot.fkine(joint_pos)
        return np.array(T.t)

    def reset(self, seed=None, options=None):
        """Reset environment to initial state"""
        super().reset(seed=seed)

        # Step 1: Move to home joint configuration
        home_angles = [0.0] * 6  # All joints to 0 radians
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        point = JointTrajectoryPoint()
        point.positions = home_angles
        point.time_from_start = rclpy.duration.Duration(seconds=1.5).to_msg()
        traj_msg.points = [point]
        self.joint_cmd_pub.publish(traj_msg)

        # Step 2: Wait and ensure joint states are updated
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        # step3: Random target position (biased to right side)
        self.target_position = np.array(
            [
                np.random.uniform(0.14, 0.29),  # x
                np.random.uniform(-0.2, 0.2),   # y (biased to right side)
                np.random.uniform(0.06, 0.36),  # z
            ]
        )

        # step4: Update state to home position
        with self.state_lock:
            self.end_effector_pos = self.fk(home_angles)

        self.previous_distance = np.linalg.norm(
            self.end_effector_pos - self.target_position
        )

        self.current_step = 0

        self._publish_target_marker()
        return self._get_observation(), {}

    def step(self, action):
        """Execute action and return new state"""
        # Scale action: ∆θ per joint, radians
        delta_theta = action * 0.1  # max ±0.1 rad per step

        # Copy full joint array so we preserve joint4–6
        with self.state_lock:
            target_angles = self.joint_positions.copy()
            # Apply ∆θ to only joint1–3
        target_angles[:3] += delta_theta
        target_angles[:3] = np.clip(target_angles[:3], -3.14, 3.14)

        # Send command to robot
        # cmd_msg = Float64MultiArray()
        # cmd_msg.data = target_angles.tolist()
        # self.joint_cmd_pub.publish(cmd_msg)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        point = JointTrajectoryPoint()
        point.positions = target_angles.tolist()
        point.time_from_start = rclpy.duration.Duration(seconds=0.2).to_msg()
        traj_msg.points = [point]
        self.joint_cmd_pub.publish(traj_msg)

        # Spin node to process updates
        # Wait longer and verify the robot actually moved
        with self.state_lock:
            old_positions = self.joint_positions.copy()

        start_time = time.time()
        while time.time() - start_time < 0.5:  # Increase timeout
            rclpy.spin_once(self, timeout_sec=0.05)
            if np.linalg.norm(self.joint_positions - old_positions) > 0.01:
                break  # Robot has moved
            time.sleep(0.02)

        # Calculate reward
        with self.state_lock:
            self.end_effector_pos = self.fk(self.joint_positions)
        reward = self._calculate_reward()
        distance_to_target = np.linalg.norm(
            self.end_effector_pos - self.target_position
        )
        # Check if done
        self.current_step += 1

        done = (
            self.current_step >= self.max_steps
            or distance_to_target < 0.05  # Success threshold
            or self._check_collision()
        )

        truncated = self.current_step >= self.max_steps
        self._publish_target_marker()
        return self._get_observation(), reward, done, truncated, {}

    def _publish_target_marker(self):
        """Publish a red sphere marker at target_position"""
        marker = Marker()
        marker.header.frame_id = "base_link"  # Adjust to your fixed frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.target_position[0])
        marker.pose.position.y = float(self.target_position[1])
        marker.pose.position.z = float(self.target_position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Sphere size (meters)
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque
        self.marker_pub.publish(marker)

    def _get_observation(self):
        """Get current state observation"""
        with self.state_lock:
            obs = np.concatenate(
                [self.joint_positions[:3], self.end_effector_pos, self.target_position]
            )
        return obs.astype(np.float32)

    def _calculate_reward(self):
        """Adaptive reward function that balances components"""

        distance_to_target = np.linalg.norm(
            self.end_effector_pos - self.target_position
        )

        # Components with similar scales
        rewards = {}

        # Distance (0 to -10 range)
        rewards["distance"] = -distance_to_target * 10

        # Progress (-5 to +5 range)
        progress = (self.previous_distance - distance_to_target) * 50
        rewards["progress"] = np.clip(progress, -5, 5)
        self.previous_distance = distance_to_target

        # Success (0 or +10)
        rewards["success"] = 10.0 if distance_to_target < 0.05 else 0.0

        # Joint limits (0 to -10)
        joint_violations = np.maximum(0, np.abs(self.joint_positions) - 2.8)
        rewards["joints"] = -np.sum(joint_violations) * 20

        # Smoothness (0 to -2)
        if hasattr(self, "prev_joints"):
            joint_change = np.linalg.norm(self.joint_positions - self.prev_joints)
            rewards["smoothness"] = -min(joint_change * 5, 2.0)
        else:
            rewards["smoothness"] = 0.0
        self.prev_joints = self.joint_positions.copy()

        # Debug: Print reward breakdown occasionally
        if self.current_step % 50 == 0:
            print(f"Rewards: {rewards}")

        return sum(rewards.values())

    def _check_collision(self):
        """Simple collision detection"""
        # Check joint limits
        if np.any(np.abs(self.joint_positions) > 3.0):
            return True

        # Check if end-effector is too low
        if self.end_effector_pos[2] < 0.1:
            return True

        return False

    def close(self):
        self.destroy_node()
        self.spin_thread.join()
        rclpy.shutdown()
