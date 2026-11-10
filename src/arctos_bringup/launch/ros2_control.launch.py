# arctos_bringup/launch/ros2_control.launch.py
# h/w plugin loads 1st
# controllers load only after h/w is ready
# controller names match moveit exactly
# works w/ moveit_controllers.yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('arctos_bringup')

    # Path to ros2_control config
    ros2_control_config = os.path.join(pkg_bringup, 'config', 'ros2_controllers.yaml')

    # 1) Controller Manager (loads hardware + controllers)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_control_config],
        output='screen'
    )

    # 2) Load joint_state_broadcaster
    load_jsb = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    # 3) Load arm controller
    load_arm = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'arm_controller'
        ],
        output='screen'
    )

    # 4) Load gripper controller
    load_gripper = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'gripper_controller'
        ],
        output='screen'
    )

    # Timed loading ensures hardware is initialized before controllers activate
    ld = LaunchDescription()
    ld.add_action(controller_manager)
    ld.add_action(TimerAction(period=2.0, actions=[load_jsb]))
    ld.add_action(TimerAction(period=3.0, actions=[load_arm]))
    ld.add_action(TimerAction(period=3.5, actions=[load_gripper]))

    return ld
