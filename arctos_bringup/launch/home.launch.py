# The MoveIt‑friendly startup sequence (final version)

# Here’s the exact order your system should start in:
# ✅ Step 1 — Start robot description

# URDF + robot_state_publisher
# ✅ Step 2 — Start ros2_control

# This loads your hardware plugin and connects to CAN.
# ✅ Step 3 — Load controllers

#     joint_state_broadcaster

#     arm_controller

#     gripper_controller

# ✅ Step 4 — Start homing/debug node

# This exposes /go_home, /dump_motors, /wait_all_converged.
# ✅ Step 5 — (Optional) Auto‑home

# Call /go_home once.
# ✅ Step 6 — Start MoveIt

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('arctos_bringup')
    pkg_moveit  = get_package_share_directory('arctos_moveit_config')

    # 1) Robot description
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'description.launch.py'))
    )

    # 2) ros2_control (hardware + controllers)
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'ros2_control.launch.py'))
    )

    # 3) Homing/debug node
    homing_node = Node(
        package='arctos_homing',
        executable='arctos_homing_node',
        name='arctos_homing_node',
        output='screen'
    )

    # 4) Optional auto-home client
    auto_home = Node(
        package='arctos_bringup',
        executable='auto_home_client',
        name='auto_home_client',
        output='screen'
    )

    # 5) MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py'))
    )

    ld = LaunchDescription()
    ld.add_action(description)
    ld.add_action(ros2_control)
    ld.add_action(homing_node)
    # ld.add_action(auto_home)  # enable if you want auto-homing
    ld.add_action(moveit)

    return ld
