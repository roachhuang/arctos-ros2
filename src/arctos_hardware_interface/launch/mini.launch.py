import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name = "arctos"  # Replace with your robot name
    package_name = "arctos_description"  # Replace with your description package

    robot_description_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "arctos.urdf.xacro",  # Or your URDF file
    )

    robot_description = {
        "robot_description": Command(["xacro ", robot_description_path])
    }

    # gazebo_ros2_ctrl = Node(
    #     package="gazebo_ros2_control",
    #     executable="gazebo_ros2_control_node",
    #     name="ros2_control_node",
    #     output="screen",
    #     parameters=[
    #         {"robot_description": "robot_description"},
    #         "robotarm_controllers.yaml",
    #     ],
    # )

    controller_config_path = os.path.join(
        get_package_share_directory(
            "robotarm_controller"
        ),  # Replace with your controller package
        "config",
        "robotarm_controllers.yaml",  # Replace with your controllers config file
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_path],
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    return LaunchDescription([control_node])
