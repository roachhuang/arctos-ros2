from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, LogInfo
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task_server = Node(
        package="arctos_remote",
        executable="task_server",
        # name="task_server",
    )
    alexa_interface_node = Node(
        package="arctos_remote",
        executable="alexa_interface.py",
        # name="alexa_interface_node",
        # output="screen",
    )

    return LaunchDescription([
        task_server,
        alexa_interface_node,
    ])
