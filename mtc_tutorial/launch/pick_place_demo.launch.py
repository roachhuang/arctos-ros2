from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # MTC Demo node with delay to ensure MoveIt action server is ready
    # Note: move_group provides the execute_task_solution action server
    pick_place_demo = TimerAction(
        period=8.0,  # Wait longer for move_group to be fully ready
        actions=[
            Node(
                package="mtc_tutorial",
                executable="mtc_node",
                output="screen",
            )
        ]
    )

    return LaunchDescription([pick_place_demo])