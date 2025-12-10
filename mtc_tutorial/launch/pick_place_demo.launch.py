from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_cfg = MoveItConfigsBuilder(
        "arctos", package_name="arctos_moveit_config"
    ).to_moveit_configs().to_dict()
   
    # Launch mtc_node after action server is ready
    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[moveit_cfg],
    )

    return LaunchDescription([
        mtc_node,
    ])
