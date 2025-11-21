from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Generate MoveIt configuration
    moveit_config = (MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
        .robot_description(file_path="config/arctos.urdf.xacro") 
        .robot_description_semantic(file_path="config/arctos.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # .planning_pipelines(["ompl", "pilz_industrial_motion_planner"])
        .joint_limits(file_path="config/joint_limits.yaml")  
        # .trajectory_execution(file_path="config/trajectory_execution.yaml")
        .to_moveit_configs())
    # MTC Tutorial node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            # moveit_config.trajectory_execution,
        ],
    )

    return LaunchDescription([pick_place_demo])