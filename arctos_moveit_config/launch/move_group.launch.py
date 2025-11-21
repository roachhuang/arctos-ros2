from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
    .planning_pipelines(pipelines=["ompl"])
    .robot_description(file_path="config/arctos.urdf.xacro")
    .robot_description_semantic(file_path="config/arctos.srdf")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .to_moveit_configs()                        
    )                 
    return generate_move_group_launch(moveit_config)
