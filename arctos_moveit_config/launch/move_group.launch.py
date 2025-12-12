from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config").to_moveit_configs()
    
    moveit_config.move_group_capabilities["capabilities"] = "move_group/ExecuteTaskSolutionCapability"
    moveit_config.trajectory_execution["allow_trajectory_execution"] = True
    moveit_config.trajectory_execution["execution_duration_monitoring"] = False
    
    return generate_move_group_launch(moveit_config)