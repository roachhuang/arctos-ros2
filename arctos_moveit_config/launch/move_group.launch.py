from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config").to_moveit_configs()
    # 2. Add the MTC Capability
    moveit_config.move_group_capabilities["capabilities"]="move_group/ExecuteTaskSolutionCapability"
    moveit_config.trajectory_execution["allow_trajectory_execution"] = True
    moveit_config.trajectory_execution["execution_duration_monitoring"] = False
    # 4. CRITICAL: Override Planning Scene Monitor to fix Segfault at 0xe8
    # Lower frequency (5Hz) prevents pointer invalidation during real-world execution
    moveit_config.planning_scene_monitor["publish_planning_scene_hz"] = 5.0
    moveit_config.planning_scene_monitor["monitor_dynamics"] = False
    
    # 5. Fix Start State Errors for real hardware drift
    # This prevents MoveIt from crashing if the real robot is slightly off the planned start
    moveit_config.move_group_capabilities["start_state_max_bounds_error"] = 0.05

    return generate_move_group_launch(moveit_config)
