import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown   
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
from launch.conditions import UnlessCondition

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_description():

    # -----------------------------
    # Launch arguments
    # -----------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use ros2_control fake hardware",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Run Gazebo simulation (if used)",
        ),
    ]

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim = LaunchConfiguration("use_sim")

    # -----------------------------
    # Paths
    # -----------------------------
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("arctos_description"),
        "urdf",
        "arctos.urdf.xacro",
    ])

    ros2_controller_yaml = PathJoinSubstitution([
        FindPackageShare("arctos_bringup"),
        "config",
        "ros2_controllers.yaml",
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("arctos_description"),
        "rviz",
        "mtc.rviz",
    ])

    # -----------------------------
    # robot_description (xacro → URDF)
    # -----------------------------
    robot_description_config = Command([
        FindExecutable(name="xacro"),
        " ",
        urdf_xacro,
        " use_fake_hardware:=", use_fake_hardware,
        " use_sim:=", use_sim,
    ])
    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}   
    
    # Load SRDF directly
    srdf_file = os.path.join(
        get_package_share_directory('arctos_moveit_config'),
        'config', 'arctos.srdf'
    )
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
  
    # -----------------------------
    # Nodes
    # -----------------------------
    # TF: world → base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "base_link"],
    )

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"       
    )

    # ros2_control Node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controller_yaml,
            # {"use_fake_hardware": use_fake_hardware},
        ],
        output="screen",
        on_exit=Shutdown(),
    )

    # Spawn controllers in correct order
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    # Load MoveIt configs
    kinematics_yaml = load_yaml(
        'arctos_moveit_config', 'config/kinematics.yaml'
    )

    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml
    }

    joint_limits_yaml = load_yaml(
        'arctos_moveit_config', 'config/joint_limits.yaml'
    )

    joint_limits_config = {
        'robot_description_planning': joint_limits_yaml
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'arctos_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # RViz (start only after controllers are ready)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )
    
    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'arctos_moveit_config', 'config/moveit_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }
      
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("arctos_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        )      
    )

    # ================================================================================
    # EVENT HANDLERS: Chain startup in order
    # Execution order:
    # 1. controller_manager (manual start)
    # 2. joint_state_broadcaster (after controller_manager starts)
    # 3. arm_controller (after jsb_spawner exits)
    # 4. gripper_controller (after arm_spawner exits)
    # 5. moveit (parallel, after gripper spawned)
    # 6. rviz (parallel, after gripper spawned)
    # ================================================================================
    
    # Arm controller starts after joint_state_broadcaster spawner exits
    start_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner]
        )
    )

    # Gripper controller starts after arm_controller spawner exits
    start_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_spawner,
            on_exit=[gripper_spawner]
        )
    )

    return LaunchDescription(
        declared_arguments + [
            static_tf,
            rsp,           
            controller_manager,
            jsb_spawner,          
            start_arm,
            start_gripper,
            moveit,           # Start after gripper (uses same controller manager)
            rviz,             # Start in parallel with moveit (just needs TF + RSP)
        ]
    )