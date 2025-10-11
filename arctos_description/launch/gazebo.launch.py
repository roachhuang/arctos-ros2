import os
from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("arctos_description"),
            "urdf",
            "arctos.urdf.xacro",
        ),
        description="The robot model to load",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("arctos_bringup"), "worlds", "empty_world.world"
        ),
        description="Gazebo Sim world file",
    )

    description_path = os.path.join(
        get_package_share_directory('arctos_description')
    )

    bringup_path = os.path.join(
        get_package_share_directory('arctos_bringup')
    )
    
    # Set gazebo sim resource path, inlcuding subfolder 'worlds' and the description path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(bringup_path, 'worlds'),
            ':' + str(Path(description_path).parent.resolve()),
        ],
    )
    
    
    # Generate the robot description from the xacro file
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")])
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Start Gazebo Sim directly (no ruby, no ros_gz_sim launch)
    start_gazebo = ExecuteProcess(
        cmd=["gz", "sim", LaunchConfiguration("world"), "-r", "-v", "4"],
        output="screen",
    )
    
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    # )


    world_path = PathJoinSubstitution([
        FindPackageShare('arctos_bringup'),
        'worlds',
    ])
    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={
            # 'gz_args': ['-r ', world_path]
            'gz_args': ['-r']
        }.items()
    )

    # bridge_params = os.path.join(
    #     get_package_share_directory('my_robot_description'),
    #     'params',
    #     'my_robot_bridge.yaml'
    # )
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            # f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    # Delay spawn to ensure Gazebo is ready
    gz_spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name", "arctos",
                    "-topic", "robot_description",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.0",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            gazebo_resource_path,
            gz_sim,
            robot_state_publisher_node,
            clock_bridge,
            gz_spawn_entity,
        ]
    )
