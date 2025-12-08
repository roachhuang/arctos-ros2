from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake sensor commands.',
        ),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='initial_positions.yaml',
            description='Path to the initial position file',
        ),
    ]

    # Launch configurations
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    init_position_file = LaunchConfiguration('init_position_file')

    # Generate URDF file using xacro
    urdf_file = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('arctos_description'),
            'urdf',
            'arctos.urdf.xacro',
        ]), ' ',
        'use_sim:=', use_sim, ' ',
        'use_fake_hardware:=', use_fake_hardware, ' ',
        'fake_sensor_commands:=', fake_sensor_commands,
    ])

    # Paths for configuration files
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('arctos_bringup'),
        'config',
        'ros2_controllers.yaml',
    ])

    # Paths
    pkg_desc = get_package_share_directory('arctos_description')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'arctos.urdf.xacro')
    # rviz_config_path = os.path.join(pkg_desc, 'rviz', 'mtc.rviz')
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('arctos_description'),
        'rviz',
        'mtc.rviz',
    ])

    # robot_description as Command(xacro)
    # robot_description_content = {'robot_description': Command(['xacro ', urdf_path])}

    # RViz (optional visualization)
    rviz_node = Node(
        package='rviz2',
        arguments=['-d', rviz_config_file],
        executable='rviz2',
        output='both',
    )

    # define nodes
    # Static TF from world to base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        # arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "base_link"]

    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_fake_hardware}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        # {"is_sim": use_fake_hardware},
    )
    
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    # arm_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['arm_controller'],
    #     output='screen'
    # )
    # gripper_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['gripper_controller'],
    #     output='screen'
    # )
    # jsb_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen'
    # )

    # Include MoveIt move_group launch
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arctos_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        )
    )

    # Event handlers to ensure order of execution
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner, on_exit=[rviz_node]
        )
    )

    ld = LaunchDescription(
        declared_arguments
        +[
            controller_manager_node,
            robot_controller_spawner,
            
            robot_state_publisher_node,
            # rviz_node,
            static_tf,
            # spawn_joint_state_broadcaster,
            # arm_spawner,
            # gripper_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            moveit,
            # executor_node,
        ]
    )
    return ld