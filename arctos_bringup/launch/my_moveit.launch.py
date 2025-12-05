from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():
    # Arguments
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Set to true to use fake hardware interface'
    )
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Paths
    pkg_desc = get_package_share_directory('arctos_description')
    urdf_path = os.path.join(pkg_desc, 'urdf', 'arctos.urdf.xacro')
    rviz_config_path = os.path.join(pkg_desc, 'rviz', 'mtc.rviz')

    # robot_description as Command(xacro)
    robot_description_content = {'robot_description': Command(['xacro ', urdf_path])}

    # RViz (optional visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # Static TF from world to base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_content],
    )

    # ros2_control controller manager
    yaml_path = os.path.join(
        get_package_share_directory("arctos_bringup"),
        "config",
        "ros2_controllers.yaml"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output='screen',
        parameters=[
            robot_description_content,
            {"use_fake_hardware": use_fake_hardware},
            yaml_path,
        ],
    )

    # Spawners: delay them slightly so controller_manager has time to be ready.
    spawn_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output='screen',
                arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
            )
        ],
    )

    spawn_arm_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output='screen',
                arguments=["arm_controller", "--controller-manager-timeout", "60"],
            )
        ],
    )

    spawn_gripper_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output='screen',
                arguments=["gripper_controller", "--controller-manager-timeout", "60"],
            )
        ],
    )

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

    # Conditionally add MoveIt Task Constructor executor if available
    executor_node = None
    try:
        get_package_share_directory('moveit_task_constructor_executor')
        executor_node = Node(
            package='moveit_task_constructor_executor',
            executable='moveit_task_constructor_executor',
            output='screen',
            name='moveit_task_constructor_executor',
            parameters=[robot_description_content],
        )
    except PackageNotFoundError:
        # don't fail launch if executor isn't installed; user can run it manually
        executor_node = LogInfo(msg='moveit_task_constructor_executor not found; will not start it here.')

    ld = LaunchDescription([
        declare_use_fake_hardware,
        rviz_node,
        static_tf,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
        moveit,
        executor_node,
    ])

    return ld
