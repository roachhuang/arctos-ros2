import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Set to true to use fake hardware interface'
    )
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    urdf_path = os.path.join(
        get_package_share_directory('arctos_description'),
        'urdf',
        'arctos.urdf.xacro'
    )
    robot_description_content = Command(['xacro ', urdf_path])

    controller_yaml = os.path.join(
        get_package_share_directory('arctos_bringup'),
        'config',
        'ros2_controllers.yaml'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_fake_hardware': use_fake_hardware},
            controller_yaml,
        ],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '60'],
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager-timeout', '60'],
    )

    moveit_include = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('arctos_moveit_config'),
            'launch',
            'move_group.launch.py',
        )
    )

    return LaunchDescription([
        declare_use_fake_hardware,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
        moveit_include,
    ])
