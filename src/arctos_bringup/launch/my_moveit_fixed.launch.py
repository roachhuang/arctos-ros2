from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('arctos_description'), 'urdf', 'arctos.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_directory('arctos_description'), 'rviz', 'default.rviz')
    controllers_config = os.path.join(get_package_share_directory('arctos_bringup'), 'config', 'ros2_controllers.yaml')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path, ' use_fake_hardware:=true']), value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_config]
    )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller']
    )
    
    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller']
    )
    
    moveit = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('arctos_moveit_config'), 'launch', 'move_group.launch.py')
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        moveit,
        rviz
    ])