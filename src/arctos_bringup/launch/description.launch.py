# arctos_bringup/launch/description.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_bringup = get_package_share_directory('arctos_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz with robot model'
    )

    # Path to your URDF/Xacro
    urdf_path = os.path.join(pkg_bringup, 'urdf', 'arctos.urdf.xacro')

    # Generate robot_description parameter
    robot_description = {
        'robot_description': Command([
            'xacro ', urdf_path,
            ' use_sim_time:=', use_sim_time
        ])
    }

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher (optional, useful for debugging)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=None  # always on; ros2_control will override real joints
    )

    # Optional RViz
    rviz_config = os.path.join(pkg_bringup, 'config', 'arctos.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(rviz_node)

    return ld
