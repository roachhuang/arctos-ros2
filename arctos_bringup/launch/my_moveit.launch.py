from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():           
    urdf_path = os.path.join(
        get_package_share_directory('arctos_description'),
        'urdf',
        'arctos.urdf.xacro'
    )
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    rviz_config_path = '/home/roach/ros2_ws/src/arctos/arctos_description/rviz/default.rviz'
    
    # ROS2 controllers configuration file path
    yaml_path = os.path.join(
        # get_package_share_directory("arctos_bringup"), "config","real_controllers.yaml"
        get_package_share_directory("arctos_bringup"), "config","ros2_controllers.yaml"
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        # output='both'
    )
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            yaml_path,
        ],
    )                       
      
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
    
    # Spawn arm controller after joint state broadcaster
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
 
    
    # Spawn gripper controller last
    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )
 
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arctos_moveit_config"),
            "launch",
            "move_group.launch.py",
        ),
    )

    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arctos_remote"),
            "launch",
            "alexa.launch.py",
        ),
    )
    
    spawn_commander = Node(
        package="arctos_commander_cpp",
        executable="spawner",      
    )
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            spawn_joint_state_broadcaster,
            spawn_arm_controller,
            spawn_gripper_controller,   
            moveit,
            rviz_node
            # remote_interface,            
        ]
    )
