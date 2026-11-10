import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path

def generate_launch_description():
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
    
    # Declare the model argument
    model_argument = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_directory('arctos_description'),
            'urdf',
            # 's.urdf'  # Corrected file name
            'arctos.urdf.xacro'  # Corrected file name
        ),
        description='The robot model to load'
    )

    # Generate the robot description from the xacro file
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        # output='both'
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # output='both'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('arctos_description'),
            'rviz',
            # 'arctos.rviz'
        )]
    )

    return LaunchDescription([
        gazebo_resource_path,
        model_argument,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])