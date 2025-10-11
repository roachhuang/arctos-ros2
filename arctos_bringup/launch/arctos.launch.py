from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os

def generate_launch_description():
    '''
    ros2 launch your_package your_main_launch.py use_sim:=true will set is_sim to true in all the included launch files.
   
    '''
    
    robot_description_path = get_package_share_directory("arctos_description")
    urdf_path = os.path.join(robot_description_path, 'urdf', "arctos.urdf.xacro")
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
       
    robot_bringup_path = get_package_share_directory("arctos_bringup")
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'default.rviz')
    controllers_cfg_path= os.path.join(robot_bringup_path, 'config', 'real_controllers.yaml')
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_description_content,
                "publish_frequency": 10.0,
            }
        ],
    )
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_cfg_path],
        output="screen"  
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )      
     
    # gripper_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["gripper_controller"],
    #     output="screen",
    # )
           
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )
    
    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            # gripper_controller_spawner,
            rviz_node,
        ]
    )
