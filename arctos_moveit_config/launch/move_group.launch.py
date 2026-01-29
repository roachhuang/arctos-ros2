from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 初始化配置 (確保載入了 URDF, SRDF, Kinematics)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="arctos", package_name="arctos_moveit_config")
        # .robot_description(file_path="urdf/arctos.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/arctos.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .planning_scene_monitor()
        .to_moveit_configs()
    )

    # 2. 準備參數字典 (整合所有內容)
    # to_dict() 已經包含了 robot_description_semantic 的內容
    common_params = [moveit_config.to_dict()]

    # 3. 定義 move_group 節點
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_params,
    )

    # 4. 🔴 修正：定義 RViz 節點並傳入同樣的參數
    rviz_config_file = os.path.join(
        get_package_share_directory("arctos_moveit_config"), "config", "moveit.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=common_params, # 這裡必須傳入，RViz 才能解析 SRDF
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])