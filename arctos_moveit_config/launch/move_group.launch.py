import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 初始化配置 (確保載入了 URDF, SRDF, Kinematics)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="arctos", package_name="arctos_moveit_config")
        .robot_description(file_path="config/arctos.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/arctos.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor()
        .to_moveit_configs()
    )

    # 2. 準備參數字典 (整合所有內容)
    # to_dict() 已經包含了 robot_description_semantic 的內容
    # common_params = [moveit_config.to_dict()]

    # Launch arguments
    # static_tf_x = LaunchConfiguration("static_tf_x")
    # static_tf_y = LaunchConfiguration("static_tf_y")
    # static_tf_z = LaunchConfiguration("static_tf_z")
    # static_tf_qx = LaunchConfiguration("static_tf_qx")
    # static_tf_qy = LaunchConfiguration("static_tf_qy")
    # static_tf_qz = LaunchConfiguration("static_tf_qz")
    # static_tf_qw = LaunchConfiguration("static_tf_qw")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_vision_guided_pick = LaunchConfiguration("use_vision_guided_pick")
    vision_pick_execute = LaunchConfiguration("vision_pick_execute")
    vision_pick_object_pose_topic = LaunchConfiguration("vision_pick_object_pose_topic")
    # 3. 定義 move_group 節點
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # Mandatory for MTC
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            {"use_sim_time": False},
            {"monitor_dynamics": False},
            # Ensure occupancy map uses a stable planning frame and nonzero voxel resolution.
            {"octomap_frame": "base_link"},
            {"octomap_resolution": 0.03},
            # Keep scene updates flowing to RViz/clients.
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            # Force periodic planning scene publication so /planning_scene isn't empty
            {"publish_planning_scene": True},
            {"publish_planning_scene_hz": 1.0},
        ],
    )

    # Publish robot TFs so MoveIt can transform sensor data into the planning frame
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[moveit_config.robot_description],
    # )

    # Static TF between robot base and Kinect (test values)
    # static_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     output="screen",
    #     arguments=[
    #         static_tf_x,
    #         static_tf_y,
    #         static_tf_z,
    #         static_tf_qx,
    #         static_tf_qy,
    #         static_tf_qz,
    #         static_tf_qw,
    #         "base_link",
    #         "kinect_rgb",
    #     ],
    # )

    # 4. 🔴 修正：定義 RViz 節點並傳入同樣的參數
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[
            moveit_config.to_dict(),
        ], # 這裡必須傳入，RViz 才能解析 SRDF
    )

    vision_guided_pick_node = Node(
        package="arctos_commander_cpp",
        executable="vision_guided_pick",
        name="vision_guided_pick",
        output="screen",
        condition=IfCondition(use_vision_guided_pick),
        parameters=[
            moveit_config.to_dict(),
            {"execute": vision_pick_execute},
            {"target_frame": "base_link"},
            {"object_pose_topic": vision_pick_object_pose_topic},
        ],
    )

    return LaunchDescription([
        # DeclareLaunchArgument("static_tf_x", default_value="0.5"),
        # DeclareLaunchArgument("static_tf_y", default_value="0.0"),
        # DeclareLaunchArgument("static_tf_z", default_value="0.6"),
        # DeclareLaunchArgument("static_tf_qx", default_value="0.0"),
        # DeclareLaunchArgument("static_tf_qy", default_value="0.0"),
        # DeclareLaunchArgument("static_tf_qz", default_value="0.0"),
        # DeclareLaunchArgument("static_tf_qw", default_value="1.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_vision_guided_pick", default_value="false"),
        DeclareLaunchArgument("vision_pick_execute", default_value="false"),
        DeclareLaunchArgument("vision_pick_object_pose_topic", default_value="/detected_object_pose"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("arctos_moveit_config"),
                "config",
                "moveit_safe.rviz",
            ),
        ),
        # robot_state_publisher_node,
        # static_tf_node,
        move_group_node,
        rviz_node,
        vision_guided_pick_node,
    ])
