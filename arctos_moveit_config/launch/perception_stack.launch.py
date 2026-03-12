"""Launch MoveIt move_group and the Kinect perception stack."""


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def _maybe_include_kinect(context):
    if LaunchConfiguration("use_kinect").perform(context).lower() not in ("1", "true", "yes", "on"):
        return []

    try:
        kinect_share = get_package_share_directory("kinect_ros2")
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    "kinect_ros2 is not installed in the current environment; "
                    "skipping Kinect launch. Build/source kinect_ros2 or pass use_kinect:=false."
                )
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    kinect_share,
                    "launch",
                    "pointcloud.launch.py",
                )
            ),
            condition=IfCondition(LaunchConfiguration("use_kinect")),
        )
    ]

def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_kinect = LaunchConfiguration("use_kinect")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    use_vision_guided_pick = LaunchConfiguration("use_vision_guided_pick")
    use_pca_grasp = LaunchConfiguration("use_pca_grasp")
    vision_pick_execute = LaunchConfiguration("vision_pick_execute")
    vision_pick_object_pose_topic = LaunchConfiguration("vision_pick_object_pose_topic")
    pca_pointcloud_topic = LaunchConfiguration("pca_pointcloud_topic")
    pca_camera_optical_frame = LaunchConfiguration("pca_camera_optical_frame")
    pca_approach_axis = LaunchConfiguration("pca_approach_axis")
    rgb_to_depth_x = LaunchConfiguration("rgb_to_depth_x")
    rgb_to_depth_y = LaunchConfiguration("rgb_to_depth_y")
    rgb_to_depth_z = LaunchConfiguration("rgb_to_depth_z")
    rgb_to_depth_roll = LaunchConfiguration("rgb_to_depth_roll")
    rgb_to_depth_pitch = LaunchConfiguration("rgb_to_depth_pitch")
    rgb_to_depth_yaw = LaunchConfiguration("rgb_to_depth_yaw")

    camera_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arctos_moveit_config"),
                "launch",
                "camera_pose.launch.py",
            )
        ),
        launch_arguments={
            "rgb_to_depth_x": rgb_to_depth_x,
            "rgb_to_depth_y": rgb_to_depth_y,
            "rgb_to_depth_z": rgb_to_depth_z,
            "rgb_to_depth_roll": rgb_to_depth_roll,
            "rgb_to_depth_pitch": rgb_to_depth_pitch,
            "rgb_to_depth_yaw": rgb_to_depth_yaw,
        }.items(),
    )

    # static_tf_x = LaunchConfiguration("static_tf_x")
    # static_tf_y = LaunchConfiguration("static_tf_y")
    # static_tf_z = LaunchConfiguration("static_tf_z")
    # static_tf_qx = LaunchConfiguration("static_tf_qx")
    # static_tf_qy = LaunchConfiguration("static_tf_qy")
    # static_tf_qz = LaunchConfiguration("static_tf_qz")
    # static_tf_qw = LaunchConfiguration("static_tf_qw")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arctos_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config,
            "use_vision_guided_pick": use_vision_guided_pick,
            "use_pca_grasp": use_pca_grasp,
            "vision_pick_execute": vision_pick_execute,
            "vision_pick_object_pose_topic": vision_pick_object_pose_topic,
            "pca_pointcloud_topic": pca_pointcloud_topic,
            "pca_camera_optical_frame": pca_camera_optical_frame,
            "pca_approach_axis": pca_approach_axis,
            # "static_tf_x": static_tf_x,
            # "static_tf_y": static_tf_y,
            # "static_tf_z": static_tf_z,
            # "static_tf_qx": static_tf_qx,
            # "static_tf_qy": static_tf_qy,
            # "static_tf_qz": static_tf_qz,
            # "static_tf_qw": static_tf_qw,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_kinect", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_vision_guided_pick", default_value="false"),
            DeclareLaunchArgument("use_pca_grasp", default_value="false"),
            DeclareLaunchArgument("vision_pick_execute", default_value="false"),
            DeclareLaunchArgument("vision_pick_object_pose_topic", default_value="/detected_object_pose"),
            DeclareLaunchArgument("pca_pointcloud_topic", default_value="/point_cloud"),
            DeclareLaunchArgument("pca_camera_optical_frame", default_value="camera_rgb_optical_frame"),
            DeclareLaunchArgument("pca_approach_axis", default_value="largest"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    get_package_share_directory("arctos_moveit_config"),
                    "config",
                    "moveit_safe.rviz",
                ),
            ),
            # DeclareLaunchArgument("static_tf_x", default_value="0.5"),
            # DeclareLaunchArgument("static_tf_y", default_value="0.0"),
            # DeclareLaunchArgument("static_tf_z", default_value="0.6"),
            # DeclareLaunchArgument("static_tf_qx", default_value="0.0"),
            # DeclareLaunchArgument("static_tf_qy", default_value="0.0"),
            # DeclareLaunchArgument("static_tf_qz", default_value="0.0"),
            # DeclareLaunchArgument("static_tf_qw", default_value="1.0"),
            DeclareLaunchArgument("rgb_to_depth_x", default_value="0.0"),
            DeclareLaunchArgument("rgb_to_depth_y", default_value="0.0"),
            DeclareLaunchArgument("rgb_to_depth_z", default_value="0.0"),
            DeclareLaunchArgument("rgb_to_depth_roll", default_value="0.0"),
            DeclareLaunchArgument("rgb_to_depth_pitch", default_value="0.0"),
            DeclareLaunchArgument("rgb_to_depth_yaw", default_value="0.0"),
            camera_pose,
            moveit_launch,
            OpaqueFunction(function=_maybe_include_kinect),
        ]
    )
