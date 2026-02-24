""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> kinect_rgb """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rgb_to_depth_x = LaunchConfiguration("rgb_to_depth_x")
    rgb_to_depth_y = LaunchConfiguration("rgb_to_depth_y")
    rgb_to_depth_z = LaunchConfiguration("rgb_to_depth_z")
    rgb_to_depth_roll = LaunchConfiguration("rgb_to_depth_roll")
    rgb_to_depth_pitch = LaunchConfiguration("rgb_to_depth_pitch")
    rgb_to_depth_yaw = LaunchConfiguration("rgb_to_depth_yaw")

    nodes = [
        DeclareLaunchArgument("rgb_to_depth_x", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_y", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_z", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_roll", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_pitch", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_yaw", default_value="0.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_tokinect",
            output="log",
            arguments=[
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "kinect_rgb",
                "--x",
                "0.0706151",
                "--y",
                "-0.660678",
                "--z",
                "1.03902",
                "--qx",
                "0.948558",
                "--qy",
                "0.0556392",
                "--qz",
                "-0.0484424",
                "--qw",
                "-0.307888",
                # "--roll",
                # "0.622871",
                # "--pitch",
                # "-3.01509",
                # "--yaw",
                # "3.06518",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_kinect_rgb_to_depth",
            output="log",
            arguments=[
                "--frame-id", "kinect_rgb",
                "--child-frame-id", "kinect_depth",
                "--x", rgb_to_depth_x,
                "--y", rgb_to_depth_y,
                "--z", rgb_to_depth_z,
                "--roll", rgb_to_depth_roll,
                "--pitch", rgb_to_depth_pitch,
                "--yaw", rgb_to_depth_yaw,
            ],
        ),
    ]
    return LaunchDescription(nodes)
