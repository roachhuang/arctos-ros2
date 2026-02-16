from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_cfg = MoveItConfigsBuilder(
        "arctos", package_name="arctos_moveit_config"
    ).to_dict()

    use_detected_object_pose = LaunchConfiguration("use_detected_object_pose")
    detected_pose_topic = LaunchConfiguration("detected_pose_topic")
    detection_wait_timeout_sec = LaunchConfiguration("detection_wait_timeout_sec")
    start_pose_stabilizer = LaunchConfiguration("start_pose_stabilizer")
    raw_detected_pose_topic = LaunchConfiguration("raw_detected_pose_topic")
    stable_repeat_count = LaunchConfiguration("stable_repeat_count")
    stable_position_tolerance = LaunchConfiguration("stable_position_tolerance")
    stable_timeout_sec = LaunchConfiguration("stable_timeout_sec")

    # Optional pre-processing node: turns noisy raw detections into a stable pose topic.
    pose_stabilizer = Node(
        package="arctos_bringup",
        executable="pose_stabilizer.py",
        output="screen",
        condition=IfCondition(start_pose_stabilizer),
        parameters=[
            {
                "input_topic": raw_detected_pose_topic,
                "output_topic": detected_pose_topic,
                "stable_repeat_count": stable_repeat_count,
                "stable_position_tolerance": stable_position_tolerance,
                "stable_timeout_sec": stable_timeout_sec,
            }
        ],
    )

    # Main runtime node. In detection mode, consume the (typically stabilized) pose topic.
    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_cfg,
            {
                "use_detected_object_pose": use_detected_object_pose,
                "detected_pose_topic": detected_pose_topic,
                "detection_wait_timeout_sec": detection_wait_timeout_sec,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_detected_object_pose", default_value="false"),
        # Keep enabled for default minimal runtime path.
        DeclareLaunchArgument("start_pose_stabilizer", default_value="true"),
        DeclareLaunchArgument("raw_detected_pose_topic", default_value="/detected_object_pose"),
        DeclareLaunchArgument("detected_pose_topic", default_value="/detected_object_pose_stable"),
        DeclareLaunchArgument("stable_repeat_count", default_value="3"),
        DeclareLaunchArgument("stable_position_tolerance", default_value="0.02"),
        DeclareLaunchArgument("stable_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("detection_wait_timeout_sec", default_value="10.0"),
        pose_stabilizer,
        mtc_node,
    ])
