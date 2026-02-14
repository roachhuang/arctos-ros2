import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Paths ---
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("arctos_description"),
        "urdf",
        "arctos.urdf.xacro"
    ])

    controllers_yaml = os.path.join(
        get_package_share_directory("arctos_bringup"),
        "config",
        "ros2_controllers.yaml"
    )

    # rviz_config_path = os.path.join(
    #     get_package_share_directory("arctos_description"),
    #     "rviz",
    #     "mtc.rviz"
    # )

    # --- robot_state_publisher (ONLY URDF OWNER) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_xacro])
        }]
    )

    # --- Static TF ---
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link"
        ],
    )

    # --- ros2_control ---
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
        condition=IfCondition(use_ros2_control),
    )

    # --- Controllers ---
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        condition=IfCondition(use_ros2_control),
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "60"],
        condition=IfCondition(use_ros2_control),
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "60"],
        condition=IfCondition(use_ros2_control),
    )

    use_fake_joint_states = LaunchConfiguration("use_fake_joint_states")

    # Optional fallback for perception-only sessions where ros2_control is not running.
    # This keeps robot link TFs available so MoveIt octomap filtering can update.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_xacro])
        }],
        condition=IfCondition(use_fake_joint_states),
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_kinect = LaunchConfiguration("use_kinect")
    rgb_to_depth_x = LaunchConfiguration("rgb_to_depth_x")
    rgb_to_depth_y = LaunchConfiguration("rgb_to_depth_y")
    rgb_to_depth_z = LaunchConfiguration("rgb_to_depth_z")
    rgb_to_depth_roll = LaunchConfiguration("rgb_to_depth_roll")
    rgb_to_depth_pitch = LaunchConfiguration("rgb_to_depth_pitch")
    rgb_to_depth_yaw = LaunchConfiguration("rgb_to_depth_yaw")

    camera_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arctos_bringup"),
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

    # --- MoveIt (NO URDF INJECTION) ---
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arctos_moveit_config"),
                "launch",
                "perception_stack.launch.py",
                # "move_group.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_kinect": use_kinect,
        }.items(),
        # launch_arguments={
        #     # Critical: prevent MoveIt from redefining URDF
        #     "publish_robot_description": "false",
        #     "publish_robot_description_semantic": "false",
        # }.items(),
    )

    # --- RViz ---
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz_config_path],
    #     output="screen",
    # )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_kinect", default_value="true"),
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Start ros2_control_node and controller spawners",
        ),
        DeclareLaunchArgument(
            "use_fake_joint_states",
            default_value="false",
            description="Publish synthetic joint_states for TF completeness when controllers are unavailable",
        ),
        DeclareLaunchArgument("rgb_to_depth_x", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_y", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_z", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_roll", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_pitch", default_value="0.0"),
        DeclareLaunchArgument("rgb_to_depth_yaw", default_value="0.0"),
        robot_state_publisher,
        static_tf,
        ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        joint_state_publisher,
        camera_pose,
        move_group,
        # rviz,
    ])
