import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link"
        ],
    )

    # --- ros2_control ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
    )

    # --- Controllers ---
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "60"],
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "60"],
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
        robot_state_publisher,
        static_tf,
        ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        move_group,
        # rviz,
    ])
