from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "is_sim",
            default_value="true",
            description="Start robot in Gazebo simulation.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware mirroring command.",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="true",
            description="Enable fake sensor commands.",
        ),
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyUSB0",
            description="Port name for hardware connection.",
        ),
    ]
    
    # Launch configurations
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    is_sim = LaunchConfiguration('is_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotarm_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_moveit"),
            "launch",
            "moveit.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arctos_remote"),
            "launch",
            "alexa.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            controller,
            gazebo,
            moveit,
            # remote_interface,
        ]
    )
