from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    enable_watchdog = LaunchConfiguration('enable_watchdog')
    enable_eigen = LaunchConfiguration('enable_eigen')
    enable_trajectory_guard = LaunchConfiguration('enable_trajectory_guard')
    enable_eigen_bridge = LaunchConfiguration('enable_eigen_bridge')
    enable_heuristic_eigen_source = LaunchConfiguration('enable_heuristic_eigen_source')

    pkg_share = FindPackageShare('arctos_hardening')
    safety_yaml = PathJoinSubstitution([pkg_share, 'config', 'hardening', 'safety_gates.yaml'])
    eigen_yaml = PathJoinSubstitution([pkg_share, 'config', 'hardening', 'eigen_monitor.yaml'])
    guard_yaml = PathJoinSubstitution([pkg_share, 'config', 'hardening', 'trajectory_guard.yaml'])
    bridge_yaml = PathJoinSubstitution([pkg_share, 'config', 'hardening', 'eigen_input_bridge.yaml'])
    heuristic_yaml = PathJoinSubstitution([pkg_share, 'config', 'hardening', 'heuristic_eigen_source.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='hw_active'),
        DeclareLaunchArgument('enable_watchdog', default_value='true'),
        DeclareLaunchArgument('enable_eigen', default_value='true'),
        DeclareLaunchArgument('enable_trajectory_guard', default_value='false'),
        DeclareLaunchArgument('enable_eigen_bridge', default_value='false'),
        DeclareLaunchArgument('enable_heuristic_eigen_source', default_value='false'),

        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='arctos_hardening',
                    executable='safety_watchdog_node.py',
                    name='safety_watchdog_node',
                    namespace='arctos',
                    output='screen',
                    parameters=[safety_yaml],
                    condition=IfCondition(enable_watchdog),
                ),
                Node(
                    package='arctos_hardening',
                    executable='eigen_monitor_node.py',
                    name='eigen_monitor_node',
                    namespace='arctos',
                    output='screen',
                    parameters=[eigen_yaml],
                    condition=IfCondition(enable_eigen),
                ),
                Node(
                    package='arctos_hardening',
                    executable='eigen_input_bridge_node.py',
                    name='eigen_input_bridge_node',
                    namespace='arctos',
                    output='screen',
                    parameters=[bridge_yaml],
                    condition=IfCondition(enable_eigen_bridge),
                ),
                Node(
                    package='arctos_hardening',
                    executable='heuristic_eigen_source_node.py',
                    name='heuristic_eigen_source_node',
                    namespace='arctos',
                    output='screen',
                    parameters=[heuristic_yaml],
                    condition=IfCondition(enable_heuristic_eigen_source),
                ),
                Node(
                    package='arctos_hardening',
                    executable='trajectory_guard_node.py',
                    name='trajectory_guard_node',
                    namespace='arctos',
                    output='screen',
                    parameters=[guard_yaml],
                    condition=IfCondition(enable_trajectory_guard),
                ),
            ],
        ),
    ])
