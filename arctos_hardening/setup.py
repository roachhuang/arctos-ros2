from setuptools import setup

package_name = 'arctos_hardening'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arctos_hardening.launch.py']),
        ('share/' + package_name + '/config/hardening', [
            'config/hardening/safety_gates.yaml',
            'config/hardening/tf_watchdog.yaml',
            'config/hardening/eigen_monitor.yaml',
            'config/hardening/moveit_runtime.yaml',
            'config/hardening/trajectory_guard.yaml',
            'config/hardening/eigen_input_bridge.yaml',
            'config/hardening/heuristic_eigen_source.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'safety_watchdog_node = arctos_hardening.safety_watchdog_node:main',
            'eigen_monitor_node = arctos_hardening.eigen_monitor_node:main',
            'trajectory_guard_node = arctos_hardening.trajectory_guard_node:main',
            'eigen_input_bridge_node = arctos_hardening.eigen_input_bridge_node:main',
            'heuristic_eigen_source_node = arctos_hardening.heuristic_eigen_source_node:main',
        ],
    },
)
