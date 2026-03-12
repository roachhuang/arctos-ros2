cmake_minimum_required(VERSION 3.8)
project(arctos_hardening)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    arctos_hardening/safety_watchdog_node.py
    arctos_hardening/eigen_monitor_node.py
    arctos_hardening/trajectory_guard_node.py
    arctos_hardening/eigen_input_bridge_node.py
    arctos_hardening/heuristic_eigen_source_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
