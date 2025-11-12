cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  #moveit_core
  #moveit_ros_planning_interface
  #std_msgs
  #geometry_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()


