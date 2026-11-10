cmake_minimum_required(VERSION 3.8)
project(arctos_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arctos_fndm_interface REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)

# set dependencies
set(dependencies
rclcpp
rclcpp_action
rclcpp_components
arctos_fndm_interface
moveit_ros_planning_interface
rclpy   # added so Python dependency is exported
)

#include directories
include_directories(include)  # changed from include/task_server to top-level include
# include_directories(include/task_server)

# add executables for scripts
add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server ${dependencies})

ament_python_install_package(${PROJECT_NAME})

# Install directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install the scripts
install(TARGETS
  task_server
  # fibonacci_client
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/alexa_interface.py
  DESTINATION lib/${PROJECT_NAME}
  )

# export dependencies
ament_export_dependencies(${dependencies})
ament_package()
