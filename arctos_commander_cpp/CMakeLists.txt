cmake_minimum_required(VERSION 3.8)
project(arctos_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(arctos_interfaces REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
  example_interfaces
  arctos_interfaces
)

add_executable(point_and_inspect src/point_and_inspect.cpp)
ament_target_dependencies(point_and_inspect
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(vision_guided_pick src/vision_guided_pick.cpp)
ament_target_dependencies(vision_guided_pick
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(jacobian_twist_controller src/jacobian_twist_controller.cpp)
ament_target_dependencies(jacobian_twist_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
)
target_link_libraries(jacobian_twist_controller Eigen3::Eigen)

add_executable(pca_grasp_node src/pca_grasp_node.cpp)
ament_target_dependencies(pca_grasp_node
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
)
target_link_libraries(pca_grasp_node Eigen3::Eigen)

install(TARGETS   
  test_moveit
  commander
  point_and_inspect
  vision_guided_pick
  jacobian_twist_controller
  pca_grasp_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
