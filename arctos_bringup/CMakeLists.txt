cmake_minimum_required(VERSION 3.8)
project(arctos_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(auto_home_client src/auto_home_client.cpp)
ament_target_dependencies(auto_home_client rclcpp std_srvs)

install(
  TARGETS auto_home_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/aruco_pose_node.py
    scripts/charuco_pose_node.py
    scripts/pose_stabilizer.py
    scripts/vision_tf_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch scripts worlds
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
