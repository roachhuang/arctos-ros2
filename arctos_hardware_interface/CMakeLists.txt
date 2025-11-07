cmake_minimum_required(VERSION 3.8)
project(arctos_hardware_interface)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  # For lifecycle components
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/arctos_hardware_interface.cpp
  src/servo_can.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  # ${LibSerial_INCLUDE_DIRS}
  "${SERIAL_INCLUDE_DIRS}"
)

# target_link_libraries(${PROJECT_NAME}
#   # "${LibSerial_LIBRARIES}"
#   "${SERIAL_LDFLAGS}"
# )

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface arctos_hardware_interface.xml)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  # DESTINATION include/${PROJECT_NAME}
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
