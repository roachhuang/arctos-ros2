cmake_minimum_required(VERSION 3.8)
project(arctos_description)

# Enable compiler warnings (optional, not strictly needed for description packages)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)  # Add xacro dependency
find_package(urdf REQUIRED)   # Add urdf dependency (optional)

# Install mesh files
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  USE_SOURCE_PERMISSIONS
)

# Install directories
install(
  DIRECTORY launch urdf rviz world
  DESTINATION share/${PROJECT_NAME}
)

# Mark the package as an ament package
ament_package()
