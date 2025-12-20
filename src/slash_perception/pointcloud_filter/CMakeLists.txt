cmake_minimum_required(VERSION 3.8)
project(pointcloud_filter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Custom message filter node (for FAST_LIO)
add_executable(custom_msg_filter_node src/custom_msg_filter_node.cpp)
ament_target_dependencies(custom_msg_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  livox_ros_driver2
)

# PointCloud2 radius filter node
add_executable(radius_filter_node src/radius_filter_node.cpp)
ament_target_dependencies(radius_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(radius_filter_node
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  custom_msg_filter_node
  radius_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files if any
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
