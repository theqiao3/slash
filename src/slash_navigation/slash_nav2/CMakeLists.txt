cmake_minimum_required(VERSION 3.8)
project(slash_nav2)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

find_package(ament_cmake_auto REQUIRED)

# 安装Python脚本
install(PROGRAMS
  scripts/publish_pcd.py
  scripts/pcd_to_pgm.py
  scripts/cmd_vel_relay.py
  scripts/vel_corrector.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  map
  PCD
  rviz
  urdf
  scripts
)

