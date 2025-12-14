cmake_minimum_required(VERSION 3.8)
project(amr_cobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  textures
  DESTINATION share/${PROJECT_NAME}/
)

# Install helper nodes/scripts (rclpy)
install(PROGRAMS
  scripts/ros2_control_description_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
