cmake_minimum_required(VERSION 3.8)
project(palletizing_amr)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# --- ADDED: dependencies needed to build the forklift PID node ---
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# ------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- ADDED: build the forklift PID node as an executable ---
add_executable(forklift_position_pid src/forklift_position_pid.cpp)
ament_target_dependencies(forklift_position_pid
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS
  forklift_position_pid
  DESTINATION lib/${PROJECT_NAME}
)
# -------------------------------------------------------------

install(DIRECTORY
  urdf
  meshes
  worlds
  launch
  rviz
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()