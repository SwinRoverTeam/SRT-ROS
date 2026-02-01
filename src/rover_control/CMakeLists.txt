cmake_minimum_required(VERSION 3.8)
project(rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create executables
add_executable(controller_node src/control_node_controller.cpp)
add_executable(joystick_node src/control_node_joystick.cpp)

ament_target_dependencies(controller_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(joystick_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS
  controller_node
  joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()