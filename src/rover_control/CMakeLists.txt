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
find_package(visualization_msgs REQUIRED) # add required package for rviz

# Include directories
include_directories(include)

# Create executables
add_executable(controller_node src/control_node_controller.cpp)
add_executable(joystick_node src/control_node_joystick.cpp)
add_executable(viz_node_controller src/viz_node_controller.cpp) # add rviz executable

# Include paths for executables
target_include_directories(controller_node PRIVATE include)
target_include_directories(joystick_node PRIVATE include)
target_include_directories(viz_node_controller PRIVATE include) # add rviz executab

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

ament_target_dependencies(viz_node_controller
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
) # add dependencies for rviz executable

# Install targets
install(TARGETS
  controller_node
  joystick_node
  viz_node_controller 
  DESTINATION lib/${PROJECT_NAME}
) # add rviz install targets

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
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