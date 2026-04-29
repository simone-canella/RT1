cmake_minimum_required(VERSION 3.8)
project(assignment1_rt2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# FIND DEPENDENCIES
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED) # Added this explicitly
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(assignment1_rt2_interfaces REQUIRED)

# ---------------------------------------------------------
# ACTION SERVER COMPONENT
# ---------------------------------------------------------
add_library(action_server_component SHARED
  src/action_server.cpp
)

ament_target_dependencies(action_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  assignment1_rt2_interfaces
)

rclcpp_components_register_node(action_server_component
  PLUGIN "assignment1_rt2::RobotActionServer"
  EXECUTABLE action_server_node
)

# ---------------------------------------------------------
# ACTION CLIENT & UI NODE (Standalone Executable)
# ---------------------------------------------------------
add_executable(action_client_node src/action_client.cpp)

ament_target_dependencies(action_client_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  assignment1_rt2_interfaces
)

# ---------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------
install(TARGETS
  action_server_component
  action_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()