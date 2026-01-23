cmake_minimum_required(VERSION 3.8)
project(assignment2_rt_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Message/service generation
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(builtin_interfaces REQUIRED)

# (These will be useful later for nodes, keep them)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


ament_package_xml()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "srv/SetThreshold.srv"
  "srv/GetLast5InputAvg.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(teleop_interface src/teleop_interface.cpp)
ament_target_dependencies(teleop_interface rclcpp geometry_msgs)

add_executable(safety_controller src/safety_controller.cpp)
ament_target_dependencies(safety_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(teleop_interface "${cpp_typesupport_target}")
target_link_libraries(safety_controller "${cpp_typesupport_target}")

install(TARGETS
  teleop_interface
  safety_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
