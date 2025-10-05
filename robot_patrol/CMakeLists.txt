cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Create executable
add_executable(robot_patrol_node src/patrol.cpp)
add_executable(robot_patrol_with_service src/patrol_with_service.cpp)
add_executable(direction_service_server src/direction_service.cpp)
add_executable(test_direction_service_client src/test_service.cpp)
add_executable(gotopose_action_server src/go_to_pose_action.cpp)

# Add dependencies
ament_target_dependencies(robot_patrol_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(robot_patrol_with_service rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(direction_service_server rclcpp)
ament_target_dependencies(test_direction_service_client rclcpp sensor_msgs)
ament_target_dependencies(gotopose_action_server rclcpp rclcpp_action nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(robot_patrol_with_service "${cpp_typesupport_target}")
target_link_libraries(direction_service_server "${cpp_typesupport_target}")
target_link_libraries(test_direction_service_client "${cpp_typesupport_target}")
target_link_libraries(gotopose_action_server "${cpp_typesupport_target}")

# Install executables
install(TARGETS robot_patrol_node robot_patrol_with_service direction_service_server test_direction_service_client gotopose_action_server
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)


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

ament_package()
