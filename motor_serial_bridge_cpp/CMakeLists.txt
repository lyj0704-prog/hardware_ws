cmake_minimum_required(VERSION 3.8)
project(motor_serial_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(motor_serial_bridge_node
  src/motor_serial_bridge_node.cpp
)

add_executable(joy_to_ackermann_node
  src/joy_to_ackermann_node.cpp
)

add_executable(keyboard_to_ackermann_node
  src/keyboard_to_ackermann_node.cpp
)

ament_target_dependencies(motor_serial_bridge_node
  rclcpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(joy_to_ackermann_node
  rclcpp
  ackermann_msgs
  sensor_msgs
)

ament_target_dependencies(keyboard_to_ackermann_node
  rclcpp
  ackermann_msgs
)

install(TARGETS
  motor_serial_bridge_node
  joy_to_ackermann_node
  keyboard_to_ackermann_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
