cmake_minimum_required(VERSION 3.8)
project(uav_px4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPx4Position.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(px4_control_action_node
  src/px4_control_action_node.cpp
)

ament_target_dependencies(px4_control_action_node
  rclcpp
  rclcpp_action
  geometry_msgs
  px4_msgs
)

target_link_libraries(px4_control_action_node
  "${cpp_typesupport_target}"
)

add_executable(px4_odometry_to_tf_node
  src/px4_odometry_to_tf_node.cpp
)

ament_target_dependencies(px4_odometry_to_tf_node
  rclcpp
  px4_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  px4_control_action_node
  px4_odometry_to_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()