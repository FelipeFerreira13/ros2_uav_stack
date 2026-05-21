cmake_minimum_required(VERSION 3.8)
project(uav_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToWaypoint.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(waypoint_action_server src/waypoint_nav.cpp)

ament_target_dependencies(waypoint_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
)

target_link_libraries(waypoint_action_server
  "${cpp_typesupport_target}"
)

install(TARGETS
  waypoint_action_server
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
