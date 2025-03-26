cmake_minimum_required(VERSION 3.8)
project(my_robot_nav_bt)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Executable
add_executable(bt_runner
  src/bt_runner.cpp
  src/BatteryMonitor.cpp
  src/NavigateToWaypoint.cpp
  src/NavigateToCharger.cpp
  src/WaitForRecharge.cpp
)

# Link libraries
target_link_libraries(bt_runner
  ${rclcpp_LIBRARIES}
)

ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
  geometry_msgs
  std_msgs
)

# Install targets and auxiliary directories
install(TARGETS
  bt_runner
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch behavior_trees maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
