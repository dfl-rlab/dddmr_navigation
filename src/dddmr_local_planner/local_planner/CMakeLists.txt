cmake_minimum_required(VERSION 3.5)
project(local_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(base_trajectory REQUIRED)
find_package(perception_3d REQUIRED)
find_package(mpc_critics REQUIRED)
find_package(trajectory_generators REQUIRED)
find_package(dddmr_sys_core REQUIRED)

find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
  angles
  base_trajectory
  perception_3d
  mpc_critics
  trajectory_generators
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${dddmr_sys_core_INCLUDE_DIRS}
)


# local_planner
add_library(local_planner SHARED
  src/local_planner.cpp
)
target_link_libraries(local_planner
  ${PCL_LIBRARIES}
)
ament_target_dependencies(local_planner
  ${dependencies}
)

#local_planner_node
add_executable(local_planner_node
  src/local_planner_node.cpp
)
target_link_libraries(local_planner_node 
  local_planner
)
ament_target_dependencies(local_planner_node
  ${dependencies}
)

#local_planner_play_ground_node
add_executable(local_planner_play_ground_node
  src/local_planner_play_ground_node.cpp
)
target_link_libraries(local_planner_play_ground_node 
  local_planner
)
ament_target_dependencies(local_planner_play_ground_node
  ${dependencies}
)

#ackermann_play_ground_node
add_executable(ackermann_play_ground_node
  src/ackermann_play_ground_node.cpp
)
target_link_libraries(ackermann_play_ground_node 
  local_planner
)
ament_target_dependencies(ackermann_play_ground_node
  ${dependencies}
)

install(TARGETS
  local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  local_planner_node
  local_planner_play_ground_node
  ackermann_play_ground_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  #script/example_client.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(local_planner)
ament_export_dependencies(${dependencies})
ament_package()