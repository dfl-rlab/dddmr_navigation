cmake_minimum_required(VERSION 3.5)
project(recovery_behaviors)

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


# robot_behavior
add_library(robot_behavior SHARED
  src/robot_behavior.cpp
)
target_link_libraries(robot_behavior
  ${PCL_LIBRARIES}
)
ament_target_dependencies(robot_behavior
  ${dependencies}
)

# stacked_robot_behavior
add_library(stacked_robot_behavior SHARED
  src/stacked_robot_behavior.cpp
)
target_link_libraries(stacked_robot_behavior
  robot_behavior
)
ament_target_dependencies(stacked_robot_behavior
  ${dependencies}
)

# recovery_behaviors_ros
add_library(recovery_behaviors_ros SHARED
  src/recovery_behaviors_ros.cpp
)
target_link_libraries(recovery_behaviors_ros
  stacked_robot_behavior
)
ament_target_dependencies(recovery_behaviors_ros
  ${dependencies}
)

# behaviors
add_library(behaviors SHARED
  behaviors/rotate_inplace_behavior.cpp
  behaviors/position_control_behavior.cpp
)
target_link_libraries(behaviors
  recovery_behaviors_ros
)
ament_target_dependencies(behaviors
  ${dependencies}
)

#recovery_behaviors_node
add_executable(recovery_behaviors_node
  src/recovery_behaviors_node.cpp
)
target_link_libraries(recovery_behaviors_node 
  recovery_behaviors_ros
)
ament_target_dependencies(recovery_behaviors_node
  ${dependencies}
)

install(TARGETS
  robot_behavior
  stacked_robot_behavior
  recovery_behaviors_ros
  behaviors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  recovery_behaviors_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  script/example_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES recovery_behaviors.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(robot_behavior stacked_robot_behavior recovery_behaviors_ros)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(recovery_behaviors recovery_behaviors.xml)
ament_package()