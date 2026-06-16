cmake_minimum_required(VERSION 3.5)
project(mpc_critics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
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
  ackermann_msgs
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
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${base_trajectory_INCLUDE_DIRS}
)

# scoring_model
add_library(scoring_model SHARED
  src/scoring_model.cpp
)
target_link_libraries(scoring_model
  ${PCL_LIBRARIES}
)
ament_target_dependencies(scoring_model
  ${dependencies}
)

# stacked_scoring_model
add_library(stacked_scoring_model SHARED
  src/stacked_scoring_model.cpp
)
target_link_libraries(stacked_scoring_model
  scoring_model
)
ament_target_dependencies(stacked_scoring_model
  ${dependencies}
)

# mpc_critics_ros
add_library(mpc_critics_ros SHARED
  src/mpc_critics_ros.cpp
)
target_link_libraries(mpc_critics_ros
  stacked_scoring_model
)
ament_target_dependencies(mpc_critics_ros
  ${dependencies}
)

# critics
add_library(critics SHARED
  models/pure_pursuit_model.cpp
  models/collision_model.cpp
  models/collision_min_max_model.cpp
  models/shortest_angle_model.cpp
  models/stick_path_model.cpp
  models/toward_global_plan_model.cpp
  models/twirling_model.cpp
)
target_link_libraries(critics
  mpc_critics_ros
)
ament_target_dependencies(critics
  ${dependencies}
)

# mpc_critics_node
add_executable(mpc_critics_node
  src/mpc_critics_node.cpp
)
target_link_libraries(mpc_critics_node
  mpc_critics_ros
)
ament_target_dependencies(mpc_critics_node
  ${dependencies}
)

install(TARGETS
  scoring_model
  stacked_scoring_model
  mpc_critics_ros
  critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  mpc_critics_node
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

install(FILES mpc_critics.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(scoring_model stacked_scoring_model mpc_critics_ros critics)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(mpc_critics mpc_critics.xml)
ament_package()