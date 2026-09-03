cmake_minimum_required(VERSION 3.5)
project(global_planner)

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
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PCL REQUIRED)
find_package(perception_3d REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP REQUIRED)

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
  perception_3d
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

# a_star_on_graph
add_library(a_star_on_graph SHARED
  src/a_star_on_pc.cpp
  src/a_star_on_pre_graph.cpp
)
target_link_libraries(a_star_on_graph
  ${PCL_LIBRARIES}
)
ament_target_dependencies(a_star_on_graph
  ${dependencies}
)

# global_planner
add_library(global_planner SHARED
  src/global_planner.cpp
)
target_link_libraries(global_planner
  a_star_on_graph
)
ament_target_dependencies(global_planner
  ${dependencies}
)

# dynamic_window_aware_global_planner
add_library(dynamic_window_aware_global_planner SHARED
  src/dynamic_window_aware_global_planner.cpp
)
target_link_libraries(dynamic_window_aware_global_planner
  global_planner
)
ament_target_dependencies(dynamic_window_aware_global_planner
  ${dependencies}
)

#global_planner_node
add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_link_libraries(global_planner_node 
  dynamic_window_aware_global_planner
)
ament_target_dependencies(global_planner_node
  ${dependencies}
)

#occupancy2ground
add_executable(occupancy2ground
  utils/occupancy2ground.cpp
)
target_link_libraries(occupancy2ground 
  ${PCL_LIBRARIES}
  yaml-cpp
  small_gicp::small_gicp
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(occupancy2ground
  ${dependencies}
)

#dummy_pc_publisher
add_executable(dummy_pc_publisher
  test/dummy_pc_pub.cpp
)
target_link_libraries(dummy_pc_publisher 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(dummy_pc_publisher
  ${dependencies}
)

install(TARGETS
  a_star_on_graph
  global_planner
  dynamic_window_aware_global_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  global_planner_node
  dummy_pc_publisher
  occupancy2ground
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  data
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  script/example_client.py
  script/dwa_global_planner_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(a_star_on_graph global_planner dynamic_window_aware_global_planner)
ament_export_dependencies(${dependencies})
ament_package()