cmake_minimum_required(VERSION 3.5)
project(dddmr_explore_and_search)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(dddmr_sys_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
  sensor_msgs
  pcl_conversions
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${dddmr_sys_core_INCLUDE_DIRS}
)

# dddmr_explore_and_search
# camera_frustum
add_library(camera_frustum SHARED
  src/camera_frustum.cpp
)
target_link_libraries(camera_frustum
  ${PCL_LIBRARIES}
)
ament_target_dependencies(camera_frustum
  ${dependencies}
)

# dddmr_explore_and_search
add_library(dddmr_explore_and_search SHARED
  src/dddmr_explore_and_search.cpp
)
target_link_libraries(dddmr_explore_and_search
  ${PCL_LIBRARIES}
  camera_frustum
)

ament_target_dependencies(dddmr_explore_and_search
  ${dependencies}
)

# dddmr_explore_and_search_node
add_executable(dddmr_explore_and_search_node
  src/main.cpp
)
target_link_libraries(dddmr_explore_and_search_node 
  dddmr_explore_and_search
)
ament_target_dependencies(dddmr_explore_and_search_node
  ${dependencies}
)

install(TARGETS
  dddmr_explore_and_search
  camera_frustum
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  dddmr_explore_and_search_node
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

ament_export_include_directories(include)
ament_export_libraries(dddmr_explore_and_search camera_frustum)
ament_export_dependencies(${dependencies})
ament_package()
