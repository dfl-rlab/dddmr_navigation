cmake_minimum_required(VERSION 3.5)
project(perception_3d)

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
find_package(angles REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
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
  angles
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

# shared data
add_library(shared_data_sensor SHARED
  src/shared_data.cpp
  src/graph/dynamic_graph.cpp
  src/graph/static_graph.cpp
)
target_link_libraries(shared_data_sensor
  ${PCL_LIBRARIES}
)
ament_target_dependencies(shared_data_sensor
  ${dependencies}
)

# perception_sensor
add_library(perception_sensor SHARED
  src/sensor.cpp
)
target_link_libraries(perception_sensor
  shared_data_sensor
)
ament_target_dependencies(perception_sensor
  ${dependencies}
)

# stacked_perception
add_library(stacked_perception SHARED
  src/stacked_perception.cpp
)
target_link_libraries(stacked_perception
  perception_sensor
)
ament_target_dependencies(stacked_perception
  ${dependencies}
)

# stacked_perception_ros
add_library(stacked_perception_ros SHARED
  src/perception_3d_ros.cpp
)
target_link_libraries(stacked_perception_ros
  stacked_perception
)
ament_target_dependencies(stacked_perception_ros
  ${dependencies}
)

#perception 3d ros node 
add_executable(perception_3d_ros_node
  src/perception_3d_node.cpp
)
target_link_libraries(perception_3d_ros_node
  stacked_perception_ros
)
ament_target_dependencies(perception_3d_ros_node
  ${dependencies}
)

# perception_cluster_marking
add_library(cluster_marking SHARED
  plugins/cluster_marking.cpp
)
target_link_libraries(cluster_marking
  perception_sensor
)
ament_target_dependencies(cluster_marking
  ${dependencies}
)

#depth_camera_lib
add_library(depth_camera_buffer SHARED
  plugins/depth_camera/depth_camera_observation.cpp
  plugins/depth_camera/depth_camera_observation_buffer.cpp
  plugins/depth_camera/frustum_utils.cpp
)
target_link_libraries(depth_camera_buffer
  ${PCL_LIBRARIES}
)
ament_target_dependencies(depth_camera_buffer
  ${dependencies}
)

# layers
add_library(layers SHARED
  plugins/nothing_layer.cpp
  plugins/no_entry_layer.cpp
  plugins/speed_limit_layer.cpp
  plugins/static_layer.cpp
  plugins/path_blocked_strategy.cpp
  plugins/multilayer_spinning_lidar.cpp
  plugins/depth_camera/depth_camera_layer.cpp
)
target_link_libraries(layers
  cluster_marking 
  depth_camera_buffer 
  stacked_perception_ros
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(layers
  ${dependencies}
)

# everything utils
set(utils_dependencies
  rclcpp
  sensor_msgs
  builtin_interfaces
  pcl_conversions
  cv_bridge
  image_geometry
)
add_executable(depthimg2pointcloud_node utils/depthimg2pointcloud_node.cpp)
target_link_libraries(depthimg2pointcloud_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(depthimg2pointcloud_node
  ${utils_dependencies}
)

install(TARGETS
  shared_data_sensor
  perception_sensor
  stacked_perception
  stacked_perception_ros
  depth_camera_buffer
  cluster_marking
  layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  perception_3d_ros_node
  depthimg2pointcloud_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  map
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES perception_3d_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(shared_data_sensor perception_sensor stacked_perception stacked_perception_ros cluster_marking depth_camera_buffer layers)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(perception_3d perception_3d_plugins.xml)
ament_package()