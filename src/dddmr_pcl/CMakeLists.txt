cmake_minimum_required(VERSION 3.5)
project(dddmr_pcl)

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
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread date_time regex timer chrono)
find_package(PCL 1.15 REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(dddmr_trt REQUIRED)
find_package(dddmr_sys_core REQUIRED)
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
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
  cv_bridge
  image_geometry
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${dddmr_sys_core_INCLUDE_DIRS}
  ${OpenMP_CXX_INCLUDE_DIRS}
)

#voxel_omp
add_library(voxel_omp SHARED
  src/voxel_omp/voxel_grid_omp.cpp
)
target_link_libraries(voxel_omp
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)
ament_target_dependencies(voxel_omp
  ${dependencies}
)

install(TARGETS
  voxel_omp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(voxel_omp)
ament_export_dependencies(${dependencies})
ament_package()