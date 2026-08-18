cmake_minimum_required(VERSION 3.5)
project(dddmr_semantic_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

find_package(PCL 1.15 REQUIRED QUIET)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  builtin_interfaces
  pcl_conversions
  cv_bridge
  image_geometry
)

if("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    add_compile_definitions(ROS_JAZZY=1)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(semantic_segmentation2point_cloud src/semantic_segmentation2point_cloud.cpp)
target_link_libraries(semantic_segmentation2point_cloud
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(semantic_segmentation2point_cloud
  ${dependencies}
)

#install lib
install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  semantic_segmentation2point_cloud
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install include
install(DIRECTORY
  DESTINATION include/
)

install(DIRECTORY
  launch
  model
  data
  scripts
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  scripts/ddrnet_ros_img_sub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
#ament_export_libraries()
ament_export_dependencies(${dependencies})
ament_package()