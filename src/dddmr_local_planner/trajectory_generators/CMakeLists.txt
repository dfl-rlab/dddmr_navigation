cmake_minimum_required(VERSION 3.5)
project(trajectory_generators)

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

# trajectory_generator_theory
add_library(trajectory_generator_theory SHARED
  src/trajectory_generator_theory.cpp
)
target_link_libraries(trajectory_generator_theory
  ${PCL_LIBRARIES}
)
ament_target_dependencies(trajectory_generator_theory
  ${dependencies}
)

# stacked_generator
add_library(stacked_generator SHARED
  src/stacked_generator.cpp
)
target_link_libraries(stacked_generator
  trajectory_generator_theory
)
ament_target_dependencies(stacked_generator
  ${dependencies}
)

# trajectory_generator_ros
add_library(trajectory_generator_ros SHARED
  src/trajectory_generators_ros.cpp
)
target_link_libraries(trajectory_generator_ros
  stacked_generator
)
ament_target_dependencies(trajectory_generator_ros
  ${dependencies}
)

# theories
add_library(theories SHARED
  theories/dd_rotate_inplace_theory.cpp
  theories/dd_simple_trajectory_generator_theory.cpp
  theories/omni_simple_trajectory_generator_theory.cpp
  theories/one_state_trajectory_generator_theory.cpp
)
target_link_libraries(theories
  trajectory_generator_ros
)
ament_target_dependencies(theories
  ${dependencies}
)

#node
add_executable(trajectory_generators_node
  src/trajectory_generators_node.cpp
)
target_link_libraries(trajectory_generators_node 
  trajectory_generator_ros
  theories
)
ament_target_dependencies(trajectory_generators_node
  ${dependencies}
)

install(TARGETS
  trajectory_generator_theory
  stacked_generator
  trajectory_generator_ros
  theories
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  trajectory_generators_node
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

install(FILES trajectory_generators.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(trajectory_generator_theory stacked_generator trajectory_generator_ros theories)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(trajectory_generators trajectory_generators.xml)
ament_package()