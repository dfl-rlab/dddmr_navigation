cmake_minimum_required(VERSION 3.5)

project(dddmr_rviz_default_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()


# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)

find_package(ignition-math6 REQUIRED)

find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED COMPONENTS filters common io)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

set(dddmr_rviz_default_plugins_headers_to_moc
  include/dddmr_rviz_default_plugins/tools/goal_pose/goal_tool.hpp
  include/dddmr_rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp
  include/dddmr_rviz_default_plugins/tools/select/edge_selection_tool.hpp
  include/dddmr_rviz_default_plugins/tools/select/node_selection_tool.hpp
  include/dddmr_rviz_default_plugins/tools/point_cloud_selection/point_cloud_selection_tool.hpp
  include/dddmr_rviz_default_plugins/tools/point_cloud_delete/point_cloud_delete_tool.hpp
)

foreach(header "${dddmr_rviz_default_plugins_headers_to_moc}")
  qt5_wrap_cpp(dddmr_rviz_default_plugins_moc_files "${header}")
endforeach()

set(dddmr_rviz_default_plugins_source_files
  src/dddmr_rviz_default_plugins/tools/goal_pose/goal_tool.cpp
  src/dddmr_rviz_default_plugins/tools/pose/pose_tool.cpp
  src/dddmr_rviz_default_plugins/tools/pose_estimate/initial_pose_tool.cpp
  src/dddmr_rviz_default_plugins/tools/select/edge_selection_tool.cpp
  src/dddmr_rviz_default_plugins/tools/move/move_tool.cpp
  src/dddmr_rviz_default_plugins/tools/select/node_selection_tool.cpp
  src/dddmr_rviz_default_plugins/tools/point_cloud_selection/point_cloud_selection_tool.cpp
  src/dddmr_rviz_default_plugins/tools/point_cloud_delete/point_cloud_delete_tool.cpp
)

add_library(dddmr_rviz_default_plugins SHARED
  ${dddmr_rviz_default_plugins_moc_files}
  ${dddmr_rviz_default_plugins_source_files}
)

target_include_directories(dddmr_rviz_default_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(dddmr_rviz_default_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ${PCL_LIBRARIES}
)

target_link_libraries(dddmr_rviz_default_plugins PRIVATE
  ignition-math6
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dddmr_rviz_default_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(dddmr_rviz_default_plugins
  PUBLIC
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_rendering
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  pcl_conversions
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(dddmr_rviz_default_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  pcl_conversions
)

install(
  TARGETS dddmr_rviz_default_plugins
  EXPORT dddmr_rviz_default_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package(
  CONFIG_EXTRAS "dddmr_rviz_default_plugins-extras.cmake"
)

