cmake_minimum_required(VERSION 3.5)

project(map_delete_panel LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)

find_package(sensor_msgs REQUIRED)
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

ament_package_xml()
set(ROS_DISTRO "ROS 2")
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")

set(${PROJECT_NAME}_headers_to_moc
        include/map_delete_panel/map_delete_panel.h
        )

foreach(header "${${PROJECT_NAME}_headers_to_moc}")
    qt5_wrap_cpp(${PROJECT_NAME}_moc_files "${header}")
endforeach()

set(${PROJECT_NAME}_source_files
        src/map_delete_panel.cpp
        )

add_library(${PROJECT_NAME} SHARED
        ${${PROJECT_NAME}_moc_files}
        ${${PROJECT_NAME}_source_files}
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        Qt5::Widgets
        ${PCL_LIBRARIES}
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        pluginlib
        rclcpp
        rviz_assimp_vendor
        rviz_rendering
        rviz_common
        sensor_msgs
        pcl_conversions
        )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
        rviz_rendering
        rviz_common
        rcl_interfaces
        pluginlib
        rclcpp
        sensor_msgs
        pcl_conversions
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_package()
