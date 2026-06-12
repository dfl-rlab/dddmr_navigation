cmake_minimum_required(VERSION 3.8)
project(dddmr_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(angles REQUIRED)
find_package(image_geometry REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core calib3d)
find_package(cv_bridge REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(dddmr_trt REQUIRED)
find_package(PCL 1.15 REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  rclcpp_action
  dddmr_sys_core
  angles
  image_geometry
  cv_bridge
  pcl_conversions
)

#Check yolo build flag
if(NOT TRT_ENABLED)
    set(TRT_ENABLED OFF CACHE BOOL "Enable or disable TensorRT")
endif()

if(TRT_ENABLED)

  add_definitions(-DTRT_ENABLED) 
  find_package(CUDAToolkit REQUIRED)

  message(STATUS "TensorRT is enabled. Compiling TRT and YOLO features")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake")
  
  include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${dddmr_sys_core_INCLUDE_DIRS}
    ${dddmr_trt_INCLUDE_DIRS}
  )

  add_library(apriltag_trt_tracking SHARED
    src/apriltag_trt_tracking.cpp
    src/apriltag_trt_pose_detector.cpp
  )
  target_link_libraries(apriltag_trt_tracking
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${dddmr_trt_LIBRARIES}
    CUDA::cudart
  )
  target_include_directories(apriltag_trt_tracking 
    PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
  )
  ament_target_dependencies(apriltag_trt_tracking
    ${dependencies}
  )

  add_library(mpc_docking SHARED
    src/mpc_docking.cpp
  )
  target_link_libraries(mpc_docking
    apriltag_trt_tracking
  )
  ament_target_dependencies(mpc_docking
    ${dependencies}
  )

  #node
  add_executable(dddmr_docking
    src/docking_node.cpp
  )
  target_link_libraries(dddmr_docking 
    mpc_docking
  )
  ament_target_dependencies(dddmr_docking
    ${dependencies}
  )

  install(TARGETS
    apriltag_trt_tracking
    mpc_docking
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  #install node
  install(TARGETS
    dddmr_docking
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(PROGRAMS
    scripts/trigger_mpc_docking.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    config
    rviz
    engine
    DESTINATION share/${PROJECT_NAME}
  )

  ament_export_include_directories(include)
  ament_export_libraries(apriltag_trt_tracking mpc_docking)
  ament_export_dependencies(${dependencies})
  ament_package()

endif()
