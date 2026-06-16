cmake_minimum_required(VERSION 3.5)
project(lego_loam_bor)

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
find_package(cloud_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread date_time regex timer chrono)
find_package(GTSAM REQUIRED)
find_package(PCL 1.15 REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(dddmr_trt REQUIRED)
find_package(dddmr_sys_core REQUIRED)
find_package(dddmr_pcl REQUIRED)

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
  rosbag2_cpp
  rosbag2_storage
  cloud_msgs
  cv_bridge
  image_geometry
  dddmr_sys_core
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
    ${GTSAM_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${dddmr_trt_INCLUDE_DIRS}
    ${dddmr_sys_core_INCLUDE_DIRS}
    ${dddmr_pcl_INCLUDE_DIRS}
  )
else()
    message(STATUS "TensorRT is disabled.")
    include_directories(
      include
      ${PCL_INCLUDE_DIRS}
      ${GTSAM_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
      ${dddmr_sys_core_INCLUDE_DIRS}
      ${dddmr_pcl_INCLUDE_DIRS}
    )
endif()

# opt_icp_gn
add_library(opt_icp_gn SHARED
  src/opt_icp_gn/optimized_ICP_GN.cpp
)
target_link_libraries(opt_icp_gn
  ${PCL_LIBRARIES}
)
ament_target_dependencies(opt_icp_gn
  ${dependencies}
)

if(TRT_ENABLED)
  add_definitions(-DTRT_ENABLED) 
  # yolo lego loam ip
  add_library(image_projection SHARED
    src/imageProjection.cpp
  )
  
  target_link_libraries(image_projection
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${dddmr_trt_LIBRARIES}
    CUDA::cudart
    ${dddmr_pcl_LIBRARIES}
  )
  target_include_directories(image_projection 
    PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
  )
  ament_target_dependencies(image_projection
    ${dependencies}
  )
else()
  # mcl/lego loam ip
  add_library(image_projection SHARED
    src/imageProjection.cpp
  )
  target_link_libraries(image_projection
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${dddmr_pcl_LIBRARIES}
  )
  ament_target_dependencies(image_projection
    ${dependencies}
  )
endif()

#lego loam fa
add_library(feature_association SHARED
  src/featureAssociation.cpp
)
target_link_libraries(feature_association
  ${PCL_LIBRARIES}
  ${dddmr_pcl_LIBRARIES}
)
ament_target_dependencies(feature_association
  ${dependencies}
)

#lego loam mo
add_library(map_optimization SHARED
  src/mapOptimization.cpp
)
target_link_libraries(map_optimization
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  opt_icp_gn
  ${dddmr_pcl_LIBRARIES}
)
ament_target_dependencies(map_optimization
  ${dependencies}
)

#lego loam visualization
add_library(lego_loam_map_visualization SHARED
  src/utils/lego_loam_map_visualization.cpp
)
target_link_libraries(lego_loam_map_visualization
  ${PCL_LIBRARIES}
  ${dddmr_pcl_LIBRARIES}
)
ament_target_dependencies(lego_loam_map_visualization
  ${dependencies}
)

#pose graph editing
add_library(pose_graph_editor SHARED
  src/pose_graph_editor/pose_graph_editor.cpp
)
target_link_libraries(pose_graph_editor
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  opt_icp_gn
)
ament_target_dependencies(pose_graph_editor
  ${dependencies}
)

#pose graph merge editing
add_library(pose_graph_merge_editor SHARED
  src/pose_graph_editor/pose_graph_merge_editor.cpp
)
target_link_libraries(pose_graph_merge_editor
  pose_graph_editor
)
ament_target_dependencies(pose_graph_merge_editor
  ${dependencies}
)

#mcl feature node
add_executable(mcl_feature
  src/mcl_feature_node.cpp
)
target_link_libraries(mcl_feature 
  image_projection
  feature_association
  gtsam
)
ament_target_dependencies(mcl_feature
  ${dependencies}
)

#interactive pose graph editing
add_library(interactive_pose_graph_editor SHARED
  src/interactive_pose_graph_editor.cpp
)
target_link_libraries(interactive_pose_graph_editor
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  map_optimization
  opt_icp_gn
)
ament_target_dependencies(interactive_pose_graph_editor
  ${dependencies}
)

#lego loam bag node
add_executable(lego_loam_bag
  src/lego_loam_bag_node.cpp
)
target_link_libraries(lego_loam_bag 
  interactive_pose_graph_editor
  image_projection
  feature_association
  map_optimization
  lego_loam_map_visualization
  gtsam
)
ament_target_dependencies(lego_loam_bag
  ${dependencies}
)

#lego loam node
add_executable(lego_loam
  src/lego_loam_node.cpp
)
target_link_libraries(lego_loam 
  interactive_pose_graph_editor
  image_projection
  feature_association
  map_optimization
  lego_loam_map_visualization
  gtsam
)
ament_target_dependencies(lego_loam
  ${dependencies}
)

#pose graph editor node
add_executable(pose_graph_editor_node
  src/pose_graph_editor/pose_graph_editor_node.cpp
)
target_link_libraries(pose_graph_editor_node 
  pose_graph_merge_editor
  gtsam
)
ament_target_dependencies(pose_graph_editor_node
  ${dependencies}
)

#pointcloud to occupancy node
add_executable(pointcloud2occupancy
  src/utils/pointcloud2occupancy.cpp
)
target_link_libraries(pointcloud2occupancy 
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pointcloud2occupancy
  ${dependencies}
)

#optimized icp gn playground
add_executable(optimized_icp_gn_playground
  src/opt_icp_gn/optimized_ICP_GN_playground.cpp
)
target_link_libraries(optimized_icp_gn_playground
  opt_icp_gn
  ${PCL_LIBRARIES}
)
ament_target_dependencies(optimized_icp_gn_playground
  ${dependencies}
)

#lego loam lidar2depth_img node
add_executable(lego_loam_bag_lidar2depthimg_node
  src/utils/lego_loam_bag_lidar2depthimg_node.cpp
)
target_link_libraries(lego_loam_bag_lidar2depthimg_node 
  interactive_pose_graph_editor
  image_projection
  feature_association
  map_optimization
  gtsam
)
ament_target_dependencies(lego_loam_bag_lidar2depthimg_node
  ${dependencies}
)

#--- Everything below require CUDA ---

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
    ${GTSAM_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${dddmr_trt_INCLUDE_DIRS}
    ${dddmr_pcl_INCLUDE_DIRS}
  )

  #yolo ros2 test node
  add_executable(yolo_ros2_image_sub
    src/utils/yolo_ros2_image_sub.cpp
  )
  target_link_libraries(yolo_ros2_image_sub 
    ${OpenCV_LIBRARIES}
    ${dddmr_trt_LIBRARIES}
    CUDA::cudart
  )
  target_include_directories(yolo_ros2_image_sub 
    PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
  )
  ament_target_dependencies(yolo_ros2_image_sub
    ${dependencies}
  )

  install(TARGETS
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  #install node
  install(TARGETS
    yolo_ros2_image_sub
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  # --- Everything CUDA finish here ---

else()
    message(STATUS "TensorRT is disabled.")
endif()

install(TARGETS
  opt_icp_gn
  image_projection
  feature_association
  map_optimization
  lego_loam_map_visualization
  pose_graph_editor
  interactive_pose_graph_editor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  mcl_feature
  lego_loam
  lego_loam_bag
  pose_graph_editor_node
  pointcloud2occupancy
  optimized_icp_gn_playground
  lego_loam_bag_lidar2depthimg_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  pcd
  engine
  DESTINATION share/${PROJECT_NAME}/
)


#--- Everything below is CI/CD test ---

#Check CI/CD build flag
if(NOT TEST_ENABLED)
    set(TEST_ENABLED OFF CACHE BOOL "Enable or disable Test")
endif()

if(TEST_ENABLED)

  add_definitions(-DTEST_ENABLED) 
  # Compile test and install them 
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}/
  )

  #testing node
  add_executable(mapping_test_node
    test/test_node/mapping_test_node.cpp
  )
  target_link_libraries(mapping_test_node 
    opt_icp_gn
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(mapping_test_node
    ${dependencies}
  )

  install(PROGRAMS
    test/test_node/mapping_test_node.py
    DESTINATION lib/${PROJECT_NAME}
  )

  #install testing node
  install(TARGETS
    mapping_test_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

endif()

# --- Everything CI/CD test finish here ---

ament_export_include_directories(include)
ament_export_libraries(opt_icp_gn image_projection feature_association map_optimization lego_loam_map_visualization pose_graph_editor interactive_pose_graph_editor)
ament_export_dependencies(${dependencies})
ament_package()