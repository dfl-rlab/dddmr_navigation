// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "mapOptimization.h"
#include <future>

using namespace gtsam;

MapOptimization::MapOptimization(std::string name,
                                 Channel<AssociationOut> &input_channel)
    : Node(name),
      _input_channel(input_channel)
{
  
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);
  pose_graph_.clear();
  has_m2ci_af3_ = false;
  current_ground_size_ = 0;

  pub_key_pose_arr_ = this->create_publisher<geometry_msgs::msg::PoseArray>("key_poses", 1);

  pub_pose_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_surround", 1);  
  
  pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 5);  
  
  pubIcpTargetKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("loopclosure_target_cloud", 1);  

  pubIcpKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("corrected_cloud", 1);  

  pubcloudKeyPoses6D = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_keypose_6d", 1);  
  
  pubRecentCornerKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("recent_corner_cloud", 1);
  pubRecentSurfKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("recent_surf_cloud", 1);
  pubLastOptimizedCornerKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("optimized_corner_cloud", 1);
  pubLastOptimizedSurfKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>("optimized_surf_cloud", 1);
  pubSelectedCloudForLMOptimization = this->create_publisher<sensor_msgs::msg::PointCloud2>("selected_lm_cloud", 1);
  
  pubM2Ci = this->create_publisher<geometry_msgs::msg::TransformStamped>("lego_loam/m2ci", 1);  
  pubC2S = this->create_publisher<geometry_msgs::msg::TransformStamped>("lego_loam/c2s", 1);  
  pubB2S = this->create_publisher<geometry_msgs::msg::TransformStamped>("lego_loam/c2s", 1);  
  //pubMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_map", 1);  
  //pubGround = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground", 1);  
  //pubGroundEdge = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground_edge", 1);
    
  //TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  clock_ = this->get_clock();

  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
  downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

  downSizeFilterCorner_omp.setNumberOfThreads(6);
  downSizeFilterCorner_omp.setLeafSize (0.2, 0.2, 0.2);
  downSizeFilterCorner_omp.setSaveLeafLayout(false);

  downSizeFilterCornerKeyFrame_omp.setNumberOfThreads(6);
  downSizeFilterCornerKeyFrame_omp.setLeafSize (0.2, 0.2, 0.2);
  downSizeFilterCornerKeyFrame_omp.setSaveLeafLayout(false);

  downSizeFilterSurf_omp.setNumberOfThreads(6);
  downSizeFilterSurf_omp.setLeafSize (0.4, 0.4, 0.4);
  downSizeFilterSurf_omp.setSaveLeafLayout(false);

  downSizeFilterSurfKeyFrame_omp.setNumberOfThreads(6);
  downSizeFilterSurfKeyFrame_omp.setLeafSize (0.4, 0.4, 0.4);
  downSizeFilterSurfKeyFrame_omp.setSaveLeafLayout(false);

  downSizeFilterOutlier_omp.setNumberOfThreads(6);
  downSizeFilterOutlier_omp.setLeafSize (0.4, 0.4, 0.4);
  downSizeFilterOutlier_omp.setSaveLeafLayout(false);

  downSizeFilterSurfTotal_omp.setNumberOfThreads(6);
  downSizeFilterSurfTotal_omp.setLeafSize (0.4, 0.4, 0.4);
  downSizeFilterSurfTotal_omp.setSaveLeafLayout(false);


  downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4);

  downSizeFilterHistoryKeyFrames_omp.setNumberOfThreads(6);
  downSizeFilterHistoryKeyFrames_omp.setLeafSize (0.4, 0.4, 0.4);
  downSizeFilterHistoryKeyFrames_omp.setSaveLeafLayout(false);

  // DS for final stitch
  downSizeFilterFinalStitch.setLeafSize(0.1, 0.1, 0.1);

  declare_parameter("mapping.enable_loop_closure", rclcpp::ParameterValue(false));
  this->get_parameter("mapping.enable_loop_closure", _loop_closure_enabled);
  RCLCPP_INFO(this->get_logger(), "mapping.enable_loop_closure: %d", _loop_closure_enabled);

  declare_parameter("mapping.history_keyframe_search_radius", rclcpp::ParameterValue(0.0));
  this->get_parameter("mapping.history_keyframe_search_radius", _history_keyframe_search_radius);
  RCLCPP_INFO(this->get_logger(), "mapping.history_keyframe_search_radius: %.2f", _history_keyframe_search_radius);

  declare_parameter("mapping.history_keyframe_search_num", rclcpp::ParameterValue(0));
  this->get_parameter("mapping.history_keyframe_search_num", _history_keyframe_search_num);
  RCLCPP_INFO(this->get_logger(), "mapping.history_keyframe_search_num: %d", _history_keyframe_search_num);

  declare_parameter("mapping.history_keyframe_fitness_score", rclcpp::ParameterValue(0.0));
  this->get_parameter("mapping.history_keyframe_fitness_score", _history_keyframe_fitness_score);
  RCLCPP_INFO(this->get_logger(), "mapping.history_keyframe_fitness_score: %.2f", _history_keyframe_fitness_score);

  declare_parameter("mapping.surrounding_keyframe_search_num", rclcpp::ParameterValue(0));
  this->get_parameter("mapping.surrounding_keyframe_search_num", _surrounding_keyframe_search_num);
  RCLCPP_INFO(this->get_logger(), "mapping.surrounding_keyframe_search_num: %d", _surrounding_keyframe_search_num);

  declare_parameter("mapping.angle_between_key_frame", rclcpp::ParameterValue(0.3));
  this->get_parameter("mapping.angle_between_key_frame", angle_between_key_frame_);
  RCLCPP_INFO(this->get_logger(), "mapping.angle_between_key_frame: %.2f", angle_between_key_frame_);

  declare_parameter("mapping.distance_between_key_frame", rclcpp::ParameterValue(0.3));
  this->get_parameter("mapping.distance_between_key_frame", distance_between_key_frame_);
  RCLCPP_INFO(this->get_logger(), "mapping.distance_between_key_frame: %.2f", distance_between_key_frame_);

  declare_parameter("mapping.ground_edge_threshold_num", rclcpp::ParameterValue(50));
  this->get_parameter("mapping.ground_edge_threshold_num", ground_edge_threshold_num_);
  RCLCPP_INFO(this->get_logger(), "mapping.ground_edge_threshold_num: %d", ground_edge_threshold_num_);

  declare_parameter("mapping.broadcast_external_odom_tf", rclcpp::ParameterValue(true));
  this->get_parameter("mapping.broadcast_external_odom_tf", broadcast_external_odom_tf_);
  RCLCPP_INFO(this->get_logger(), "mapping.broadcast_external_odom_tf: %d", broadcast_external_odom_tf_);

  declare_parameter("mapping.generate_testing_pg", rclcpp::ParameterValue(false));
  this->get_parameter("mapping.generate_testing_pg", generate_testing_pg_);
  RCLCPP_INFO(this->get_logger(), "mapping.generate_testing_pg: %d", generate_testing_pg_);  
  
  allocateMemory();

  timer_run_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_pub_gbl_map_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_loop_closure_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  get_key_frame_srv_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  srvSavePCD = this->create_service<std_srvs::srv::Empty>("save_mapped_point_cloud", std::bind(&MapOptimization::pcdSaver, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, get_key_frame_srv_cb_group_);
  srvGetKeyFrameCloud = this->create_service<dddmr_sys_core::srv::GetKeyFrameCloud>("get_key_frame_cloud", std::bind(&MapOptimization::getKeyFrameCloud, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, get_key_frame_srv_cb_group_);

  timer_run_ = this->create_wall_timer(1ms, std::bind(&MapOptimization::run, this), timer_run_cb_group_);
  timer_loop_closure_ = this->create_wall_timer(1000ms, std::bind(&MapOptimization::loopClosureThread, this), timer_loop_closure_cb_group_);
  //timer_ground_edge_detection_ = this->create_wall_timer(200ms, std::bind(&MapOptimization::groundEdgeDetectionThread, this), timer_pub_gbl_map_cb_group_);

}

void MapOptimization::getKeyFrameCloud(const std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Request> request,
          std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Response> response){

  pcl::PointCloud<PointType>::Ptr keyFrameBaseLink;
  keyFrameBaseLink.reset(new pcl::PointCloud<PointType>());
  pcl::toROSMsg(*keyFrameBaseLink, response->key_frame_cloud);
  pcl::toROSMsg(*keyFrameBaseLink, response->key_frame_ground);
  pcl::toROSMsg(*keyFrameBaseLink, response->key_frame_ground_edge);
  
  if(!has_m2ci_af3_) return;
  
  if(request->key_frame_number>=cornerCloudKeyFrames.size())
    return;

  if(request->key_frame_number>=patchedGroundKeyFrames.size())
    return;

  if(request->key_frame_number>=patchedGroundEdgeKeyFrames.size())
    return;

  if(request->key_frame_number>=cloudKeyPoses6D->size())
    return;

  *keyFrameBaseLink = *cornerCloudKeyFrames[request->key_frame_number] + *outlierCloudKeyFrames[request->key_frame_number];
  
  pcl::toROSMsg(*keyFrameBaseLink, response->key_frame_cloud);
  pcl::toROSMsg(*patchedGroundKeyFrames[request->key_frame_number], response->key_frame_ground);
  pcl::toROSMsg(*patchedGroundEdgeKeyFrames[request->key_frame_number], response->key_frame_ground_edge);

}

void MapOptimization::pcdSaver(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response){
  
  if (cloudKeyPoses3D->points.empty() == true) 
    return;

  std::string mapping_dir_string;
  auto env_p = std::getenv("DDDMR_MAPPING_DIR");
  if(generate_testing_pg_){
    mapping_dir_string = std::string("/tmp/testing_pg");
    try {
      std::uintmax_t removed_count = std::filesystem::remove_all(mapping_dir_string); 
      RCLCPP_INFO(this->get_logger(), "Generate testing pg is enabled, removing dir: %s", mapping_dir_string.c_str());
    } catch (const std::filesystem::filesystem_error& e) {
    }
    std::filesystem::create_directory(mapping_dir_string);
    RCLCPP_INFO(this->get_logger(), "Generate testing pg is enabled, create dir: %s", mapping_dir_string.c_str());
  }
  else if ( env_p == NULL ) {
    mapping_dir_string = std::string("/tmp/") + currentDateTime();
    std::filesystem::create_directory(mapping_dir_string);
    RCLCPP_INFO(this->get_logger(), "Create dir: %s", mapping_dir_string.c_str());
  } else {
    mapping_dir_string = std::string( env_p ) + currentDateTime();
    std::filesystem::create_directory(mapping_dir_string);
    RCLCPP_INFO(this->get_logger(), "Create dir: %s", mapping_dir_string.c_str());
  }


  // save map
  for (int i = 0; i < cloudKeyPoses3D->points.size(); ++i) {
    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
    *completeGlobalStitch += *transformPointCloud(
        cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch += *transformPointCloud(
    //    surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch +=
    //    *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
    //                         &cloudKeyPoses6D->points[thisKeyInd]);
  }

  downSizeFilterFinalStitch.setInputCloud(completeGlobalStitch);
  downSizeFilterFinalStitch.filter(*completeGlobalStitch);
  pcl::transformPointCloud(*completeGlobalStitch, *completeGlobalStitch, trans_s2c_af3_);
  pcl::io::savePCDFileASCII(mapping_dir_string + "/map.pcd", *completeGlobalStitch);
  
  //save surface
  completeGlobalStitch.reset(new pcl::PointCloud<PointType>());
  for (int i = 0; i < cloudKeyPoses3D->points.size(); ++i) {
    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
    *completeGlobalStitch += *transformPointCloud(
        patchedGroundKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
  }
  downSizeFilterFinalStitch.setInputCloud(completeGlobalStitch);
  downSizeFilterFinalStitch.filter(*completeGlobalStitch);
  pcl::transformPointCloud(*completeGlobalStitch, *completeGlobalStitch, trans_s2c_af3_);
  pcl::io::savePCDFileASCII(mapping_dir_string + "/ground.pcd", *completeGlobalStitch);
  
  completeGlobalStitch.reset(new pcl::PointCloud<PointType>());
  for (int i = 0; i < cloudKeyPoses3D->points.size(); ++i) {
    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
    //*completeGlobalStitch += *transformPointCloud(
    //    cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch += *transformPointCloud(
    //    surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    //*completeGlobalStitch +=
    //    *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
    //                         &cloudKeyPoses6D->points[thisKeyInd]);
  }

  //@ -----Write poses-----
  pcl::PointCloud<PointTypePose> cloudKeyPoses6DBaseLink;
  std::vector<geometry_msgs::msg::TransformStamped> cloudKeyPoses6DBaseLink_geo;
  for(auto it = cloudKeyPoses6D->points.begin(); it!=cloudKeyPoses6D->points.end(); it++){
    tf2::Transform tf2_trans_ci2c;
    tf2::Quaternion q;
    q.setRPY( (*it).roll, (*it).pitch, (*it).yaw);
    tf2_trans_ci2c.setRotation(q);
    tf2_trans_ci2c.setOrigin(tf2::Vector3((*it).x, (*it).y, (*it).z));

    tf2::Transform tf2_trans_m2c, tf2_trans_m2s, tf2_trans_m2b;
    tf2_trans_m2c.mult(tf2_trans_m2ci_, tf2_trans_ci2c);
    tf2_trans_m2s.mult(tf2_trans_m2c, tf2_trans_c2s_);
    tf2_trans_m2b.mult(tf2_trans_m2s, tf2_trans_b2s_.inverse());

    PointTypePose pt;
    pt.x = tf2_trans_m2b.getOrigin().x();
    pt.y = tf2_trans_m2b.getOrigin().y();
    pt.z = tf2_trans_m2b.getOrigin().z();
    tf2::Matrix3x3 m(tf2_trans_m2b.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pt.roll = roll;
    pt.pitch = pitch;
    pt.yaw = yaw;
    pt.intensity = (*it).intensity;
    cloudKeyPoses6DBaseLink.push_back(pt);

    geometry_msgs::msg::TransformStamped pose_6d_geo;
    pose_6d_geo.transform.translation.x = tf2_trans_m2b.getOrigin().x();
    pose_6d_geo.transform.translation.y = tf2_trans_m2b.getOrigin().y();
    pose_6d_geo.transform.translation.z = tf2_trans_m2b.getOrigin().z();
    pose_6d_geo.transform.rotation.x = tf2_trans_m2b.getRotation().x();
    pose_6d_geo.transform.rotation.y = tf2_trans_m2b.getRotation().y();
    pose_6d_geo.transform.rotation.z = tf2_trans_m2b.getRotation().z();
    pose_6d_geo.transform.rotation.w = tf2_trans_m2b.getRotation().w();
    cloudKeyPoses6DBaseLink_geo.push_back(pose_6d_geo);
  }  
  pcl::io::savePCDFileASCII(mapping_dir_string + "/poses.pcd", cloudKeyPoses6DBaseLink);
  
  //@ -----Write graph-----
  pcl::PointCloud<pcl::PointXYZ> edges;
  int edge_number = 0;
  for(auto it = pose_graph_.begin(); it!=pose_graph_.end(); it++){
    pcl::PointXYZ pt;
    pt.x = edge_number;
    pt.y = (*it).first;
    pt.z = (*it).second;
    edges.push_back(pt);
    edge_number++;
  }
  pcl::io::savePCDFileASCII(mapping_dir_string + "/edges.pcd", edges);

  //@ -----Write pcd-----
  std::string pcd_dir = mapping_dir_string + "/pcd";
  std::filesystem::create_directory(pcd_dir);
  for (int i = 0; i < cloudKeyPoses6D->points.size(); ++i) {

    Eigen::Affine3d af3 = tf2::transformToEigen(cloudKeyPoses6DBaseLink_geo[i]);

    keyFrameCorner.reset(new pcl::PointCloud<PointType>());
    int thisKeyInd = (int)cloudKeyPoses6D->points[i].intensity;
    *keyFrameCorner += (*cornerCloudKeyFrames[thisKeyInd]);
    *keyFrameCorner= *transformPointCloud(keyFrameCorner, &cloudKeyPoses6D->points[i]);
    pcl::transformPointCloud(*keyFrameCorner, *keyFrameCorner, trans_m2ci_af3_);
    pcl::transformPointCloud(*keyFrameCorner, *keyFrameCorner, af3.inverse());
    pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_feature.pcd", *keyFrameCorner);

    keyFrameGround.reset(new pcl::PointCloud<PointType>());
    *keyFrameGround += (*patchedGroundKeyFrames[thisKeyInd]);
    *keyFrameGround= *transformPointCloud(keyFrameGround, &cloudKeyPoses6D->points[i]);
    pcl::transformPointCloud(*keyFrameGround, *keyFrameGround, trans_m2ci_af3_);
    pcl::transformPointCloud(*keyFrameGround, *keyFrameGround, af3.inverse());
    pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_ground.pcd", *keyFrameGround);

    keyFrameSurface.reset(new pcl::PointCloud<PointType>());
    *keyFrameSurface += (*surfCloudKeyFrames[thisKeyInd]);
    *keyFrameSurface= *transformPointCloud(keyFrameSurface, &cloudKeyPoses6D->points[i]);
    pcl::transformPointCloud(*keyFrameSurface, *keyFrameSurface, trans_m2ci_af3_);
    pcl::transformPointCloud(*keyFrameSurface, *keyFrameSurface, af3.inverse());
    pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_surface.pcd", *keyFrameSurface);
  }  

}


MapOptimization::~MapOptimization()
{
  _input_channel.send({});
}


void MapOptimization::allocateMemory() {
  cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

  surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
  surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudSurfLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudPatchedGroundLast.reset(
      new pcl::PointCloud<PointType>());  // for ground pcd stitching

  laserCloudPatchedGroundEdgeLast.reset(
      new pcl::PointCloud<PointType>());  // for ground edge pcd stitching

  laserCloudCornerLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner featuer set
                                          // from odoOptimization
  laserCloudSurfLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization
  laserCloudOutlierLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudOutlierLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner feature set
                                          // from odoOptimization
  laserCloudSurfTotalLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudSurfTotalLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization

  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

  nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
  nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

  latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

  globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
  globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
  globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
  globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());
  globalGroundKeyFrames.reset(new pcl::PointCloud<PointType>());
  completeGlobalStitch.reset(new pcl::PointCloud<PointType>());


  timeLaserOdometry = 0;
  timeLastGloalMapPublish = 0;

  for (int i = 0; i < 6; ++i) {
    transformLast[i] = 0;
    transformSum[i] = 0;
    transformIncre[i] = 0;
    transformTobeMapped[i] = 0;
    transformBefMapped[i] = 0;
    transformAftMapped[i] = 0;
  }

  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  priorNoise = noiseModel::Diagonal::Variances(Vector6);
  odometryNoise = noiseModel::Diagonal::Variances(Vector6);

  matA0.setZero();
  matB0.fill(-1);
  matX0.setZero();

  matA1.setZero();
  matD1.setZero();
  matV1.setZero();

  isDegenerate = false;
  matP.setZero();

  laserCloudCornerFromMapDSNum = 0;
  laserCloudSurfFromMapDSNum = 0;
  laserCloudSurfLastDSNum = 0;
  laserCloudOutlierLastDSNum = 0;

  potentialLoopFlag = false;
  aLoopIsClosed = false;

  latestFrameID = 0;
}

void MapOptimization::loopClosureThread()
{
  if(_loop_closure_enabled){
    performLoopClosure();
  }
}


void MapOptimization::transformAssociateToMap() {

  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) -
             sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) +
             cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz +
                       calx * calz * cblx * cblz) -
              cbcx * sbcy *
                  (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                   calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                   cblx * salx * sbly) -
              cbcx * cbcy *
                  (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                   calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                   cblx * cbly * salx);
  transformTobeMapped[0] = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) -
                         cblx * sblz * (caly * calz + salx * saly * salz) +
                         calx * saly * sblx) -
                 cbcx * cbcy *
                     ((caly * calz + salx * saly * salz) *
                          (cblz * sbly - cbly * sblx * sblz) +
                      (caly * salz - calz * salx * saly) *
                          (sbly * sblz + cbly * cblz * sblx) -
                      calx * cblx * cbly * saly) +
                 cbcx * sbcy *
                     ((caly * calz + salx * saly * salz) *
                          (cbly * cblz + sblx * sbly * sblz) +
                      (caly * salz - calz * salx * saly) *
                          (cbly * sblz - cblz * sblx * sbly) +
                      calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) -
                         cblx * cblz * (saly * salz + caly * calz * salx) +
                         calx * caly * sblx) +
                 cbcx * cbcy *
                     ((saly * salz + caly * calz * salx) *
                          (sbly * sblz + cbly * cblz * sblx) +
                      (calz * saly - caly * salx * salz) *
                          (cblz * sbly - cbly * sblx * sblz) +
                      calx * caly * cblx * cbly) -
                 cbcx * sbcy *
                     ((saly * salz + caly * calz * salx) *
                          (cbly * sblz - cblz * sblx * sbly) +
                      (calz * saly - caly * salx * salz) *
                          (cbly * cblz + sblx * sbly * sblz) -
                      calx * caly * cblx * sbly);
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                 crycrx / cos(transformTobeMapped[0]));

  float srzcrx =
      (cbcz * sbcy - cbcy * sbcx * sbcz) *
          (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
           calx * calz * (sbly * sblz + cbly * cblz * sblx) +
           cblx * cbly * salx) -
      (cbcy * cbcz + sbcx * sbcy * sbcz) *
          (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
           calx * salz * (cbly * cblz + sblx * sbly * sblz) +
           cblx * salx * sbly) +
      cbcx * sbcz *
          (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  float crzcrx =
      (cbcy * sbcz - cbcz * sbcx * sbcy) *
          (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
           calx * salz * (cbly * cblz + sblx * sbly * sblz) +
           cblx * salx * sbly) -
      (sbcy * sbcz + cbcy * cbcz * sbcx) *
          (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
           calx * calz * (sbly * sblz + cbly * cblz * sblx) +
           cblx * cbly * salx) +
      cbcx * cbcz *
          (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] -
       sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] +
       cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] =
      transformAftMapped[3] -
      (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] =
      transformAftMapped[5] -
      (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void MapOptimization::transformUpdate() {
 
  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
  //transformTobeMapped: pitch, yaw, roll, y, z, x
}

void MapOptimization::updatePointAssociateToMapSinCos() {
  cRoll = cos(transformTobeMapped[0]);
  sRoll = sin(transformTobeMapped[0]);

  cPitch = cos(transformTobeMapped[1]);
  sPitch = sin(transformTobeMapped[1]);

  cYaw = cos(transformTobeMapped[2]);
  sYaw = sin(transformTobeMapped[2]);

  tX = transformTobeMapped[3];
  tY = transformTobeMapped[4];
  tZ = transformTobeMapped[5];
}

void MapOptimization::pointAssociateToMap(PointType const *const pi,
                                          PointType *const po) {
  float x1 = cYaw * pi->x - sYaw * pi->y;
  float y1 = sYaw * pi->x + cYaw * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = cRoll * y1 - sRoll * z1;
  float z2 = sRoll * y1 + cRoll * z1;

  po->x = cPitch * x2 + sPitch * z2 + tX;
  po->y = y2 + tY;
  po->z = -sPitch * x2 + cPitch * z2 + tZ;
  po->intensity = pi->intensity;
}


pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {

  //Rotation order: yaw->roll->pitch
  // pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  // PointType *pointFrom;
  // PointType pointTo;

  // int cloudSize = cloudIn->points.size();
  // cloudOut->resize(cloudSize);

  // for (int i = 0; i < cloudSize; ++i) {
  //   pointFrom = &cloudIn->points[i];
  //   float x1 = cos(transformIn->yaw) * pointFrom->x -
  //              sin(transformIn->yaw) * pointFrom->y;
  //   float y1 = sin(transformIn->yaw) * pointFrom->x +
  //              cos(transformIn->yaw) * pointFrom->y;
  //   float z1 = pointFrom->z;

  //   float x2 = x1;
  //   float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
  //   float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

  //   pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 +
  //               transformIn->x;
  //   pointTo.y = y2 + transformIn->y;
  //   pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 +
  //               transformIn->z;
  //   pointTo.intensity = pointFrom->intensity;

  //   cloudOut->points[i] = pointTo;
  // }

  pcl::PointCloud<PointType>::Ptr cloudOut2(new pcl::PointCloud<PointType>());
  
  Eigen::Affine3f af3_yaw = Eigen::Affine3f::Identity();
  af3_yaw.rotate (Eigen::AngleAxisf (transformIn->yaw, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f af3_roll = Eigen::Affine3f::Identity();
  af3_roll.rotate (Eigen::AngleAxisf (transformIn->roll, Eigen::Vector3f::UnitX()));
  Eigen::Affine3f af3_pitch = Eigen::Affine3f::Identity();
  af3_pitch.rotate (Eigen::AngleAxisf (transformIn->pitch, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f af3_translation = Eigen::Affine3f::Identity();
  af3_translation.translation() << transformIn->x, transformIn->y, transformIn->z;
  pcl_opt::transformPointCloudSequentially(*cloudIn, *cloudOut2, af3_yaw.matrix(), af3_roll.matrix(), af3_pitch.matrix(), af3_translation.matrix());

  return cloudOut2;
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloudInverse(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {


  pcl::PointCloud<PointType>::Ptr cloudOut2(new pcl::PointCloud<PointType>());
  
  Eigen::Affine3f af3_yaw = Eigen::Affine3f::Identity();
  af3_yaw.rotate (Eigen::AngleAxisf (transformIn->yaw, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f af3_roll = Eigen::Affine3f::Identity();
  af3_roll.rotate (Eigen::AngleAxisf (transformIn->roll, Eigen::Vector3f::UnitX()));
  Eigen::Affine3f af3_pitch = Eigen::Affine3f::Identity();
  af3_pitch.rotate (Eigen::AngleAxisf (transformIn->pitch, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f af3_translation = Eigen::Affine3f::Identity();
  af3_translation.translation() << transformIn->x, transformIn->y, transformIn->z;
  pcl_opt::transformPointCloudSequentially(*cloudIn, *cloudOut2, af3_translation.inverse().matrix(), af3_pitch.inverse().matrix(), 
      af3_roll.inverse().matrix(), af3_yaw.inverse().matrix());

  return cloudOut2;
}

void MapOptimization::publishTF() {

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(transformTobeMapped[2], -transformTobeMapped[0], -transformTobeMapped[1]);
  geometry_msgs::msg::Quaternion geoQuat;
  tf2::convert(quat_tf, geoQuat);

  tf2::Stamped<tf2::Transform> tf2_trans_ci2c, tf2_trans_o2b;
  tf2_trans_ci2c.setOrigin(tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
  tf2_trans_ci2c.setRotation(tf2::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  
  tf2_trans_o2b.setOrigin(tf2::Vector3(externalOdometry.pose.pose.position.x, externalOdometry.pose.pose.position.y, externalOdometry.pose.pose.position.z));
  tf2_trans_o2b.setRotation(tf2::Quaternion(externalOdometry.pose.pose.orientation.x, externalOdometry.pose.pose.orientation.y, externalOdometry.pose.pose.orientation.z, externalOdometry.pose.pose.orientation.w));

  tf2::Stamped<tf2::Transform> tf2_trans_m2c;
  tf2::Stamped<tf2::Transform> tf2_trans_m2s;
  tf2::Stamped<tf2::Transform> tf2_trans_m2b;
  tf2::Stamped<tf2::Transform> tf2_trans_m2o;

  tf2_trans_m2c.mult(tf2_trans_m2ci_, tf2_trans_ci2c);
  tf2_trans_m2s.mult(tf2_trans_m2c, tf2_trans_c2s_);
  tf2_trans_m2b.mult(tf2_trans_m2s, tf2_trans_b2s_.inverse());
  tf2_trans_m2o.mult(tf2_trans_m2b, tf2_trans_o2b.inverse());
  
  geometry_msgs::msg::TransformStamped map2odom;
  map2odom.header.frame_id = "map";

  map2odom.header.stamp = timeLaserOdometry_header_.stamp;
  map2odom.child_frame_id = externalOdometry.header.frame_id;
  map2odom.transform.rotation.x = tf2_trans_m2o.getRotation().x();
  map2odom.transform.rotation.y = tf2_trans_m2o.getRotation().y();
  map2odom.transform.rotation.z = tf2_trans_m2o.getRotation().z();
  map2odom.transform.rotation.w = tf2_trans_m2o.getRotation().w();
  map2odom.transform.translation.x = tf2_trans_m2o.getOrigin().x();
  map2odom.transform.translation.y = tf2_trans_m2o.getOrigin().y();
  map2odom.transform.translation.z = tf2_trans_m2o.getOrigin().z();
  tf_broadcaster_->sendTransform(map2odom);

  if(broadcast_external_odom_tf_){
    geometry_msgs::msg::TransformStamped odom2baselink;
    odom2baselink.header.stamp = externalOdometry.header.stamp;
    odom2baselink.header.frame_id = map2odom.child_frame_id;
    odom2baselink.child_frame_id = externalOdometry.child_frame_id;
    odom2baselink.transform.rotation.x = externalOdometry.pose.pose.orientation.x;
    odom2baselink.transform.rotation.y = externalOdometry.pose.pose.orientation.y;
    odom2baselink.transform.rotation.z = externalOdometry.pose.pose.orientation.z;
    odom2baselink.transform.rotation.w = externalOdometry.pose.pose.orientation.w;
    odom2baselink.transform.translation.x = externalOdometry.pose.pose.position.x;
    odom2baselink.transform.translation.y = externalOdometry.pose.pose.position.y;
    odom2baselink.transform.translation.z = externalOdometry.pose.pose.position.z;
    tf_broadcaster_->sendTransform(odom2baselink);
  }
}

void MapOptimization::publishKeyPosesAndFrames() {
  
  if(!has_m2ci_af3_)
    return;

  pubM2Ci->publish(trans_m2ci_);
  pubC2S->publish(trans_c2s_);
  pubB2S->publish(trans_b2s_);

  geometry_msgs::msg::PoseArray pose_array;
  for(auto it = cloudKeyPoses6D->points.begin(); it!=cloudKeyPoses6D->points.end(); it++){
    tf2::Transform tf2_trans_ci2c;
    tf2::Quaternion q;
    q.setRPY( (*it).roll, (*it).pitch, (*it).yaw);
    tf2_trans_ci2c.setRotation(q);
    tf2_trans_ci2c.setOrigin(tf2::Vector3((*it).x, (*it).y, (*it).z));

    tf2::Stamped<tf2::Transform> tf2_trans_m2c;
    tf2::Stamped<tf2::Transform> tf2_trans_m2s;
    tf2::Stamped<tf2::Transform> tf2_trans_m2b;

    tf2_trans_m2c.mult(tf2_trans_m2ci_, tf2_trans_ci2c);
    tf2_trans_m2s.mult(tf2_trans_m2c, tf2_trans_c2s_);
    tf2_trans_m2b.mult(tf2_trans_m2s, tf2_trans_b2s_.inverse());

    geometry_msgs::msg::Pose a_pose;
    a_pose.position.x = tf2_trans_m2b.getOrigin().x();
    a_pose.position.y = tf2_trans_m2b.getOrigin().y();
    a_pose.position.z = tf2_trans_m2b.getOrigin().z();
    a_pose.orientation.x = tf2_trans_m2b.getRotation().x();
    a_pose.orientation.y = tf2_trans_m2b.getRotation().y();
    a_pose.orientation.z = tf2_trans_m2b.getRotation().z();
    a_pose.orientation.w = tf2_trans_m2b.getRotation().w();
    pose_array.poses.push_back(a_pose);
  }
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = clock_->now();
  pub_key_pose_arr_->publish(pose_array);


  //@visualize edge
  visualization_msgs::msg::MarkerArray markerArray;
  int cnt = 0;
  for(auto it=pose_graph_.begin();it!=pose_graph_.end();it++){
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = "map";
    markerEdge.header.stamp = clock_->now();
    markerEdge.action = visualization_msgs::msg::Marker::MODIFY;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.pose.orientation.w = 1.0;
    markerEdge.ns = "pg_0_edge";
    markerEdge.id = 0;
    markerEdge.scale.x = 0.25;markerEdge.scale.y = 0.25;markerEdge.scale.z = 0.25;
    markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
    markerEdge.color.a = 0.9;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    p1.x = pose_array.poses[(*it).first].position.x;
    p1.y = pose_array.poses[(*it).first].position.y;
    p1.z = pose_array.poses[(*it).first].position.z; //make edge under node, make visualization easier
    p2.x = pose_array.poses[(*it).second].position.x;
    p2.y = pose_array.poses[(*it).second].position.y;
    p2.z = pose_array.poses[(*it).second].position.z;//make edge under node, make visualization easier
    markerEdge.points.push_back(p1);
    markerEdge.points.push_back(p2);
    markerEdge.id = cnt;
    markerArray.markers.push_back(markerEdge);
    cnt++;
  }

  cnt = 0;
  for(auto it=pose_array.poses.begin(); it!=pose_array.poses.end(); it++){

    visualization_msgs::msg::Marker markerPoint;
    markerPoint.header.frame_id = "map";
    markerPoint.header.stamp = clock_->now();
    markerPoint.action = visualization_msgs::msg::Marker::MODIFY;
    markerPoint.type = visualization_msgs::msg::Marker::SPHERE;
    markerPoint.ns = "pg_0_node";
    markerPoint.id = 0;
    markerPoint.scale.x = 0.25; markerPoint.scale.y = 0.25; markerPoint.scale.z = 0.25;
    markerPoint.color.r = 0.1; markerPoint.color.g = 0.1; markerPoint.color.b = 1;
    markerPoint.color.a = 0.8; 

    markerPoint.pose = (*it);
    markerPoint.id = cnt;
    markerArray.markers.push_back(markerPoint);
    cnt++;
  }
  pub_pose_graph_->publish(markerArray);
  
  sensor_msgs::msg::PointCloud2 cloud_msg_pose_6d;
  pcl::toROSMsg(*cloudKeyPoses6D, cloud_msg_pose_6d);
  cloud_msg_pose_6d.header.stamp = timeLaserOdometry_header_.stamp;
  cloud_msg_pose_6d.header.frame_id = "map";
  pubcloudKeyPoses6D->publish(cloud_msg_pose_6d);
  
}

bool MapOptimization::detectLoopClosure() {
  latestSurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloudDS->clear();

  // find the closest history key frame
  std::vector<int> pointSearchIndLoop;
  std::vector<float> pointSearchSqDisLoop;
  kdtreeHistoryKeyPoses.setInputCloud(cloudKeyPoses3D);

  kdtreeHistoryKeyPoses.radiusSearch(
      currentRobotPosPoint_, _history_keyframe_search_radius, pointSearchIndLoop,
      pointSearchSqDisLoop);
  
  double d_distance;
  closestHistoryFrameID = -1;
  std::vector<std::pair<int,double>> loop_closure_candidates;
  for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
    int id = pointSearchIndLoop[i];

    double dx = cloudKeyPoses6D->points[id].x - transformLast[3];
    double dy = cloudKeyPoses6D->points[id].y - transformLast[4];
    double dz = cloudKeyPoses6D->points[id].z - transformLast[5];
    d_distance = sqrt(dx*dx + dy*dy + dz*dz);

    //@ Check accumulated distance from closestHistoryFrameID to curent, if it is less than 20 meters, we dont need loop closure
    //@ bacause it is not possible to need loop closure with 20 meters
    //@ this setup implicity make edge distance between current frame to previous frame to exceed 20 meters
    double accumulated_distance = 0.0;
    PointTypePose ref_pt = cloudKeyPoses6D->points[id];
    for(int j = id; j<cloudKeyPoses6D->points.size(); j++){
      double dx = ref_pt.x - cloudKeyPoses6D->points[j].x;
      double dy = ref_pt.y - cloudKeyPoses6D->points[j].y;
      double dz = ref_pt.z - cloudKeyPoses6D->points[j].z;
      accumulated_distance+=sqrt(dx*dx + dy*dy + dz*dz);
      ref_pt = cloudKeyPoses6D->points[j];
    }

    if(accumulated_distance>20.0){
      loop_closure_candidates.push_back(std::make_pair((cloudKeyPoses6D->points.size()-id), d_distance));
    }

  }
  
  //increase weight for elder keyframe
  double min_distance = 999.9;

  for(auto li=loop_closure_candidates.begin();li!=loop_closure_candidates.end();li++){
    double weighted_distance = (*li).second/(((*li).first+1));
    if(weighted_distance<min_distance){
      closestHistoryFrameID = cloudKeyPoses6D->points.size() - (*li).first;
      min_distance = weighted_distance;
    }
  }

  if (closestHistoryFrameID == -1) {
    return false;
  }
  else{
    
    //calculate relative pose for ICP initial guess
    tf2_current2closestKeyFrame_.setOrigin(tf2::Vector3(cloudKeyPoses6D->points[closestHistoryFrameID].x - cloudKeyPoses6D->points.back().x, 
                                                        cloudKeyPoses6D->points[closestHistoryFrameID].y - cloudKeyPoses6D->points.back().y, 
                                                        cloudKeyPoses6D->points[closestHistoryFrameID].z - cloudKeyPoses6D->points.back().z));
    tf2_current2closestKeyFrame_.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    /*
    Remember the 'x' is left and right in real world because we are in camera frame
    Remember the 'y' is up and down in real world because we are in camera frame
    Remember the 'z' is front and rear in real world because we are in camera frame
    */
    RCLCPP_DEBUG(this->get_logger(), 
      "Closest key id: %d, relative pose: %.2f, %.2f, %.2f", closestHistoryFrameID, 
        tf2_current2closestKeyFrame_.getOrigin().x(), tf2_current2closestKeyFrame_.getOrigin().y(), tf2_current2closestKeyFrame_.getOrigin().z());
  }

  // save latest key frames
  latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
  *latestSurfKeyFrameCloud +=
      *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure],
                           &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  *latestSurfKeyFrameCloud +=
      *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure],
                           &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  
  //@ without transform function in lego loam, the original frame is camera in cornerCloudKeyFrames/surfCloudKeyFrames
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudBaseLinkFrame(new pcl::PointCloud<PointType>());

  *latestSurfKeyFrameCloudBaseLinkFrame += (*cornerCloudKeyFrames[latestFrameIDLoopCloure]);
  *latestSurfKeyFrameCloudBaseLinkFrame += (*surfCloudKeyFrames[latestFrameIDLoopCloure]);
  pcl::transformPointCloud(*latestSurfKeyFrameCloudBaseLinkFrame, *latestSurfKeyFrameCloudBaseLinkFrame, trans_s2c_af3_);
  //@ Narrow corrider should be prevented when doing icp: Narrow field of view usually introducing skewed icp result
  
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudBaseLinkFrame_Pass(new pcl::PointCloud<PointType>());
  auto original_size = latestSurfKeyFrameCloudBaseLinkFrame->points.size();
  pcl::PassThrough<PointType> pass;
  pass.setInputCloud (latestSurfKeyFrameCloudBaseLinkFrame);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-5.0, 5.0);
  pass.filter (*latestSurfKeyFrameCloudBaseLinkFrame_Pass);
  pass.setInputCloud (latestSurfKeyFrameCloudBaseLinkFrame_Pass);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3.0, 3.0);
  pass.filter (*latestSurfKeyFrameCloudBaseLinkFrame_Pass);

  if( latestSurfKeyFrameCloudBaseLinkFrame_Pass->points.size() > original_size/2){
    double dx = cloudKeyPoses6D->points[latestFrameIDLoopCloure].x - cloudKeyPoses6D->points[closestHistoryFrameID].x;
    double dy = cloudKeyPoses6D->points[latestFrameIDLoopCloure].y - cloudKeyPoses6D->points[closestHistoryFrameID].y;
    double dz = cloudKeyPoses6D->points[latestFrameIDLoopCloure].z - cloudKeyPoses6D->points[closestHistoryFrameID].z;
    if(sqrt(dx*dx+dy*dy+dz*dz)<1.0){

    }
    else{
      RCLCPP_WARN(this->get_logger(), "Too clustered! In range: %lu points, overall: %lu points. Distance between two frame is higher than 1.0 m, ignore loop closure.", latestSurfKeyFrameCloudBaseLinkFrame_Pass->points.size(), original_size);
      return false;
    }

  }
  
  // check continuity between two frame from current to candidate
  // if continuity is good, we dont need closure, solving spiral up issue
  
  if(latestFrameIDLoopCloure - closestHistoryFrameID < 200){
    double height_diff = cloudKeyPoses6D->points[latestFrameIDLoopCloure].y - cloudKeyPoses6D->points[closestHistoryFrameID].y;
    // we need to use path planning to find shortest path
    if(fabs(height_diff)>=2.0){ //wheelchair passage slope is 0.05, max is 0.1
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, 
          "Detect vertical movement in a structure, height difference between frame %d and %d is: %.4f", latestFrameIDLoopCloure, closestHistoryFrameID, fabs(height_diff));
      return false;
    }
  }
  
  // save history near key frames
  for (int j = - _history_keyframe_search_num; j <= _history_keyframe_search_num; ++j) {
    if (closestHistoryFrameID + j < 0 ||
        closestHistoryFrameID + j > latestFrameIDLoopCloure)
      continue;
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(
        cornerCloudKeyFrames[closestHistoryFrameID + j],
        &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(
        surfCloudKeyFrames[closestHistoryFrameID + j],
        &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
  }
  
  //downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
  //downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);
  downSizeFilterHistoryKeyFrames_omp.setInputCloud(nearHistorySurfKeyFrameCloud);
  downSizeFilterHistoryKeyFrames_omp.setFinalFilter(true);
  downSizeFilterHistoryKeyFrames_omp.filter(*nearHistorySurfKeyFrameCloudDS);
  // publish history near key frames
  
  sensor_msgs::msg::PointCloud2 cloudMsgTemp;
  pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
  cloudMsgTemp.header.stamp = timeLaserOdometry_header_.stamp;
  cloudMsgTemp.header.frame_id = "camera_init";
  pubIcpTargetKeyFrames->publish(cloudMsgTemp);
  
  return true;
}

void MapOptimization::performLoopClosure() {

  if (cloudKeyPoses3D->points.empty() == true)
    return;


  // try to find close key frame if there are any
  if (potentialLoopFlag == false) {
    if (detectLoopClosure() == true) {
      potentialLoopFlag = true;  // find some key frames that is old enough or
                                 // close enough for loop closure
      timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
    }
    if (potentialLoopFlag == false) return;
  }
  // reset the flag first no matter icp successes or not
  potentialLoopFlag = false;
  // ICP Settings
  /*
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  // Align clouds
  icp.setInputSource(latestSurfKeyFrameCloud);
  icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
  pcl::PointCloud<PointType>::Ptr unused_result(
      new pcl::PointCloud<PointType>());
  icp.align(*unused_result);
  RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp.getFitnessScore());
  
  if (icp.hasConverged() == false ||
      icp.getFitnessScore() > _history_keyframe_fitness_score)
  {
    return;
  }
  */
  /*
  T_predict << 1.0, 0.0, 0.0, 'x',
               0.0, 1.0, 0.0, 'y',
               0.0, 0.0, 1.0, 'z',
               0.0, 0.0, 0.0, 1.0;  
  Remember the 'y' is up and down in real world because we are in camera frame
  Remember the 'x' is left and right in real world because we are in camera frame
  Remember the 'z' is front and rear in real world because we are in camera frame
  */
  pcl::PointCloud<PointType>::Ptr cloud_source_opti_transformed_ptr;
  cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<PointType>());
  Eigen::Matrix4f T_predict, T_final;
  T_predict.setIdentity();

  //@ use relative pose as initial pose, so we can bound max torelance to 1.0
  //@ note that the TF predict is from history to current because history has elder index
  T_predict << 1.0, 0.0, 0.0, tf2_current2closestKeyFrame_.getOrigin().x(),
               0.0, 1.0, 0.0, tf2_current2closestKeyFrame_.getOrigin().y(),
               0.0, 0.0, 1.0, tf2_current2closestKeyFrame_.getOrigin().z(),
               0.0, 0.0, 0.0, 1.0;
  OptimizedICPGN icp_opti;
  icp_opti.SetTargetCloud(nearHistorySurfKeyFrameCloudDS);
  icp_opti.SetTransformationEpsilon(1e-2);
  icp_opti.SetMaxIterations(50);

  double correpond_distance = sqrt(tf2_current2closestKeyFrame_.getOrigin().x()*tf2_current2closestKeyFrame_.getOrigin().x()+
  tf2_current2closestKeyFrame_.getOrigin().y()*tf2_current2closestKeyFrame_.getOrigin().y()+
  tf2_current2closestKeyFrame_.getOrigin().z()*tf2_current2closestKeyFrame_.getOrigin().z());

  icp_opti.SetMaxCorrespondDistance(correpond_distance);
  icp_opti.Match(latestSurfKeyFrameCloud, T_predict, cloud_source_opti_transformed_ptr, T_final);
  //RCLCPP_INFO(this->get_logger(), "_history_keyframe_fitness_score: %.2f, ICP score: %.2f, convergence: %d", _history_keyframe_fitness_score, icp_opti.GetFitnessScore(), icp_opti.HasConverged());
  //!icp_opti.HasConverged() || 
  if (icp_opti.GetFitnessScore() > _history_keyframe_fitness_score)
  {
    //RCLCPP_INFO(this->get_logger(), "Edge not created from: %d to %d, ICP score: %.2f", latestFrameIDLoopCloure, closestHistoryFrameID, icp_opti.GetFitnessScore());
    return;
  }
  else{
    //RCLCPP_INFO(this->get_logger(), "_history_keyframe_fitness_score: %.2f, ICP score: %.2f, convergence: %d", _history_keyframe_fitness_score, icp_opti.GetFitnessScore(), icp_opti.HasConverged());
    //RCLCPP_INFO_STREAM(this->get_logger(), "T_final: \n" << T_final);
    RCLCPP_INFO(this->get_logger(), "Edge created from: %d to %d, ICP score: %.2f", latestFrameIDLoopCloure, closestHistoryFrameID, icp_opti.GetFitnessScore());
  }
  //RCLCPP_INFO_STREAM(this->get_logger(), "T_final: \n" << T_final);
  //RCLCPP_INFO_STREAM(this->get_logger(), "Relative: \n" << tf2_current2closestKeyFrame_af3.matrix());  
  // publish corrected cloud
  if (true) {
    pcl::PointCloud<PointType>::Ptr closed_cloud(
        new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud, 
                             T_final);
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry_header_.stamp;
    cloudMsgTemp.header.frame_id = "camera_init";
    pubIcpKeyFrames->publish(cloudMsgTemp);
  }
  //
  //        get pose constraint
  //
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correctionCameraFrame;
  correctionCameraFrame =
      T_final;  // get transformation in camera frame
                                     // (because points are in camera frame)
  pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch,
                                    yaw);
  Eigen::Affine3f correctionLidarFrame =
      pcl::getTransformation(z, x, y, yaw, roll, pitch);
  // transform from world origin to wrong pose
  Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(
      cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  // transform from world origin to corrected pose
  Eigen::Affine3f tCorrect =
      correctionLidarFrame *
      tWrong;  // pre-multiplying -> successive rotation about a fixed frame
  pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
  gtsam::Pose3 poseFrom =
      Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 poseTo =
      pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]);

  //
  //        add constraints
  //
  addEdgeFromPose(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom, poseTo, icp_opti.GetFitnessScore());
}

void MapOptimization::addEdgeFromPose(int pose_1, int pose_2, gtsam::Pose3 poseFrom, gtsam::Pose3 poseTo, float icp_score){

  std::pair<int, int> edge;
  edge.first = pose_1;
  edge.second = pose_2;

  if(!pose_graph_.insert(edge).second)
  { 
    //@ We dont need to ignore the same edge, because the isam can keep update/interate using new edge relation
    //@ test and justified on 05 July 2023
  }
    
  gtsam::Vector Vector6(6);
  float constraint_noise_mag = icp_score/10.;
  Vector6 << constraint_noise_mag, constraint_noise_mag, constraint_noise_mag, constraint_noise_mag, constraint_noise_mag,
      constraint_noise_mag;
  constraintNoise = noiseModel::Diagonal::Variances(Vector6);

  gtSAMgraph.add(
      BetweenFactor<Pose3>(pose_1, pose_2,
                           poseFrom.between(poseTo), constraintNoise));

  isam->update(gtSAMgraph);
  isam->update();
  gtSAMgraph.resize(0);
  aLoopIsClosed = true;
}

void MapOptimization::extractSurroundingKeyFrames() {

  if (cloudKeyPoses3D->points.empty() == true) return;
  
  // only use recent key poses for graph building
  if (recentCornerCloudKeyFrames.size() <
      _surrounding_keyframe_search_num) {  // queue is not full (the beginning
                                        // of mapping or a loop is just
                                        // closed)
                                        // clear recent key frames queue
    recentCornerCloudKeyFrames.clear();
    recentSurfCloudKeyFrames.clear();
    recentOutlierCloudKeyFrames.clear();
    int numPoses = cloudKeyPoses3D->points.size();
    for (int i = numPoses - 1; i >= 0; --i) {
      int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
      // extract surrounding map
      recentCornerCloudKeyFrames.push_front(
          transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]));
      recentSurfCloudKeyFrames.push_front(
          transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]));
      recentOutlierCloudKeyFrames.push_front(
          transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]));
      if (recentCornerCloudKeyFrames.size() >= _surrounding_keyframe_search_num)
        break;
    }
  } else {  // queue is full, pop the oldest key frame and push the latest
            // key frame
    if (latestFrameID != cloudKeyPoses3D->points.size() -
                              1) {  // if the robot is not moving, no need to
                                    // update recent frames

      recentCornerCloudKeyFrames.pop_front();
      recentSurfCloudKeyFrames.pop_front();
      recentOutlierCloudKeyFrames.pop_front();
      // push latest scan to the end of queue
      latestFrameID = cloudKeyPoses3D->points.size() - 1;
      recentCornerCloudKeyFrames.push_back(
          transformPointCloud(cornerCloudKeyFrames[latestFrameID], &cloudKeyPoses6D->points[latestFrameID]));
      recentSurfCloudKeyFrames.push_back(
          transformPointCloud(surfCloudKeyFrames[latestFrameID], &cloudKeyPoses6D->points[latestFrameID]));
      recentOutlierCloudKeyFrames.push_back(
          transformPointCloud(outlierCloudKeyFrames[latestFrameID], &cloudKeyPoses6D->points[latestFrameID]));
    }
  }

  for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i) {
    *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
    *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
    *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
  }
  
  // Downsample the surrounding corner key frames (or map)
  std::vector<int> indices_tmp1;
  laserCloudCornerFromMap->is_dense = false;
  pcl::removeNaNFromPointCloud(*laserCloudCornerFromMap, *laserCloudCornerFromMap, indices_tmp1);
  //downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
  //downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
  downSizeFilterCornerKeyFrame_omp.setInputCloud(laserCloudCornerFromMap);
  downSizeFilterCornerKeyFrame_omp.setFinalFilter(true);
  downSizeFilterCornerKeyFrame_omp.filter(*laserCloudCornerFromMapDS);
  laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();

  // Downsample the surrounding surf key frames (or map)
  std::vector<int> indices_tmp2;
  laserCloudSurfFromMap->is_dense = false;
  pcl::removeNaNFromPointCloud(*laserCloudSurfFromMap, *laserCloudSurfFromMap, indices_tmp2);
  //downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
  //downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
  downSizeFilterSurfKeyFrame_omp.setInputCloud(laserCloudSurfFromMap);
  downSizeFilterSurfKeyFrame_omp.setFinalFilter(true);
  downSizeFilterSurfKeyFrame_omp.filter(*laserCloudSurfFromMapDS);
  laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();

}

void MapOptimization::downsampleCurrentScan() {
  
  laserCloudCornerLastDS->clear();
  //downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  //downSizeFilterCorner.filter(*laserCloudCornerLastDS);
  downSizeFilterCorner_omp.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner_omp.setFinalFilter(true);
  downSizeFilterCorner_omp.filter(*laserCloudCornerLastDS);
  
  laserCloudSurfLastDS->clear();
  //downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  //downSizeFilterSurf.filter(*laserCloudSurfLastDS);
  downSizeFilterSurf_omp.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf_omp.setFinalFilter(true);
  downSizeFilterSurf_omp.filter(*laserCloudSurfLastDS);
  laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();
  
  laserCloudOutlierLastDS->clear();
  //downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
  //downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
  downSizeFilterOutlier_omp.setInputCloud(laserCloudOutlierLast);
  downSizeFilterOutlier_omp.setFinalFilter(true);
  downSizeFilterOutlier_omp.filter(*laserCloudOutlierLastDS);
  laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

  laserCloudSurfTotalLast->clear();
  laserCloudSurfTotalLastDS->clear();
  *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
  *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
  //downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
  //downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
  downSizeFilterSurfTotal_omp.setInputCloud(laserCloudSurfTotalLast);
  downSizeFilterSurfTotal_omp.setFinalFilter(true);
  downSizeFilterOutlier_omp.filter(*laserCloudSurfTotalLastDS);
}

void MapOptimization::cornerOptimization(int iterCount) {

  updatePointAssociateToMapSinCos();
  int cnt = 0;
  for (size_t i = 0; i < laserCloudCornerLastDS->points.size(); i++) {
    pointOri = laserCloudCornerLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    if(!pcl::isFinite(pointSel))
      continue;
    kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd,
                                        pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
      float cx = 0, cy = 0, cz = 0;
      for (int j = 0; j < 5; j++) {
        cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
        cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
        cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
      }
      cx /= 5;
      cy /= 5;
      cz /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int j = 0; j < 5; j++) {
        float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
        float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
        float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      matA1(0, 0) = a11;
      matA1(0, 1) = a12;
      matA1(0, 2) = a13;
      matA1(1, 0) = a12;
      matA1(1, 1) = a22;
      matA1(1, 2) = a23;
      matA1(2, 0) = a13;
      matA1(2, 1) = a23;
      matA1(2, 2) = a33;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);

      matD1 = esolver.eigenvalues().real();
      matV1 = esolver.eigenvectors().real();

      if (matD1[2] > 3 * matD1[1]) {
        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;
        float x1 = cx + 0.1 * matV1(0, 0);
        float y1 = cy + 0.1 * matV1(0, 1);
        float z1 = cz + 0.1 * matV1(0, 2);
        float x2 = cx - 0.1 * matV1(0, 0);
        float y2 = cy - 0.1 * matV1(0, 1);
        float z2 = cz - 0.1 * matV1(0, 2);

        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                              ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                          ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                              ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                          ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                              ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                         (z1 - z2) * (z1 - z2));

        float la =
            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
            a012 / l12;

        float lb =
            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float lc =
            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float ld2 = a012 / l12;

        float s = 1 - 0.9 * fabs(ld2);

        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        if (s > 0.1) {
          laserCloudOri->push_back(pointOri);
          cnt++;
          coeffSel->push_back(coeff);
        }
      }
    }
  }
  //RCLCPP_INFO(this->get_logger(), "Corner number: %d", cnt);
}

void MapOptimization::surfOptimization(int iterCount) {
  updatePointAssociateToMapSinCos();
  int cnt_wall = 0;
  int cnt_ground = 0;
  pcl::PointCloud<PointType> ground_pc;
  pcl::PointCloud<PointType> wall_pc;
  pcl::PointCloud<PointType> ground_coeff;
  pcl::PointCloud<PointType> wall_coeff;
  for (size_t i = 0; i < laserCloudSurfTotalLastDS->points.size(); i++) {
    pointOri = laserCloudSurfTotalLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    if(!pcl::isFinite(pointSel))
      continue;
    kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd,
                                      pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
        matA0(j, 1) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
        matA0(j, 2) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
      }
      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                 pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                 pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                 pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) /
                          sqrt(sqrt(pointSel.x * pointSel.x +
                                    pointSel.y * pointSel.y +
                                    pointSel.z * pointSel.z));

        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;
        //RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f, %.2f", pa, pb, pc, s);
        if (s > 0.1) {
          //@ increase weight of y direction (xz plane) to reduce pitch drift
          if(fabs(pb)>3*fabs(pa)+3*fabs(pc) || fabs(pointOri.y - tf2_trans_b2s_.getOrigin().z())<0.5){
            cnt_ground++;
            ground_pc.push_back(pointOri);
            ground_coeff.push_back(coeff);
          }
          else{
            cnt_wall++;
            wall_pc.push_back(pointOri);
            wall_coeff.push_back(coeff);
          }
          
          //laserCloudOri->push_back(pointOri);
          //coeffSel->push_back(coeff);
        }
      }
    }
  }
  if(cnt_wall>3*cnt_ground){
    //@ surface selection is skewed, resample them
    for(size_t i=0; i<ground_pc.points.size();i++){
        laserCloudOri->push_back(ground_pc.points[i]);
        coeffSel->push_back(ground_coeff.points[i]);
    }
    for(size_t i=0; i<wall_pc.points.size();i+=2){
        laserCloudOri->push_back(wall_pc.points[i]);
        coeffSel->push_back(wall_coeff.points[i]);
    }
  }
  else if(cnt_ground>3*cnt_wall){
    //@ surface selection is skewed, resample them
    for(size_t i=0; i<ground_pc.points.size();i+=2){
        laserCloudOri->push_back(ground_pc.points[i]);
        coeffSel->push_back(ground_coeff.points[i]);
    }
    for(size_t i=0; i<wall_pc.points.size();i++){
        laserCloudOri->push_back(wall_pc.points[i]);
        coeffSel->push_back(wall_coeff.points[i]);
    }
  }
  else{
    for(size_t i=0; i<ground_pc.points.size();i++){
        laserCloudOri->push_back(ground_pc.points[i]);
        coeffSel->push_back(ground_coeff.points[i]);
    }
    for(size_t i=0; i<wall_pc.points.size();i++){
        laserCloudOri->push_back(wall_pc.points[i]);
        coeffSel->push_back(wall_coeff.points[i]);
    }
  }
  //RCLCPP_INFO(this->get_logger(), "Wall number: %d, Ground number: %d", cnt_wall, cnt_ground);
}

bool MapOptimization::LMOptimization(int iterCount) {
  float srx = sin(transformTobeMapped[0]);
  float crx = cos(transformTobeMapped[0]);
  float sry = sin(transformTobeMapped[1]);
  float cry = cos(transformTobeMapped[1]);
  float srz = sin(transformTobeMapped[2]);
  float crz = cos(transformTobeMapped[2]);

  int laserCloudSelNum = laserCloudOri->points.size();
  if (laserCloudSelNum < 50) {
    return false;
  }

  Eigen::Matrix<float,Eigen::Dynamic,6> matA(laserCloudSelNum, 6);
  Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,laserCloudSelNum);
  Eigen::Matrix<float,6,6> matAtA;
  Eigen::VectorXf matB(laserCloudSelNum);
  Eigen::Matrix<float,6,1> matAtB;
  Eigen::Matrix<float,6,1> matX;

  for (int i = 0; i < laserCloudSelNum; i++) {
    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float arx =
        (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
         srx * sry * pointOri.z) *
            coeff.x +
        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) *
            coeff.y +
        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
         cry * srx * pointOri.z) *
            coeff.z;

    float ary =
        ((cry * srx * srz - crz * sry) * pointOri.x +
         (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) *
            coeff.x +
        ((-cry * crz - srx * sry * srz) * pointOri.x +
         (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) *
            coeff.z;

    float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                 (-cry * crz - srx * sry * srz) * pointOri.y) *
                    coeff.x +
                (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                ((sry * srz + cry * crz * srx) * pointOri.x +
                 (crz * sry - cry * srx * srz) * pointOri.y) *
                    coeff.z;

    matA(i, 0) = arx;
    matA(i, 1) = ary;
    matA(i, 2) = arz;
    matA(i, 3) = coeff.x;
    matA(i, 4) = coeff.y;
    matA(i, 5) = coeff.z;
    matB(i, 0) = -coeff.intensity;
  }
  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1,6> matE;
    Eigen::Matrix<float,6,6> matV;
    Eigen::Matrix<float,6,6> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();

     matV2 = matV;

    isDegenerate = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,6, 1> matX2(matX);
    matX2 = matX;
    matX = matP * matX2;
  }

  transformTobeMapped[0] += matX(0, 0);
  transformTobeMapped[1] += matX(1, 0);
  transformTobeMapped[2] += matX(2, 0);
  transformTobeMapped[3] += matX(3, 0);
  transformTobeMapped[4] += matX(4, 0);
  transformTobeMapped[5] += matX(5, 0);

  float deltaR = sqrt(pow(pcl::rad2deg(matX(0, 0)), 2) +
                      pow(pcl::rad2deg(matX(1, 0)), 2) +
                      pow(pcl::rad2deg(matX(2, 0)), 2));
  float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                      pow(matX(4, 0) * 100, 2) +
                      pow(matX(5, 0) * 100, 2));

  if (deltaR < 0.05 && deltaT < 0.05) {
    return true;
  }
  return false;
}

void MapOptimization::scan2MapOptimization() {

  //RCLCPP_ERROR(this->get_logger(), "%lu, %lu", laserCloudCornerFromMapDSNum, laserCloudSurfFromMapDSNum);
  if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {
    kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMapDS);
    kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMapDS);

    for (int iterCount = 0; iterCount < 10; iterCount++) {
      laserCloudOri->clear();
      coeffSel->clear();

      cornerOptimization(iterCount);
      surfOptimization(iterCount);

      if (LMOptimization(iterCount) == true) break;
    }

    transformUpdate();
    
    PointTypePose thisPose6D;
    thisPose6D.roll = transformAftMapped[0];
    thisPose6D.pitch = transformAftMapped[1];
    thisPose6D.yaw = transformAftMapped[2];
    thisPose6D.x = transformAftMapped[3];
    thisPose6D.y = transformAftMapped[4];
    thisPose6D.z = transformAftMapped[5];
    sensor_msgs::msg::PointCloud2 cloud_msg_selected_lm;
    pcl::toROSMsg(*transformPointCloud(laserCloudOri, &thisPose6D), cloud_msg_selected_lm);
    cloud_msg_selected_lm.header.stamp = timeLaserOdometry_header_.stamp;
    cloud_msg_selected_lm.header.frame_id = "camera_init";
    pubSelectedCloudForLMOptimization->publish(cloud_msg_selected_lm);
    
  }
  
}

void MapOptimization::saveKeyFramesAndFactor() {

  currentRobotPos_.x = transformAftMapped[3];
  currentRobotPos_.y = transformAftMapped[4];
  currentRobotPos_.z = transformAftMapped[5];
  currentRobotPos_.yaw = transformAftMapped[1];
  currentRobotPos_.pitch = transformAftMapped[0];
  currentRobotPos_.roll = transformAftMapped[2];

  currentRobotPosPoint_.x = transformAftMapped[3];
  currentRobotPosPoint_.y = transformAftMapped[4];
  currentRobotPosPoint_.z = transformAftMapped[5];
  
  bool saveThisKeyFrame = true;
  if (sqrt((previousRobotPos_.x - currentRobotPos_.x) *
               (previousRobotPos_.x - currentRobotPos_.x) +
           (previousRobotPos_.y - currentRobotPos_.y) *
               (previousRobotPos_.y - currentRobotPos_.y) +
           (previousRobotPos_.z - currentRobotPos_.z) *
               (previousRobotPos_.z - currentRobotPos_.z)) < distance_between_key_frame_ &&
               fabs(previousRobotPos_.yaw - currentRobotPos_.yaw) < angle_between_key_frame_) {
    saveThisKeyFrame = false;
    
  }
  
  if(current_ground_size_<100 && cloudKeyPoses3D->points.size()<10){
    saveThisKeyFrame = true;
  }

  if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty()) return;

  previousRobotPos_ = currentRobotPos_;
  //
  // update grsam graph
  //
  std::pair<int, int> edge;

  if (cloudKeyPoses3D->points.empty()) {

    edge.first = 0;
    edge.second = 0;

    gtSAMgraph.add(PriorFactor<Pose3>(
        0,
        Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                           transformTobeMapped[1]),
              Point3(transformTobeMapped[5], transformTobeMapped[3],
                     transformTobeMapped[4])),
        priorNoise));
    initialEstimate.insert(
        0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                              transformTobeMapped[1]),
                 Point3(transformTobeMapped[5], transformTobeMapped[3],
                        transformTobeMapped[4])));
    for (int i = 0; i < 6; ++i) transformLast[i] = transformTobeMapped[i];
  } else {
    
    //RCLCPP_INFO(this->get_logger(), "%.3f, %.3f ,%.3f >>> %.3f, %.3f ,%.3f", transformLast[2], transformLast[0], transformLast[1], 
    //                                                              transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]);
    edge.first = cloudKeyPoses3D->points.size() - 1;
    edge.second = cloudKeyPoses3D->points.size();
    // RCLCPP_INFO(this->get_logger(), "%.3f, %.3f, %.3f", fabs(transformLast[0]-transformAftMapped[0]), transformSum[0], transformAftMapped[0]);
    // if(fabs(transformLast[0]-transformAftMapped[0])<0.015){
    //   auto aftmapped_pitch = transformAftMapped[0];
    //   transformAftMapped[0] = (0.1*transformAftMapped[0]+0.9*transformLast[0]);
    //   if(fabs(transformSum[0])<0.2 && fabs(aftmapped_pitch)<0.1){
    //     transformAftMapped[4] = (0.1*transformAftMapped[4]+0.9*transformLast[4]);
    //     //
    //   }
    // }
    
    //@ second pose should be computed and stored after optimization
    gtsam::Pose3 poseFrom = Pose3(
        Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
        Point3(transformLast[5], transformLast[3], transformLast[4]));
    gtsam::Pose3 poseTo =
        Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4]));
    gtSAMgraph.add(BetweenFactor<Pose3>(
        cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(),
        poseFrom.between(poseTo), odometryNoise));
    initialEstimate.insert(
        cloudKeyPoses3D->points.size(),
        Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4])));
  }
  //
  // update iSAM
  //
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  //
  // save key poses
  //
  PointType thisPose3D;
  PointTypePose thisPose6D;
  Pose3 latestEstimate;

  isamCurrentEstimate = isam->calculateEstimate();
  latestEstimate =
      isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

  thisPose3D.x = latestEstimate.translation().y();
  thisPose3D.y = latestEstimate.translation().z();
  thisPose3D.z = latestEstimate.translation().x();
  thisPose3D.intensity =
      cloudKeyPoses3D->points.size();  // this can be used as index
  cloudKeyPoses3D->push_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
  thisPose6D.roll = latestEstimate.rotation().pitch();
  thisPose6D.pitch = latestEstimate.rotation().yaw();
  thisPose6D.yaw = latestEstimate.rotation().roll();  // in camera frame
  thisPose6D.time = timeLaserOdometry;
  cloudKeyPoses6D->push_back(thisPose6D);
  
  //
  // save updated transform
  //
  if (cloudKeyPoses3D->points.size() > 1) {
    transformAftMapped[0] = latestEstimate.rotation().pitch();
    transformAftMapped[1] = latestEstimate.rotation().yaw();
    transformAftMapped[2] = latestEstimate.rotation().roll();
    transformAftMapped[3] = latestEstimate.translation().y();
    transformAftMapped[4] = latestEstimate.translation().z();
    transformAftMapped[5] = latestEstimate.translation().x();

    for (int i = 0; i < 6; ++i) {
      transformLast[i] = transformAftMapped[i];
      transformTobeMapped[i] = transformAftMapped[i];
    }
  }
  
  // publish recent and optimized result for visual check
  sensor_msgs::msg::PointCloud2 cloud_msg_corner_from_map;
  pcl::toROSMsg(*laserCloudCornerFromMapDS, cloud_msg_corner_from_map);
  cloud_msg_corner_from_map.header.stamp = timeLaserOdometry_header_.stamp;
  cloud_msg_corner_from_map.header.frame_id = "camera_init";
  pubRecentCornerKeyFrames->publish(cloud_msg_corner_from_map);

  sensor_msgs::msg::PointCloud2 cloud_msg_surf_from_map;
  pcl::toROSMsg(*laserCloudSurfFromMapDS, cloud_msg_surf_from_map);
  cloud_msg_surf_from_map.header.stamp = timeLaserOdometry_header_.stamp;
  cloud_msg_surf_from_map.header.frame_id = "camera_init";
  pubRecentSurfKeyFrames->publish(cloud_msg_surf_from_map);

  sensor_msgs::msg::PointCloud2 cloud_msg_opt_corner;
  pcl::toROSMsg(*transformPointCloud(laserCloudCornerLastDS, &thisPose6D), cloud_msg_opt_corner);
  cloud_msg_opt_corner.header.stamp = timeLaserOdometry_header_.stamp;
  cloud_msg_opt_corner.header.frame_id = "camera_init";
  pubLastOptimizedCornerKeyFrames->publish(cloud_msg_opt_corner);

  sensor_msgs::msg::PointCloud2 cloud_msg_opt_surf;
  pcl::toROSMsg(*transformPointCloud(laserCloudSurfLastDS, &thisPose6D), cloud_msg_opt_surf);
  cloud_msg_opt_surf.header.stamp = timeLaserOdometry_header_.stamp;
  cloud_msg_opt_surf.header.frame_id = "camera_init";
  pubLastOptimizedSurfKeyFrames->publish(cloud_msg_opt_surf);

  //@update graph
  if(!pose_graph_.insert(edge).second)
  {   
    RCLCPP_ERROR(this->get_logger(), "It is not possible! We are inserting the same edge in ISAM.");
  }

  
  pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisPatchedGroundKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisPatchedGroundEdgeKeyFrame(
      new pcl::PointCloud<PointType>());

  pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
  pcl::copyPointCloud(*laserCloudPatchedGroundLast, *thisPatchedGroundKeyFrame);
  pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);
  pcl::copyPointCloud(*laserCloudPatchedGroundEdgeLast, *thisPatchedGroundEdgeKeyFrame);

  cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  surfCloudKeyFrames.push_back(thisSurfKeyFrame);
  patchedGroundKeyFrames.push_back(thisPatchedGroundKeyFrame);
  patchedGroundEdgeKeyFrames.push_back(thisPatchedGroundEdgeKeyFrame);
  outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);

}

void MapOptimization::correctPoses() {

  if (aLoopIsClosed == true) {
    recentCornerCloudKeyFrames.clear();
    recentSurfCloudKeyFrames.clear();
    recentOutlierCloudKeyFrames.clear();
    // update key poses
    int numPoses = isamCurrentEstimate.size();
    for (int i = 0; i < numPoses; ++i) {
      cloudKeyPoses3D->points[i].x =
          isamCurrentEstimate.at<Pose3>(i).translation().y();
      cloudKeyPoses3D->points[i].y =
          isamCurrentEstimate.at<Pose3>(i).translation().z();
      cloudKeyPoses3D->points[i].z =
          isamCurrentEstimate.at<Pose3>(i).translation().x();

      cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
      cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
      cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
      cloudKeyPoses6D->points[i].roll =
          isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
      cloudKeyPoses6D->points[i].pitch =
          isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
      cloudKeyPoses6D->points[i].yaw =
          isamCurrentEstimate.at<Pose3>(i).rotation().roll();
    }
    aLoopIsClosed = false;
  }
}

void MapOptimization::clearCloud() {

  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  laserCloudCornerFromMapDS->clear();
  laserCloudSurfFromMapDS->clear();
}


void MapOptimization::run() {
  
  AssociationOut association;
  _input_channel.receive(association);

  laserCloudCornerLast = association.cloud_corner_last;
  laserCloudSurfLast = association.cloud_surf_last;
  laserCloudOutlierLast = association.cloud_outlier_last;

  //pcl::copyPointCloud(association.cloud_corner_last, *laserCloudCornerLast);
  nav_msgs::msg::Odometry decisive_odometry = std::move(association.decisive_odometry);

  timeLaserOdometry = decisive_odometry.header.stamp.sec + decisive_odometry.header.stamp.nanosec/1000000000.;
  timeLaserOdometry_header_.stamp = decisive_odometry.header.stamp;

  trans_c2s_af3_ = tf2::transformToEigen(association.trans_c2s);
  trans_s2c_af3_ = trans_c2s_af3_.inverse();
  trans_b2s_ = association.trans_b2s;
  trans_b2s_af3_ = tf2::transformToEigen(association.trans_b2s);
  trans_c2s_ = association.trans_c2s;
  tf2_trans_c2s_.setRotation(tf2::Quaternion(association.trans_c2s.transform.rotation.x, association.trans_c2s.transform.rotation.y, association.trans_c2s.transform.rotation.z, association.trans_c2s.transform.rotation.w));
  tf2_trans_c2s_.setOrigin(tf2::Vector3(association.trans_c2s.transform.translation.x, association.trans_c2s.transform.translation.y, association.trans_c2s.transform.translation.z));
  tf2_trans_b2s_.setRotation(tf2::Quaternion(association.trans_b2s.transform.rotation.x, association.trans_b2s.transform.rotation.y, association.trans_b2s.transform.rotation.z, association.trans_b2s.transform.rotation.w));
  tf2_trans_b2s_.setOrigin(tf2::Vector3(association.trans_b2s.transform.translation.x, association.trans_b2s.transform.translation.y, association.trans_b2s.transform.translation.z));
  trans_m2ci_ = association.trans_m2ci;
  trans_m2ci_af3_ = tf2::transformToEigen(association.trans_m2ci); //for pcl conversion
  tf2_trans_m2ci_.setRotation(tf2::Quaternion(association.trans_m2ci.transform.rotation.x, association.trans_m2ci.transform.rotation.y, association.trans_m2ci.transform.rotation.z, association.trans_m2ci.transform.rotation.w));
  tf2_trans_m2ci_.setOrigin(tf2::Vector3(association.trans_m2ci.transform.translation.x, association.trans_m2ci.transform.translation.y, association.trans_m2ci.transform.translation.z));
  has_m2ci_af3_ = true;
  externalOdometry = association.external_odometry;
  
  
  pcl::transformPointCloud(*association.cloud_patched_ground_last, *laserCloudPatchedGroundLast, trans_c2s_af3_);
  pcl::transformPointCloud(*association.cloud_patched_ground_edge_last, *laserCloudPatchedGroundEdgeLast, trans_c2s_af3_);

  OdometryToTransform(decisive_odometry, transformSum);

  transformAssociateToMap();
  
  extractSurroundingKeyFrames();
  
  downsampleCurrentScan();
  
  scan2MapOptimization();

  saveKeyFramesAndFactor();

  correctPoses();

  publishTF();

  publishKeyPosesAndFrames();

  clearCloud();
  
}

void MapOptimization::runWoLO(){
  
  AssociationOut association;
  _input_channel.receive(association);

  laserCloudCornerLast = association.cloud_corner_last;
  laserCloudSurfLast = association.cloud_surf_last;
  laserCloudOutlierLast = association.cloud_outlier_last;

  //pcl::copyPointCloud(association.cloud_corner_last, *laserCloudCornerLast);
  nav_msgs::msg::Odometry decisive_odometry = std::move(association.decisive_odometry);

  timeLaserOdometry = decisive_odometry.header.stamp.sec + decisive_odometry.header.stamp.nanosec/1000000000.;
  timeLaserOdometry_header_.stamp = decisive_odometry.header.stamp;

  trans_c2s_af3_ = tf2::transformToEigen(association.trans_c2s);
  trans_s2c_af3_ = trans_c2s_af3_.inverse();
  trans_b2s_af3_ = tf2::transformToEigen(association.trans_b2s);
  tf2_trans_c2s_.setRotation(tf2::Quaternion(association.trans_c2s.transform.rotation.x, association.trans_c2s.transform.rotation.y, association.trans_c2s.transform.rotation.z, association.trans_c2s.transform.rotation.w));
  tf2_trans_c2s_.setOrigin(tf2::Vector3(association.trans_c2s.transform.translation.x, association.trans_c2s.transform.translation.y, association.trans_c2s.transform.translation.z));
  tf2_trans_b2s_.setRotation(tf2::Quaternion(association.trans_b2s.transform.rotation.x, association.trans_b2s.transform.rotation.y, association.trans_b2s.transform.rotation.z, association.trans_b2s.transform.rotation.w));
  tf2_trans_b2s_.setOrigin(tf2::Vector3(association.trans_b2s.transform.translation.x, association.trans_b2s.transform.translation.y, association.trans_b2s.transform.translation.z));
  has_m2ci_af3_ = true;

  /*
  decisive_odometry.stamp = laser_cloud.stamp
  external_odometry.stamp = odom.stamp
  */
  externalOdometry = association.external_odometry;

  pcl::transformPointCloud(*association.cloud_patched_ground_last, *laserCloudPatchedGroundLast, trans_c2s_af3_);
  pcl::transformPointCloud(*association.cloud_patched_ground_edge_last, *laserCloudPatchedGroundEdgeLast, trans_c2s_af3_);

  OdometryToTransform(decisive_odometry, transformSum);

  transformAssociateToMap();

  publishTF();

  clearCloud();
  
}