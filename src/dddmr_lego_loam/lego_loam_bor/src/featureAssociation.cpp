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
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "featureAssociation.h"

const float RAD2DEG = 180.0 / M_PI;

FeatureAssociation::FeatureAssociation(std::string name, Channel<ProjectionOut> &input_channel,
                                       Channel<AssociationOut> &output_channel)
    : Node(name),
      _input_channel(input_channel),
      _output_channel(output_channel){
  
  initialize_laser_odom_at_first_frame_ = false;
  odom_topic_alive_ = false;
  odom_tf_alive_ = false;
  odom_tf_detect_number_ = 0;
  baselink_frame_ = "base_link";
  //@ this cloud is for localization, therefore, we need good QoS
  pubCornerPointsSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("laser_cloud_sharp", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pubCornerPointsLessSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("laser_cloud_less_sharp", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pubSurfPointsFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("laser_cloud_flat", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pubSurfPointsLessFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("laser_cloud_less_flat", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  _pub_cloud_corner_last = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("laser_cloud_corner_last", 1);
  _pub_cloud_surf_last = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("laser_cloud_surf_last", 1);
  _pub_outlier_cloudLast = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("outlier_cloud_last", 1);
  pubLaserOdometry = this->create_publisher<nav_msgs::msg::Odometry>
      ("laser_odom_to_init", 1);
  pubOriginizedWheelOdometryPath = this->create_publisher<nav_msgs::msg::Path>
      ("originized_wheel_odom_path", 1);
  pubOriginizedLaserOdometryPath = this->create_publisher<nav_msgs::msg::Path>
      ("originized_laser_odom_path", 1);

  //TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  clock_ = this->get_clock();

  declare_parameter("featureAssociation.edge_threshold", rclcpp::ParameterValue(0.0));
  this->get_parameter("featureAssociation.edge_threshold", _edge_threshold);
  RCLCPP_INFO(this->get_logger(), "featureAssociation.edge_threshold: %.2f", _edge_threshold);

  declare_parameter("featureAssociation.surf_threshold", rclcpp::ParameterValue(0.0));
  this->get_parameter("featureAssociation.surf_threshold", _surf_threshold);
  RCLCPP_INFO(this->get_logger(), "featureAssociation.surf_threshold: %.2f", _surf_threshold);
  
  float nearest_dist;
  declare_parameter("featureAssociation.nearest_feature_search_distance", rclcpp::ParameterValue(0.0));
  this->get_parameter("featureAssociation.nearest_feature_search_distance", nearest_dist);
  RCLCPP_INFO(this->get_logger(), "featureAssociation.nearest_feature_search_distance: %.2f", nearest_dist);

  _nearest_feature_dist_sqr = nearest_dist*nearest_dist;
 
  declare_parameter("mapping.to_map_optimization", rclcpp::ParameterValue(true));
  this->get_parameter("mapping.to_map_optimization", to_map_optimization_);
  RCLCPP_INFO(this->get_logger(), "mapping.to_map_optimization: %d", to_map_optimization_);

  odom_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); 

  
  first_odom_prepared_ = false;
  odom_sanity_check_ = false;

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = odom_cb_group_;
  subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&FeatureAssociation::odomHandler, this, std::placeholders::_1), sub_options);

  //initializationValue();
  timer_ = this->create_wall_timer(10ms, std::bind(&FeatureAssociation::runFeatureAssociation, this), timer_cb_group_);
}

FeatureAssociation::~FeatureAssociation()
{
  _input_channel.send({});
}

void FeatureAssociation::initializeValue() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  cloudSmoothness.resize(cloud_size);

  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilter_omp.setNumberOfThreads(6);
  downSizeFilter_omp.setLeafSize (0.2, 0.2, 0.2);
  downSizeFilter_omp.setSaveLeafLayout(false);

  segmentedCloud.reset(new pcl::PointCloud<PointType>());
  outlierCloud.reset(new pcl::PointCloud<PointType>());

  cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
  cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
  surfPointsFlat.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

  surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

  cloudCurvature.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size);
  cloudLabel.resize(cloud_size);

  pointSelCornerInd.resize(cloud_size);
  pointSearchCornerInd1.resize(cloud_size);
  pointSearchCornerInd2.resize(cloud_size);

  pointSelSurfInd.resize(cloud_size);
  pointSearchSurfInd1.resize(cloud_size);
  pointSearchSurfInd2.resize(cloud_size);
  pointSearchSurfInd3.resize(cloud_size);

  systemInitCount = 0;
  systemInited = false;

  skipFrameNum = 1;

  for (int i = 0; i < 6; ++i) {
    transformCur[i] = 0;
    transformLaserOdometrySum[i] = 0;
  }

  systemInitedLM = false;

  laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());

  if(odom_type_=="wheel_odometry"){
    laser_odom_path_.header.frame_id = "camera_wheel_based";
    wheel_odom_path_.header.frame_id = "camera_wheel_based";
  }
  else{
    laser_odom_path_.header.frame_id = "camera_laser_based";
    wheel_odom_path_.header.frame_id = "camera_laser_based";
  }

  
  mappingOdometry.header.frame_id = "camera_init";
  mappingOdometry.child_frame_id = "laser_odom";

  isDegenerate = false;

  frameCount = skipFrameNum;
}

void FeatureAssociation::tfInitial(){

  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
}

void FeatureAssociation::odomHandler(const nav_msgs::msg::Odometry::SharedPtr odomIn){
  
  odom_topic_alive_ = true;
  exteralOdometry = (*odomIn);

  if(!odom_tf_alive_ && odom_tf_detect_number_<3){

    odom_tf_detect_number_++;
    try
    {
      geometry_msgs::msg::TransformStamped trans_o2b;
      trans_o2b = tf2Buffer_->lookupTransform(
          odomIn->header.frame_id, odomIn->child_frame_id, tf2::TimePointZero);
      odom_tf_alive_ = true;
      RCLCPP_INFO(this->get_logger(), "Detect %s to %s TF, MO will not broadcast from %s to %s.", odomIn->header.frame_id.c_str(), baselink_frame_.c_str(), odomIn->header.frame_id.c_str(), baselink_frame_.c_str());
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_DEBUG(this->get_logger(), "Could not get %s to %s, check your odom topic and baselink_frame", odomIn->child_frame_id.c_str(), baselink_frame_.c_str());
    }
    
  }

  
  if(odom_type_!="wheel_odometry"){
    return;
  }

  //ROS_INFO_ONCE("Recieve odom.");
  //@ If odom is not initilized with 0, we record it and then transform it
  //@ Because odom in lego loam should always start with 0
  if(!first_odom_prepared_){
    tf2_first_odom2b_.setOrigin(tf2::Vector3(odomIn->pose.pose.position.x, odomIn->pose.pose.position.y, odomIn->pose.pose.position.z));
    tf2_first_odom2b_.setRotation(tf2::Quaternion(odomIn->pose.pose.orientation.x, odomIn->pose.pose.orientation.y, odomIn->pose.pose.orientation.z, odomIn->pose.pose.orientation.w));
    tf2_first_odom2s_.mult(tf2_first_odom2b_, tf2_trans_b2s_);
    tf2_first_odom2s_inverse_ = tf2_first_odom2s_.inverse();
    first_odom_prepared_ = true;
    RCLCPP_WARN(this->get_logger(), "The first odom is record: %.2f, %.2f, %.2f", odomIn->pose.pose.position.x, odomIn->pose.pose.position.y, odomIn->pose.pose.position.z);
  }
  
  // sanity check of odom->child_frame to base_link frame 
  if(!odom_sanity_check_){
    try
    {
      geometry_msgs::msg::TransformStamped trans_o2b;
      trans_o2b = tf2Buffer_->lookupTransform(
          odomIn->child_frame_id, baselink_frame_, tf2::TimePointZero);
  
      odom_sanity_check_= true;
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not get %s to %s, check your odom topic and baselink frame", odomIn->child_frame_id.c_str(), baselink_frame_.c_str());
    }
  }

  tf2::Transform tf2_trans_o2b, tf2_trans_o2s, tf2_trans_first_s2s;
  tf2_trans_o2b.setOrigin(tf2::Vector3(odomIn->pose.pose.position.x, odomIn->pose.pose.position.y, odomIn->pose.pose.position.z));
  tf2_trans_o2b.setRotation(tf2::Quaternion(odomIn->pose.pose.orientation.x, odomIn->pose.pose.orientation.y, odomIn->pose.pose.orientation.z, odomIn->pose.pose.orientation.w));
  tf2_trans_o2s.mult(tf2_trans_o2b, tf2_trans_b2s_);

  tf2_trans_first_s2s.mult(tf2_first_odom2s_inverse_, tf2_trans_o2s);

  //@Get RPY
  tf2::Matrix3x3 m(tf2_trans_first_s2s.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double odom_time = static_cast<double>(odomIn->header.stamp.sec) + static_cast<double>(odomIn->header.stamp.nanosec) * 1e-9;
  double cloud_time = static_cast<double>(segInfo.header.stamp.sec) + static_cast<double>(segInfo.header.stamp.nanosec) * 1e-9;
  //if(fabs(odom_time-cloud_time)<0.05 || fabs(odom_time)<=0.1 || fabs(cloud_time)<=0.1){ //@ try to handle 0 timestamp
  
    transformExternalOdometrySum[0] = pitch;
    transformExternalOdometrySum[1] = yaw;
    transformExternalOdometrySum[2] = roll;
    transformExternalOdometrySum[3] = tf2_trans_first_s2s.getOrigin().y();
    transformExternalOdometrySum[4] = tf2_trans_first_s2s.getOrigin().z();
    transformExternalOdometrySum[5] = tf2_trans_first_s2s.getOrigin().x();
    
  //}
  
  //RCLCPP_WARN(this->get_logger(), "%.2f, %.2f, %.2f <> %.2f, %.2f, %.2f", roll, pitch, yaw, odomIn->pose.pose.position.x, odomIn->pose.pose.position.y, odomIn->pose.pose.position.z);
}

void FeatureAssociation::adjustDistortion() {
  bool halfPassed = false;
  int cloudSize = segmentedCloud->points.size();
  
  PointType point;

  for (int i = 0; i < cloudSize; i++) {
    point.x = segmentedCloud->points[i].y;
    point.y = segmentedCloud->points[i].z;
    point.z = segmentedCloud->points[i].x;
    
    float ori = -atan2(segmentedCloud->points[i].y, segmentedCloud->points[i].x);
    //float ori = -atan2(segmentedCloud_pitch_corrected->points[i].y, segmentedCloud_pitch_corrected->points[i].x);
    if (!halfPassed) {
      if (ori < segInfo.start_orientation - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.start_orientation + M_PI * 3 / 2)
        ori -= 2 * M_PI;

      if (ori - segInfo.start_orientation > M_PI) halfPassed = true;
    } else {
      ori += 2 * M_PI;

      if (ori < segInfo.end_orientation - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.end_orientation + M_PI / 2)
        ori -= 2 * M_PI;
    }
    
    /*
    double ori2;
    if(segmentedCloud->points[i].x<0 && segmentedCloud->points[i].y>=0){
      ori2 = -atan2(segmentedCloud->points[i].y, segmentedCloud->points[i].x) + 2 * M_PI;
    }
    else if(segmentedCloud->points[i].x<0 && segmentedCloud->points[i].y<0){
      ori2 = -atan2(segmentedCloud->points[i].y, segmentedCloud->points[i].x);
    }
    else if(segmentedCloud->points[i].x>=0 && segmentedCloud->points[i].y>=0){
      ori2 = -atan2(segmentedCloud->points[i].y, segmentedCloud->points[i].x) + 2 * M_PI;
    }
    else if(segmentedCloud->points[i].x>=0 && segmentedCloud->points[i].y<0){
      ori2 = -atan2(segmentedCloud->points[i].y, segmentedCloud->points[i].x);
      if(ori2>=1.5707963267948966)
        ori2 = M_PI / 2.;
      else if(ori2<1.5707963267948966 && ori2>1)
        ori2 = -atan2(segmentedCloud->points[i].y, segmentedCloud->points[i].x);
      else
        ori2 += 2 * M_PI;
    }

    if(fabs(ori-ori2)>0.01){
      RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f, %.2f", segmentedCloud->points[i].x, segmentedCloud->points[i].y, ori, ori2);
    }
    */

    float relTime = (ori - segInfo.start_orientation) / segInfo.orientation_diff;
    if(_scan_period<=0.0){
      //keep original time
    }
    else{
      point.intensity = int(segmentedCloud->points[i].intensity) + _scan_period * relTime;
    }
    segmentedCloud->points[i] = point;
  }

}

void FeatureAssociation::calculateSmoothness() {
  int cloudSize = segmentedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange = segInfo.segmented_cloud_range[i - 5] +
                      segInfo.segmented_cloud_range[i - 4] +
                      segInfo.segmented_cloud_range[i - 3] +
                      segInfo.segmented_cloud_range[i - 2] +
                      segInfo.segmented_cloud_range[i - 1] -
                      segInfo.segmented_cloud_range[i] * 10 +
                      segInfo.segmented_cloud_range[i + 1] +
                      segInfo.segmented_cloud_range[i + 2] +
                      segInfo.segmented_cloud_range[i + 3] +
                      segInfo.segmented_cloud_range[i + 4] +
                      segInfo.segmented_cloud_range[i + 5];

    cloudCurvature[i] = diffRange * diffRange;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureAssociation::markOccludedPoints() {
  int cloudSize = segmentedCloud->points.size();

  for (int i = 5; i < cloudSize - 6; ++i) {
    float depth1 = segInfo.segmented_cloud_range[i];
    float depth2 = segInfo.segmented_cloud_range[i + 1];
    int columnDiff = std::abs(int(segInfo.segmented_cloud_col_ind[i + 1] -
                                  segInfo.segmented_cloud_col_ind[i]));

    if (columnDiff < 10) {
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(segInfo.segmented_cloud_range[i - 1] -
                           segInfo.segmented_cloud_range[i]);
    float diff2 = std::abs(segInfo.segmented_cloud_range[i + 1] -
                           segInfo.segmented_cloud_range[i]);

    if (diff1 > 0.02 * segInfo.segmented_cloud_range[i] &&
        diff2 > 0.02 * segInfo.segmented_cloud_range[i])
      cloudNeighborPicked[i] = 1;
  }
}

void FeatureAssociation::extractFeatures() {
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  for (int i = 0; i < _vertical_scans; i++) {
    surfPointsLessFlatScan->clear();

    for (int j = 0; j < 6; j++) {
      int sp =
          (segInfo.start_ring_index[i] * (6 - j) + segInfo.end_ring_index[i] * j) /
          6;
      int ep = (segInfo.start_ring_index[i] * (5 - j) +
                segInfo.end_ring_index[i] * (j + 1)) /
                   6 -
               1;

      if (sp >= ep) continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > _edge_threshold && 
            segInfo.segmented_cloud_ground_flag[ind] == false) {
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if( ind + l >= segInfo.segmented_cloud_col_ind.size() ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                             segInfo.segmented_cloud_col_ind[ind + l - 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if( ind + l < 0 ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                             segInfo.segmented_cloud_col_ind[ind + l + 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < _surf_threshold &&
            segInfo.segmented_cloud_ground_flag[ind] == true) {
          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(segmentedCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if( ind + l >= segInfo.segmented_cloud_col_ind.size() ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                             segInfo.segmented_cloud_col_ind.at(ind + l - 1)));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (ind + l < 0) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                             segInfo.segmented_cloud_col_ind.at(ind + l + 1)));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
        }
      }
    }

    //
    // double ground_height = 0.0;
    // for(auto it=surfPointsFlat->points.begin(); it!=surfPointsFlat->points.end();it++){
    //   ground_height+=(*it).y;
    // }
    // //RCLCPP_INFO(this->get_logger(), "Averaged height: %.2f", ground_height/surfPointsFlat->points.size());
    // surfPointsFlatFiltered.reset(new pcl::PointCloud<PointType>());
    // double averaged_height = ground_height/surfPointsFlat->points.size();
    // for(auto it=segmentedCloud->points.begin();it!=segmentedCloud->points.end();it++){
    //   float range = hypot((*it).x, (*it).z);
    //   if(range > 5.0 && fabs((*it).y-averaged_height)<0.05){
    //     surfPointsFlatFiltered->push_back((*it));
    //   }
    // }
    
    // surfPointsFlat = surfPointsFlatFiltered;

    surfPointsLessFlatScanDS->clear();
    //downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    //downSizeFilter.filter(*surfPointsLessFlatScanDS);
    downSizeFilter_omp.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter_omp.setFinalFilter(true);
    downSizeFilter_omp.filter(*surfPointsLessFlatScanDS);
    
    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
}

void FeatureAssociation::TransformToStart(PointType const *const pi,
                                          PointType *const po) {
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

void FeatureAssociation::TransformToEnd(PointType const *const pi,
                                        PointType *const po) {
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transformCur[0];
  ry = transformCur[1];
  rz = transformCur[2];
  tx = transformCur[3];
  ty = transformCur[4];
  tz = transformCur[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  po->x = x6;
  po->y = y6;
  po->z = z6;
  po->intensity = int(pi->intensity);
}

void FeatureAssociation::AccumulateRotation(float cx, float cy, float cz,
                                            float lx, float ly, float lz,
                                            float &ox, float &oy, float &oz) {
  float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) -
              cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
  ox = -asin(srx);

  float srycrx =
      sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) +
      cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) +
      cos(lx) * cos(ly) * cos(cx) * sin(cy);
  float crycrx =
      cos(lx) * cos(ly) * cos(cx) * cos(cy) -
      cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) -
      sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx =
      sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) +
      cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) +
      cos(lx) * cos(cx) * cos(cz) * sin(lz);
  float crzcrx =
      cos(lx) * cos(lz) * cos(cx) * cos(cz) -
      cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) -
      sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void FeatureAssociation::findCorrespondingCornerFeatures(int iterCount) {
  int cornerPointsSharpNum = cornerPointsSharp->points.size();

  for (int i = 0; i < cornerPointsSharpNum; i++) {
    PointType pointSel;
    TransformToStart(&cornerPointsSharp->points[i], &pointSel);
    if(!pcl::isFinite(pointSel))
      continue;
    if (iterCount % 5 == 0) {
      kdtreeCornerLast.nearestKSearch(pointSel, 1, pointSearchInd,
                                       pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1;

      if (pointSearchSqDis[0] < _nearest_feature_dist_sqr) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan =
            int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = _nearest_feature_dist_sqr;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
          if (int(laserCloudCornerLast->points[j].intensity) >
              closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) >
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudCornerLast->points[j].intensity) <
              closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) <
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
      }

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    }

    if (pointSearchCornerInd2[i] >= 0) {
      PointType tripod1 =
          laserCloudCornerLast->points[pointSearchCornerInd1[i]];
      PointType tripod2 =
          laserCloudCornerLast->points[pointSearchCornerInd2[i]];

      float x0 = pointSel.x;
      float y0 = pointSel.y;
      float z0 = pointSel.z;
      float x1 = tripod1.x;
      float y1 = tripod1.y;
      float z1 = tripod1.z;
      float x2 = tripod2.x;
      float y2 = tripod2.y;
      float z2 = tripod2.z;

      float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
      float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
      float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

      float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                       (z1 - z2) * (z1 - z2));

      float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

      float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

      float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

      float ld2 = a012 / l12;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * fabs(ld2);
      }

      if (s > 0.1 && ld2 != 0) {
        PointType coeff;
        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        laserCloudOri->push_back(cornerPointsSharp->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

void FeatureAssociation::findCorrespondingSurfFeatures(int iterCount) {
  int surfPointsFlatNum = surfPointsFlat->points.size();

  for (int i = 0; i < surfPointsFlatNum; i++) {
    PointType pointSel;
    TransformToStart(&surfPointsFlat->points[i], &pointSel);
    if(!pcl::isFinite(pointSel))
      continue;
    if (iterCount % 5 == 0) {
      kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd,
                                     pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

      if (pointSearchSqDis[0] < _nearest_feature_dist_sqr) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan =
            int(laserCloudSurfLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = _nearest_feature_dist_sqr,
                          minPointSqDis3 = _nearest_feature_dist_sqr;
        for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
          if (int(laserCloudSurfLast->points[j].intensity) >
              closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) <=
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudSurfLast->points[j].intensity) <
              closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) >=
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
      }

      pointSearchSurfInd1[i] = closestPointInd;
      pointSearchSurfInd2[i] = minPointInd2;
      pointSearchSurfInd3[i] = minPointInd3;
    }

    if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
      PointType tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
      PointType tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
      PointType tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

      float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) -
                 (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
      float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) -
                 (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
      float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) -
                 (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
      float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

      float ps = sqrt(pa * pa + pb * pb + pc * pc);

      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 -
            1.8 * fabs(pd2) /
                sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y +
                          pointSel.z * pointSel.z));
      }

      if (s > 0.1 && pd2 != 0) {
        PointType coeff;
        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        laserCloudOri->push_back(surfPointsFlat->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

bool FeatureAssociation::calculateTransformationSurf(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,3> matA(pointSelNum, 3);
  Eigen::Matrix<float,3,Eigen::Dynamic> matAt(3,pointSelNum);
  Eigen::Matrix<float,3,3> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,3,1> matAtB;
  Eigen::Matrix<float,3,1> matX;
  Eigen::Matrix<float,3,3> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float arx =
        (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x +
        (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y +
        (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x +
                (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y +
                (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = arx;
    matA(i, 1) = arz;
    matA(i, 2) = aty;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1,3> matE;
    Eigen::Matrix<float,3,3> matV;
    Eigen::Matrix<float,3,3> matV2;
    
    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,3,3> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
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
    Eigen::Matrix<float,3,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[0] += matX(0, 0);
  transformCur[2] += matX(1, 0);
  transformCur[4] += matX(2, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2) +
                      pow(RAD2DEG * (matX(1, 0)), 2));
  float deltaT = sqrt(pow(matX(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformationCorner(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,3> matA(pointSelNum, 3);
  Eigen::Matrix<float,3,Eigen::Dynamic> matAt(3,pointSelNum);
  Eigen::Matrix<float,3,3> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,3,1> matAtB;
  Eigen::Matrix<float,3,1> matX;
  Eigen::Matrix<float,3,3> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c5 = crx * srz;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float ary =
        (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x +
        (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = ary;
    matA(i, 1) = atx;
    matA(i, 2) = atz;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1, 3> matE;
    Eigen::Matrix<float,3, 3> matV;
    Eigen::Matrix<float,3, 3> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,3,3> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
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
    Eigen::Matrix<float,3,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[1] += matX(0, 0);
  transformCur[3] += matX(1, 0);
  transformCur[5] += matX(2, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2));
  float deltaT = sqrt(pow(matX(1, 0) * 100, 2) +
                      pow(matX(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformation(int iterCount) {
  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
  Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
  Eigen::Matrix<float,6,6> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,6,1> matAtB;
  Eigen::Matrix<float,6,1> matX;
  Eigen::Matrix<float,6,6> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float arx =
        (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x +
        (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y +
        (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float ary =
        (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x +
        (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x +
                (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y +
                (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = arx;
    matA(i, 1) = ary;
    matA(i, 2) = arz;
    matA(i, 3) = atx;
    matA(i, 4) = aty;
    matA(i, 5) = atz;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1, 6> matE;
    Eigen::Matrix<float,6, 6> matV;
    Eigen::Matrix<float,6, 6> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6,6> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[6] = {10, 10, 10, 10, 10, 10};
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
    Eigen::Matrix<float,6,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[0] += matX(0, 0);
  transformCur[1] += matX(1, 0);
  transformCur[2] += matX(2, 0);
  transformCur[3] += matX(3, 0);
  transformCur[4] += matX(4, 0);
  transformCur[5] += matX(5, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2) +
                      pow(RAD2DEG * (matX(1, 0)), 2) +
                      pow(RAD2DEG * (matX(2, 0)), 2));
  float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                      pow(matX(4, 0) * 100, 2) +
                      pow(matX(5, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

void FeatureAssociation::checkSystemInitialization() {
  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast.setInputCloud(laserCloudSurfLast);

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  systemInitedLM = true;
}

void FeatureAssociation::updateTransformation() {
  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100) return;

  for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingSurfFeatures(iterCount1);

    if (laserCloudOri->points.size() < 10) continue;
    if (calculateTransformationSurf(iterCount1) == false) break;
  }

  for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingCornerFeatures(iterCount2);

    if (laserCloudOri->points.size() < 10) continue;
    if (calculateTransformationCorner(iterCount2) == false) break;
  }
}

void FeatureAssociation::integrateTransformation() {
  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(transformLaserOdometrySum[0], transformLaserOdometrySum[1], transformLaserOdometrySum[2],
                     -transformCur[0], -transformCur[1], -transformCur[2], rx,
                     ry, rz);

  float x1 = cos(rz) * (transformCur[3]) -
             sin(rz) * (transformCur[4]);
  float y1 = sin(rz) * (transformCur[3]) +
             cos(rz) * (transformCur[4]);
  float z1 = transformCur[5];

  float x2 = x1;
  float y2 = cos(rx) * y1 - sin(rx) * z1;
  float z2 = sin(rx) * y1 + cos(rx) * z1;

  tx = transformLaserOdometrySum[3] - (cos(ry) * x2 + sin(ry) * z2);
  ty = transformLaserOdometrySum[4] - y2;
  tz = transformLaserOdometrySum[5] - (-sin(ry) * x2 + cos(ry) * z2);

  transformLaserOdometrySum[0] = rx;
  transformLaserOdometrySum[1] = ry;
  transformLaserOdometrySum[2] = rz;
  transformLaserOdometrySum[3] = tx;
  transformLaserOdometrySum[4] = ty;
  transformLaserOdometrySum[5] = tz;
}

void FeatureAssociation::adjustOutlierCloud() {
  PointType point;
  int cloudSize = outlierCloud->points.size();
  for (int i = 0; i < cloudSize; ++i) {
    point.x = outlierCloud->points[i].y;
    point.y = outlierCloud->points[i].z;
    point.z = outlierCloud->points[i].x;
    point.intensity = outlierCloud->points[i].intensity;
    outlierCloud->points[i] = point;
  }
}

void FeatureAssociation::assignMappingOdometry(float (&ts)[6]){
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion geoQuat;
    //-------------------------------------------
    quat_tf.setRPY(ts[2], -ts[0], -ts[1]);
    tf2::convert(quat_tf, geoQuat);
    mappingOdometry.header.stamp = cloudHeader.stamp;
    mappingOdometry.pose.pose.orientation.x = -geoQuat.y;
    mappingOdometry.pose.pose.orientation.y = -geoQuat.z;
    mappingOdometry.pose.pose.orientation.z = geoQuat.x;
    mappingOdometry.pose.pose.orientation.w = geoQuat.w;
    mappingOdometry.pose.pose.position.x = ts[3];
    mappingOdometry.pose.pose.position.y = ts[4];
    mappingOdometry.pose.pose.position.z = ts[5];
    //pubLaserOdometry->publish(mappingOdometry);
}

void FeatureAssociation::publishOdometryPath() {

  tf2::Quaternion quat_tf;
  geometry_msgs::msg::Quaternion geoQuat;

  //-------------------------------------------
  quat_tf.setRPY(transformLaserOdometrySum[2], -transformLaserOdometrySum[0], -transformLaserOdometrySum[1]);
  tf2::convert(quat_tf, geoQuat);
  
  geometry_msgs::msg::PoseStamped tmp_pose;
  tmp_pose.header.stamp = cloudHeader.stamp;
  tmp_pose.pose.orientation.x = -geoQuat.y;
  tmp_pose.pose.orientation.y = -geoQuat.z;
  tmp_pose.pose.orientation.z = geoQuat.x;
  tmp_pose.pose.orientation.w = geoQuat.w;
  tmp_pose.pose.position.x = transformLaserOdometrySum[3];
  tmp_pose.pose.position.y = transformLaserOdometrySum[4];
  tmp_pose.pose.position.z = transformLaserOdometrySum[5];
  laser_odom_path_.poses.push_back(tmp_pose);
  pubOriginizedLaserOdometryPath->publish(laser_odom_path_);

  //-------------------------------------------
  quat_tf.setRPY(transformExternalOdometrySum[2], -transformExternalOdometrySum[0], -transformExternalOdometrySum[1]);
  tf2::convert(quat_tf, geoQuat);  

  geometry_msgs::msg::PoseStamped tmp_pose2;
  tmp_pose2.header.stamp = cloudHeader.stamp;
  tmp_pose2.pose.orientation.x = -geoQuat.y;
  tmp_pose2.pose.orientation.y = -geoQuat.z;
  tmp_pose2.pose.orientation.z = geoQuat.x;
  tmp_pose2.pose.orientation.w = geoQuat.w;
  tmp_pose2.pose.position.x = transformExternalOdometrySum[3];
  tmp_pose2.pose.position.y = transformExternalOdometrySum[4];
  tmp_pose2.pose.position.z = transformExternalOdometrySum[5];
  wheel_odom_path_.poses.push_back(tmp_pose2);
  pubOriginizedWheelOdometryPath->publish(wheel_odom_path_);
}

void FeatureAssociation::publishCloud() {
  sensor_msgs::msg::PointCloud2 laserCloudOutMsg;

  auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                     const pcl::PointCloud<PointType>::Ptr &cloud) {
    
    pcl::PointCloud<PointType>::Ptr tmp_pub_cloud;
    tmp_pub_cloud.reset(new pcl::PointCloud<PointType>());
    Eigen::Affine3d trans_c2s_af3d = tf2::transformToEigen(trans_c2s_);
    pcl::transformPointCloud(*cloud, *tmp_pub_cloud, trans_c2s_af3d.inverse());
    
    pcl::toROSMsg(*tmp_pub_cloud, laserCloudOutMsg);
    laserCloudOutMsg.header = cloudHeader;
    pub->publish(laserCloudOutMsg);
    
  };

  Publish(pubCornerPointsSharp, cornerPointsSharp);
  Publish(pubCornerPointsLessSharp, cornerPointsLessSharp);
  Publish(pubSurfPointsFlat, surfPointsFlat);
  Publish(pubSurfPointsLessFlat, surfPointsLessFlat);
}

void FeatureAssociation::publishCloudsLast() {

  int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
  for (int i = 0; i < cornerPointsLessSharpNum; i++) {
    TransformToEnd(&cornerPointsLessSharp->points[i],
                   &cornerPointsLessSharp->points[i]);
  }

  int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
  for (int i = 0; i < surfPointsLessFlatNum; i++) {
    TransformToEnd(&surfPointsLessFlat->points[i],
                   &surfPointsLessFlat->points[i]);
  }

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
    kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast.setInputCloud(laserCloudSurfLast);
  }

  frameCount++;
  adjustOutlierCloud();

  if (frameCount >= skipFrameNum + 1) {
    frameCount = 0;
    sensor_msgs::msg::PointCloud2 cloudTemp;

    auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                       const pcl::PointCloud<PointType>::Ptr &cloud) {

      pcl::PointCloud<PointType>::Ptr tmp_pub_cloud;
      tmp_pub_cloud.reset(new pcl::PointCloud<PointType>());
      Eigen::Affine3d trans_c2s_af3d = tf2::transformToEigen(trans_c2s_);
      pcl::transformPointCloud(*cloud, *tmp_pub_cloud, trans_c2s_af3d.inverse());
      pcl::toROSMsg(*tmp_pub_cloud, cloudTemp);
      cloudTemp.header = cloudHeader;
      pub->publish(cloudTemp);
      
    };

    Publish(_pub_outlier_cloudLast, outlierCloud);
    Publish(_pub_cloud_corner_last, laserCloudCornerLast);
    Publish(_pub_cloud_surf_last, laserCloudSurfLast);
  }
}

void FeatureAssociation::runFeatureAssociation() {

  ProjectionOut projection;
  _input_channel.receive(projection);

  //--------------
  std::lock_guard<std::mutex> lock(_odom_mutex);

  outlierCloud = projection.outlier_cloud;
  segmentedCloud = projection.segmented_cloud;
  segInfo = std::move(projection.seg_msg);

  cloudHeader = segInfo.header;
  cloudHeader.stamp = segInfo.header.stamp;
  trans_c2s_ = projection.trans_c2s;
  tf2_trans_b2s_.setOrigin(tf2::Vector3(projection.trans_b2s.transform.translation.x, 
                              projection.trans_b2s.transform.translation.y, projection.trans_b2s.transform.translation.z));
  tf2_trans_b2s_.setRotation(tf2::Quaternion(projection.trans_b2s.transform.rotation.x, 
                    projection.trans_b2s.transform.rotation.y, projection.trans_b2s.transform.rotation.z, projection.trans_b2s.transform.rotation.w));
  odom_type_ = projection.odom_type;

  if(!initialize_laser_odom_at_first_frame_){

    _vertical_scans = projection.vertical_scans;
    _horizontal_scans = projection.horizontal_scans;
    _scan_period = projection.scan_period;  
    initializeValue();

    //@ inital default value
    exteralOdometry.pose.pose.orientation.w = 1.0;
    exteralOdometry.header.frame_id = "odom";
    exteralOdometry.child_frame_id = baselink_frame_;

    initialize_laser_odom_at_first_frame_ = true;
    return;
  }
  adjustDistortion();

  calculateSmoothness();

  markOccludedPoints();

  extractFeatures();

  publishCloud();  // cloud for visualization

  // Feature Association
  if (!systemInitedLM) {
    checkSystemInitialization();
    return;
  }

  if(odom_type_=="laser_odometry"){
    updateTransformation();
    integrateTransformation();
    exteralOdometry.header.stamp = cloudHeader.stamp;
    assignMappingOdometry(transformLaserOdometrySum);
  }
  else{
    assignMappingOdometry(transformExternalOdometrySum);
  }
  
  if(odom_type_!="laser_odometry"){
    if(!first_odom_prepared_){
      RCLCPP_WARN(this->get_logger(), "External odometry is asked, but the topic is not received yet.");
      return;
    }
  }
  
  //publishOdometryPath();

  
  if (to_map_optimization_) {

    publishCloudsLast();  // cloud to mapOptimization
    
    AssociationOut out;
    out.cloud_corner_last.reset(new pcl::PointCloud<PointType>());
    out.cloud_surf_last.reset(new pcl::PointCloud<PointType>());
    out.cloud_outlier_last.reset(new pcl::PointCloud<PointType>());

    *out.cloud_corner_last = *laserCloudCornerLast;
    *out.cloud_surf_last = *laserCloudSurfLast;
    *out.cloud_outlier_last = *outlierCloud;

    out.cloud_patched_ground_last.reset(new pcl::PointCloud<PointType>());
    out.cloud_patched_ground_edge_last.reset(new pcl::PointCloud<PointType>());

    out.cloud_patched_ground_last = projection.patched_ground;
    out.cloud_patched_ground_edge_last = projection.patched_ground_edge;

    out.decisive_odometry = mappingOdometry;
    out.trans_c2s = trans_c2s_;
    out.trans_b2s = projection.trans_b2s;
    out.trans_m2ci = projection.trans_m2ci;
    out.external_odometry = exteralOdometry;
    _output_channel.send(std::move(out));
  }
  
}
