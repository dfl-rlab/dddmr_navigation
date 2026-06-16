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

#include <boost/circular_buffer.hpp>
#include "imageProjection.h"

using std::placeholders::_1;

ImageProjection::ImageProjection(std::string name, Channel<ProjectionOut>& output_channel)
    : Node(name), _output_channel(output_channel), first_frame_processed_(0), got_baselink2sensor_tf_(false)
{

  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  clock_ = this->get_clock();
  last_save_depth_img_time_ = 0;
  sensor_install_pitch_ = 0.0;
  is_trt_engine_exist_ = false;
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  _pub_projected_image = this->create_publisher<sensor_msgs::msg::Image>("projected_image", 1);

  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lslidar_point_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), 
        std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

  _pub_full_info_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("full_cloud_info", 1);  

  _pub_ground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("ground_cloud", 1);  

  _pub_segmented_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud", 1); 

  _pub_segmented_cloud_pure = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("segmented_cloud_pure", 1);

  _pub_segmented_cloud_info = this->create_publisher<cloud_msgs::msg::CloudInfo>
      ("segmented_cloud_info", 1); 

  _pub_outlier_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>
      ("outlier_cloud", 1); 
  
  pub_annotated_img_ = this->create_publisher<sensor_msgs::msg::Image>("annotated_image", 1);
  
  declare_parameter("laser.num_vertical_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_vertical_scans", _vertical_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_vertical_scans: %d", _vertical_scans);
  
  declare_parameter("laser.num_horizontal_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_horizontal_scans", _horizontal_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_horizontal_scans: %d", _horizontal_scans);
  
  declare_parameter("laser.scan_period", rclcpp::ParameterValue(0.1));
  this->get_parameter("laser.scan_period", _scan_period);
  RCLCPP_INFO(this->get_logger(), "laser.scan_period: %.2f", _scan_period);
  
  declare_parameter("laser.vertical_angle_bottom", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_bottom", _ang_bottom);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_bottom: %.2f", _ang_bottom);

  float vertical_angle_top;
  declare_parameter("laser.vertical_angle_top", rclcpp::ParameterValue(0.0));
  this->get_parameter("laser.vertical_angle_top", vertical_angle_top);
  RCLCPP_INFO(this->get_logger(), "laser.vertical_angle_top: %.2f", vertical_angle_top);

  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;
  
  declare_parameter("imageProjection.segment_theta", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.segment_theta", _segment_theta);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_theta: %.2f", _segment_theta);
  _segment_theta *= DEG_TO_RAD;
  
  declare_parameter("imageProjection.segment_valid_point_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.segment_valid_point_num", _segment_valid_point_num);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_valid_point_num: %d", _segment_valid_point_num);
  
  declare_parameter("imageProjection.segment_valid_line_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.segment_valid_line_num",_segment_valid_line_num);
  RCLCPP_INFO(this->get_logger(), "imageProjection.segment_valid_line_num: %d", _segment_valid_line_num);

  declare_parameter("laser.odom_type", rclcpp::ParameterValue(""));
  this->get_parameter("laser.odom_type", odom_type_);
  RCLCPP_INFO(this->get_logger(), "laser.odom_type: %s", odom_type_.c_str());
  
  declare_parameter("laser.base_ground_frame", rclcpp::ParameterValue("base_link"));
  this->get_parameter("laser.base_ground_frame", base_ground_frame_);
  RCLCPP_INFO(this->get_logger(), "laser.base_ground_frame: %s", base_ground_frame_.c_str());
  
  declare_parameter("imageProjection.maximum_detection_range", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.maximum_detection_range", _maximum_detection_range);
  RCLCPP_INFO(this->get_logger(), "imageProjection.maximum_detection_range: %.2f", _maximum_detection_range);

  declare_parameter("imageProjection.minimum_detection_range", rclcpp::ParameterValue(0.3));
  this->get_parameter("imageProjection.minimum_detection_range", _minimum_detection_range);
  RCLCPP_INFO(this->get_logger(), "imageProjection.minimum_detection_range: %.2f", _minimum_detection_range);

  declare_parameter("imageProjection.distance_for_patch_between_rings", rclcpp::ParameterValue(1.0));
  this->get_parameter("imageProjection.distance_for_patch_between_rings", distance_for_patch_between_rings_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.distance_for_patch_between_rings: %.2f", distance_for_patch_between_rings_);

  declare_parameter("imageProjection.to_fa", rclcpp::ParameterValue(true));
  this->get_parameter("imageProjection.to_fa", to_fa_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.to_fa: %d", to_fa_);
  
  declare_parameter("imageProjection.time_step_between_depth_image", rclcpp::ParameterValue(0.5));
  this->get_parameter("imageProjection.time_step_between_depth_image", time_step_between_depth_image_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.time_step_between_depth_image: %.2f", time_step_between_depth_image_);

  declare_parameter("imageProjection.stitcher_num", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.stitcher_num", stitcher_num_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.stitcher_num: %d", stitcher_num_);
  stitcher_num_ = std::max(stitcher_num_, 1);

  declare_parameter("imageProjection.ground_fov_bottom", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ground_fov_bottom", ground_fov_bottom_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_fov_bottom: %.6f", ground_fov_bottom_);

  declare_parameter("imageProjection.ground_fov_top", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ground_fov_top", ground_fov_top_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_fov_top: %.6f", ground_fov_top_);

  declare_parameter("imageProjection.ground_positive_start", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ground_positive_start", ground_positive_start_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_positive_start: %.6f", ground_positive_start_);

  declare_parameter("imageProjection.ground_positive_stop", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ground_positive_stop", ground_positive_stop_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_positive_stop: %.6f", ground_positive_stop_);

  declare_parameter("imageProjection.ground_negative_start", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ground_negative_start", ground_negative_start_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_negative_start: %.6f", ground_negative_start_);

  declare_parameter("imageProjection.ground_negative_stop", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ground_negative_stop", ground_negative_stop_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_negative_stop: %.6f", ground_negative_stop_);
  
  //@ ignore this region
  declare_parameter("imageProjection.ignore_fov_bottom", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ignore_fov_bottom", ignore_fov_bottom_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ignore_fov_bottom: %.6f", ignore_fov_bottom_);

  declare_parameter("imageProjection.ignore_fov_top", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ignore_fov_top", ignore_fov_top_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ignore_fov_top: %.6f", ignore_fov_top_);

  declare_parameter("imageProjection.ignore_positive_start", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ignore_positive_start", ignore_positive_start_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ignore_positive_start: %.6f", ignore_positive_start_);

  declare_parameter("imageProjection.ignore_positive_stop", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ignore_positive_stop", ignore_positive_stop_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ignore_positive_stop: %.6f", ignore_positive_stop_);

  declare_parameter("imageProjection.ignore_negative_start", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ignore_negative_start", ignore_negative_start_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ignore_negative_start: %.6f", ignore_negative_start_);

  declare_parameter("imageProjection.ignore_negative_stop", rclcpp::ParameterValue(0.0));
  this->get_parameter("imageProjection.ignore_negative_stop", ignore_negative_stop_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ignore_negative_stop: %.6f", ignore_negative_stop_);

  declare_parameter("imageProjection.ground_slope_tolerance", rclcpp::ParameterValue(0.174533));
  this->get_parameter("imageProjection.ground_slope_tolerance", ground_slope_tolerance_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_slope_tolerance: %.6f", ground_slope_tolerance_);

  declare_parameter("imageProjection.ground_dz_tolerance", rclcpp::ParameterValue(0.1));
  this->get_parameter("imageProjection.ground_dz_tolerance", ground_dz_tolerance_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.ground_dz_tolerance: %.6f", ground_dz_tolerance_);

  declare_parameter("imageProjection.use_sensor_height_to_filter_out_ground", rclcpp::ParameterValue(false));
  this->get_parameter("imageProjection.use_sensor_height_to_filter_out_ground", use_sensor_height_to_filter_out_ground_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.use_sensor_height_to_filter_out_ground: %d", use_sensor_height_to_filter_out_ground_);

  declare_parameter("imageProjection.patch_first_ring_to_baselink", rclcpp::ParameterValue(true));
  this->get_parameter("imageProjection.patch_first_ring_to_baselink", patch_first_ring_to_baselink_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.patch_first_ring_to_baselink: %d", patch_first_ring_to_baselink_);

  this->declare_parameter("imageProjection.trt_model_path", rclcpp::ParameterValue(""));
  this->get_parameter("imageProjection.trt_model_path", trt_model_path_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.trt_model_path: %s" , trt_model_path_.c_str());

  this->declare_parameter("imageProjection.projected_image_stack_size", rclcpp::ParameterValue(0));
  this->get_parameter("imageProjection.projected_image_stack_size", projected_image_stack_size_);
  RCLCPP_INFO(this->get_logger(), "imageProjection.projected_image_stack_size: %d" , projected_image_stack_size_);
  if(projected_image_stack_size_<1){
    projected_image_stack_size_ = 1;
  }
  std::string filename = "my_file.txt"; // Replace with your file name

  if (std::filesystem::exists(trt_model_path_)) {
    is_trt_engine_exist_ = true;
  }

#ifdef TRT_ENABLED
  if(is_trt_engine_exist_){
    YoloV8Config config;
    yolov8_ = std::make_shared<YoloV8>("", trt_model_path_, config);
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "\033[1;31mtensorRT is specified but %s does not exist\033[0m", trt_model_path_.c_str());
  }
#endif

  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _z_pitch_roll_decisive_feature_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());
  patched_ground_.reset(new pcl::PointCloud<PointType>());
  patched_ground_edge_.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);
  
  dsf_patched_ground_.setLeafSize(0.1, 0.1, 0.1);
  
  dsf_patched_ground_omp_.setNumberOfThreads(6);
  dsf_patched_ground_omp_.setLeafSize (0.1, 0.1, 0.1);
  dsf_patched_ground_omp_.setSaveLeafLayout(false);

  dsf_patched_ground_edge_omp_.setNumberOfThreads(6);
  dsf_patched_ground_edge_omp_.setLeafSize (0.1, 0.1, 0.1);
  dsf_patched_ground_edge_omp_.setSaveLeafLayout(false);

  yolo_labelled_point_cloud_.reset(new pcl::PointCloud<PointType>());
}


void ImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  _seg_msg.start_ring_index.assign(_vertical_scans, 0);
  _seg_msg.end_ring_index.assign(_vertical_scans, 0);

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size, 0);
}

void ImageProjection::tfInitial(){

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

bool ImageProjection::allEssentialTFReady(std::string sensor_frame){
  
  if(!got_baselink2sensor_tf_){
    sensor_frame_ = sensor_frame;
    try
    {
      trans_b2s_ = tf2Buffer_->lookupTransform(
          base_ground_frame_, sensor_frame_, tf2::TimePointZero);
      
      tf2_trans_b2s_.setRotation(tf2::Quaternion(trans_b2s_.transform.rotation.x, trans_b2s_.transform.rotation.y, trans_b2s_.transform.rotation.z, trans_b2s_.transform.rotation.w));
      tf2_trans_b2s_.setOrigin(tf2::Vector3(trans_b2s_.transform.translation.x, trans_b2s_.transform.translation.y, trans_b2s_.transform.translation.z));
      
      //@ Get sensor pitch and transform them to correct orientation
      //@ Generate map2camera_init. This will be used to export final point cloud/pose graph based on map frame
      //@ "0 0 0 pi/2 0 pi/2 map -> camera_init" and "0 0 0 -pi/2 -pi/2 0 camera -> base_link"
      //@affine_3d for pcl conversion, tf2::stamped for key_pose
      
      trans_m2ci_.header.frame_id = "map";
      trans_m2ci_.child_frame_id = "camera_init";
      tf2::Quaternion qm2ci;
      tf2::Matrix3x3 m(tf2_trans_b2s_.getRotation());
      double sensor_install_roll, sensor_install_yaw;
      m.getRPY(sensor_install_roll, sensor_install_pitch_, sensor_install_yaw);
      qm2ci.setRPY(1.570795 + sensor_install_pitch_, 0.0, 1.570795);
      trans_m2ci_.transform.translation.x = 0.0; trans_m2ci_.transform.translation.y = 0.0; trans_m2ci_.transform.translation.z = 0.0;
      trans_m2ci_.transform.rotation.x = qm2ci.x(); trans_m2ci_.transform.rotation.y = qm2ci.y();
      trans_m2ci_.transform.rotation.z = qm2ci.z(); trans_m2ci_.transform.rotation.w = qm2ci.w();
      
      tf_static_broadcaster_->sendTransform(trans_m2ci_);
      
      // camera to sensor
      tf2::Quaternion qc2s;
      qc2s.setRPY(0, -1.570795, -1.570795);
      tf2_trans_c2s_.setOrigin(tf2::Vector3(0, 0, 0));
      tf2_trans_c2s_.setRotation(qc2s);
      
      got_baselink2sensor_tf_= true;

      trans_c2s_.transform.translation.x = tf2_trans_c2s_.getOrigin().x(); 
      trans_c2s_.transform.translation.y = tf2_trans_c2s_.getOrigin().y(); 
      trans_c2s_.transform.translation.z = tf2_trans_c2s_.getOrigin().z();
      trans_c2s_.transform.rotation.x = tf2_trans_c2s_.getRotation().x();
      trans_c2s_.transform.rotation.y = tf2_trans_c2s_.getRotation().y();
      trans_c2s_.transform.rotation.z = tf2_trans_c2s_.getRotation().z();
      trans_c2s_.transform.rotation.w = tf2_trans_c2s_.getRotation().w();

      return true;
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not get footprint frame to sensor frame, did you launch a static broadcaster node for the tf between footprint to sensor?");
      return false;
    }
  }
  else{
    return true;
  }
}

void ImageProjection::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){

  if(!allEssentialTFReady(laserCloudMsg->header.frame_id))
    return;

  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  _laser_cloud_in->is_dense = false;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  

  //@if not stitch, save copy time
  pcl::PointCloud<PointType>::Ptr pcl_stitched_msg (new pcl::PointCloud<PointType>);
  if(stitcher_num_<=1){
  }
  else{
    if(pcl_stitcher_.size()<stitcher_num_){
      pcl_stitcher_.push_back(*_laser_cloud_in);
    }
    else{
      pcl_stitcher_.pop_front();
      pcl_stitcher_.push_back(*_laser_cloud_in);
    }
    
    for(auto si=pcl_stitcher_.begin(); si!=pcl_stitcher_.end();si++){
      *pcl_stitched_msg += (*si);
    }
    *_laser_cloud_in = *pcl_stitched_msg;
  }

  pc_valid_ = true;
  if(_laser_cloud_in->points.size()<_vertical_scans*_horizontal_scans*0.1*(stitcher_num_)){
    RCLCPP_ERROR(this->get_logger(), "Expecting: %d points, but you only got %lu, check your lidar scan.", _vertical_scans*_horizontal_scans, _laser_cloud_in->points.size());
    pc_valid_ = false;
    return;
  }

  _seg_msg.header = laserCloudMsg->header;
  _seg_msg.header.stamp = laserCloudMsg->header.stamp;
  _seg_msg.header.frame_id = laserCloudMsg->header.frame_id;

  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  zPitchRollFeatureRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}


void ImageProjection::projectPointCloud() {
  
  //cv image
  range_mat_removing_moving_object_ = cv::Mat::zeros(_vertical_scans, _horizontal_scans, CV_8UC3); 
  cv::Mat projected_image(_vertical_scans, _horizontal_scans, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat projected_stack_result;

  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  //if(_vertical_scans*_horizontal_scans!=_laser_cloud_in->points.size())
  //  RCLCPP_ERROR(this->get_logger(), "Number of lidar points %lu does not match the %d*%d", _laser_cloud_in->points.size(), _vertical_scans, _horizontal_scans);

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(thisPoint.z / range);

    int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
    if (rowIdn < 0 || rowIdn >= _vertical_scans) {
      continue;
    }

    float horizonAngle = std::atan2(thisPoint.y, thisPoint.x);

    int viscolumnIdn = -round((horizonAngle) / _ang_resolution_X) + _horizontal_scans * 0.5;

    if(viscolumnIdn>=_horizontal_scans)
      viscolumnIdn = viscolumnIdn - _horizontal_scans;

    if (viscolumnIdn < 0 || viscolumnIdn >= _horizontal_scans){
      continue;
    }

    if (range < _minimum_detection_range || range > _maximum_detection_range){
      continue;
    }

    _range_mat(rowIdn, viscolumnIdn) = range;
    
    //@ generate projected_image
    //@ the rowIdn for _range_mat is from top to bottom, which means the line 0 is the first row
    //@ to visualize the depth image more intuitively, we make line 0 to the last rowIdn

    if(rowIdn>0 && rowIdn<=_vertical_scans && viscolumnIdn<_horizontal_scans && viscolumnIdn>=0){

      uint16_t scaled_int = static_cast<uint16_t>(range*100);
      // 2. Split the 16-bit integer into high and low bytes
      uint8_t high_byte = (uint8_t)(scaled_int >> 8);
      uint8_t low_byte = (uint8_t)(scaled_int & 0xFF);
      if(high_byte>0){
        high_byte*=4;
        projected_image.at<cv::Vec3b>(_vertical_scans-rowIdn, viscolumnIdn)[0] = 0XFF;
        projected_image.at<cv::Vec3b>(_vertical_scans-rowIdn, viscolumnIdn)[1] = 0XFF-low_byte;
        projected_image.at<cv::Vec3b>(_vertical_scans-rowIdn, viscolumnIdn)[2] = 0xFF-high_byte*8;
      }
      else{
        projected_image.at<cv::Vec3b>(_vertical_scans-rowIdn, viscolumnIdn)[0] = high_byte;
        projected_image.at<cv::Vec3b>(_vertical_scans-rowIdn, viscolumnIdn)[1] = low_byte;
        projected_image.at<cv::Vec3b>(_vertical_scans-rowIdn, viscolumnIdn)[2] = 0;
      }

      
    }

    thisPoint.intensity = (float)rowIdn + (float)viscolumnIdn / 10000.0;
    size_t index = viscolumnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }
  
  //@ queue projected image
  if(projected_image_queue_.size()<projected_image_stack_size_){
    projected_image_queue_.push_back(projected_image);
  }
  else{
    projected_image_queue_.push_back(projected_image);
    projected_image_queue_.pop_front();
  }

  //@ stack all projected image to get time elapse
  projected_stack_result = projected_image_queue_.front();
  for(auto it=projected_image_queue_.begin()+1; it!=projected_image_queue_.end(); it++){
    cv::vconcat(projected_stack_result, (*it), projected_stack_result);
  }


#ifdef TRT_ENABLED
  
  if(to_fa_ && is_trt_engine_exist_ && projected_image_queue_.size()==projected_image_stack_size_){ //to_fa means we are not exporting depth image, so skip inference to save time 
  cv::Mat inferenced_image = projected_stack_result;
  // Run inference
  const auto objects = yolov8_->detectObjects(inferenced_image);

  // Draw the bounding boxes on the image
  yolov8_->drawObjectLabels(inferenced_image, objects);

  // Remove object from projected_image
  cv::Mat full_sized_mask = cv::Mat::zeros(inferenced_image.size(), CV_8UC1);
  for (const auto &object : objects) {

    //@ label=0 is people, remove it
    if(object.label==0){
      cv::Mat roi_mask = full_sized_mask(object.rect);
      object.boxMask.copyTo(roi_mask);
    }
    //cv::Size imageSize = object.boxMask.size();
    //RCLCPP_INFO(this->get_logger(), "object class: %d, confidence: %.2f", object.label, object.probability);
  }
  cv::Mat inverted_mask;
  cv::bitwise_not(full_sized_mask, inverted_mask);
  cv::bitwise_and(inferenced_image, inferenced_image, range_mat_removing_moving_object_, inverted_mask);
  
  
  //@ cut final inferenced image
  int x_start = 0;
  int y_start = inferenced_image.rows - inferenced_image.rows/projected_image_stack_size_;
  int crop_width = inferenced_image.cols;
  int crop_height = inferenced_image.rows/projected_image_stack_size_;
  cv::Rect myROI(x_start, y_start, crop_width, crop_height);
  cv::Mat last_row_of_projected_image = inferenced_image(myROI);//@inferenced image is sizeof(projected_image_stack_size_ by 1)
  range_mat_removing_moving_object_ = range_mat_removing_moving_object_(myROI);
  
  cv_bridge::CvImage img_annotated;
  img_annotated.image = last_row_of_projected_image;
  img_annotated.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  sensor_msgs::msg::Image::SharedPtr ros2_annotated_img = img_annotated.toImageMsg();
  pub_annotated_img_->publish(*ros2_annotated_img);
  }
#endif

  cv_bridge::CvImage img_bridge;
  img_bridge.image = projected_stack_result;
  img_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  sensor_msgs::msg::Image::SharedPtr msg = img_bridge.toImageMsg();
  _pub_projected_image->publish(*msg);

  //@ write depth img
  double image_time = _seg_msg.header.stamp.sec + _seg_msg.header.stamp.nanosec/1e9;

  if(!to_fa_ && image_time - last_save_depth_img_time_ >= time_step_between_depth_image_){
    std::string timestamp;
    std::stringstream ss;
    ss << _seg_msg.header.stamp.sec << "_" << std::setw(9) << std::setfill('0') << _seg_msg.header.stamp.nanosec;
    timestamp = ss.str();
    std::string spec = 
      std::to_string(_vertical_scans)+"x"+std::to_string(_horizontal_scans) \
      +"_stitch"+std::to_string(stitcher_num_) \
      +"_vstack"+std::to_string(projected_image_stack_size_);
    std::string file_name = mapping_dir_string_ + "/" + timestamp + "_" + spec + ".png";
    cv::imwrite(file_name, projected_stack_result);
    last_save_depth_img_time_ = image_time;
  }

}

void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.start_orientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.end_orientation - _seg_msg.start_orientation > 3 * M_PI) {
    _seg_msg.end_orientation -= 2 * M_PI;
  } else if (_seg_msg.end_orientation - _seg_msg.start_orientation < M_PI) {
    _seg_msg.end_orientation += 2 * M_PI;
  }
  _seg_msg.orientation_diff =
      _seg_msg.end_orientation - _seg_msg.start_orientation;

}

void ImageProjection::getNoPitchPoint(PointType& pt_in, PointType& pt_out){
  //@ remove pitch to robot frame
  //_full_cloud->points[lowerInd]
  geometry_msgs::msg::TransformStamped trans_lidar2horizontal;
  tf2::Quaternion q;
  q.setRPY( 0, sensor_install_pitch_, 0);
  trans_lidar2horizontal.transform.rotation.x = q.x(); trans_lidar2horizontal.transform.rotation.y = q.y();
  trans_lidar2horizontal.transform.rotation.z = q.z(); trans_lidar2horizontal.transform.rotation.w = q.w();
  Eigen::Affine3d trans_lidar2horizontal_af3 = tf2::transformToEigen(trans_lidar2horizontal);

  pcl::PointCloud<PointType> temp_pc;
  temp_pc.push_back(pt_in);
  pcl::transformPointCloud(temp_pc, temp_pc, trans_lidar2horizontal_af3);
  pt_out = temp_pc.points[0];
}

void ImageProjection::zPitchRollFeatureRemoval() {
  
  yolo_labelled_point_cloud_.reset(new pcl::PointCloud<PointType>());

  geometry_msgs::msg::TransformStamped trans_lidar2horizontal;
  tf2::Quaternion q;
  q.setRPY( 0, sensor_install_pitch_, 0);
  trans_lidar2horizontal.transform.rotation.x = q.x(); trans_lidar2horizontal.transform.rotation.y = q.y();
  trans_lidar2horizontal.transform.rotation.z = q.z(); trans_lidar2horizontal.transform.rotation.w = q.w();
  Eigen::Affine3d trans_lidar2horizontal_af3 = tf2::transformToEigen(trans_lidar2horizontal);

  pcl::PointCloud<PointType> _full_cloud_no_pitch;
  pcl::transformPointCloud(*_full_cloud, _full_cloud_no_pitch, trans_lidar2horizontal_af3);

  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  _z_pitch_roll_decisive_feature_cloud->clear();
  patched_ground_->points.clear();
  patched_ground_edge_->points.clear();

  for (size_t j = 0; j < _horizontal_scans; ++j) {
    size_t ring_edge = 0;
    size_t closest_ring_edge = _vertical_scans-1;
    bool do_patch = false;
    for (size_t i = 0; i < _vertical_scans-1; ++i) {

      if(_range_mat(i,j)==FLT_MAX){
        _label_mat(i, j) = -1;
      }

      size_t lowerInd = j + (i)*_horizontal_scans;
      size_t upperInd = j + (i + 1) * _horizontal_scans;

      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      //@ if the point should be ignore, i.e. lidar see the robot body itself
      
      double current_i_angle = i * _ang_resolution_Y - _ang_bottom; //_ang_bottom has beedn changed sign at begining
      double current_iplus1_angle = (i+1) * _ang_resolution_Y - _ang_bottom;
      double current_j_angle = -1.0*((j - _horizontal_scans * 0.5) * _ang_resolution_X);

      if(current_i_angle>=ignore_fov_bottom_ && current_i_angle<=ignore_fov_top_ &&
         current_iplus1_angle>=ignore_fov_bottom_ && current_iplus1_angle<=ignore_fov_top_){

        if(current_j_angle>=0){
          if(current_j_angle>=ignore_positive_start_ && current_j_angle<=ignore_positive_stop_){
            _ground_mat(i, j) = -1;
            continue;
          }
        }
        else{
          if(current_j_angle<=ignore_negative_start_ && current_j_angle>=ignore_negative_stop_){
            _ground_mat(i, j) = -1;
            continue;
          } 
        }
      }

      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY));

      // zPitchRoll feature
      if ( vertical_angle <= 5 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
        _z_pitch_roll_decisive_feature_cloud->push_back(_full_cloud->points[upperInd]);
        _z_pitch_roll_decisive_feature_cloud->push_back(_full_cloud->points[lowerInd]);
      
        //We have found zPitchRoll features, label something we dont need for xYYawfeatures
        _label_mat(i, j) = -1;
        _label_mat(i+1, j) = -1;
      }

      //@ 1. check upper and lower are in ground FOV
      //@ 2. check two point slope is less than some reasonable value

      bool in_ground_fov = false;

      if(current_i_angle>=ground_fov_bottom_ && current_i_angle<=ground_fov_top_ &&
         current_iplus1_angle>=ground_fov_bottom_ && current_iplus1_angle<=ground_fov_top_){

        if(current_j_angle>=0){
          if(current_j_angle>=ground_positive_start_ && current_j_angle<=ground_positive_stop_){
            in_ground_fov = true;
          }
        }
        else{
          if(current_j_angle<=ground_negative_start_ && current_j_angle>=ground_negative_stop_){
            in_ground_fov = true;
          } 
        }
      }

#ifdef TRT_ENABLED
      // TRT object detection to remove things we dont want to use for SLAM, such as moving object
      if(is_trt_engine_exist_ && projected_image_queue_.size()==projected_image_stack_size_){
        // it was generated as _range_mat(rowIdn, viscolumnIdn) = range;
        cv::Vec3b pixel = range_mat_removing_moving_object_.at<cv::Vec3b>(_vertical_scans-i, j);
        if(pixel[0]==0 && pixel[1]==0 && pixel[2]==0){
          _label_mat(i, j) = -1;
          _ground_mat(i, j) = -1;
          _label_mat(i+1, j) = -1;
          yolo_labelled_point_cloud_->push_back(_full_cloud->points[lowerInd]);
          in_ground_fov = false;
        }
      }
#endif

      if(in_ground_fov){
        PointType lowerInd_pt_no_pitch, upperInd_pt_no_pitch;
        PointType lowerInd_left_pt_no_pitch;
        PointType lowerInd_right_pt_no_pitch;
        bool valid_point = false;
        size_t horizontal_search_number = 20;
        if(j<horizontal_search_number){
          lowerInd_pt_no_pitch = _full_cloud_no_pitch[lowerInd];
          for(int jj=1;jj<horizontal_search_number;jj++){
            size_t compensateInd = 0;
            if(lowerInd<jj){
              compensateInd = _horizontal_scans+lowerInd-jj;
            }
            else{
              compensateInd = lowerInd-jj;
            }
            lowerInd_left_pt_no_pitch = _full_cloud_no_pitch[compensateInd];
            if(fabs(lowerInd_pt_no_pitch.y - lowerInd_left_pt_no_pitch.y)>0.05){
              valid_point = true;
              break;
            }
              
          }
          for(int jj=1;jj<horizontal_search_number;jj++){
            lowerInd_right_pt_no_pitch = _full_cloud_no_pitch[lowerInd+jj];
            if(fabs(lowerInd_pt_no_pitch.y - lowerInd_right_pt_no_pitch.y)>0.05){
              valid_point = true;
              break;
            }
              
          }
        }

        else if(j>_horizontal_scans-horizontal_search_number-1){
          lowerInd_pt_no_pitch = _full_cloud_no_pitch[lowerInd];
          for(int jj=1;jj<horizontal_search_number;jj++){
            lowerInd_left_pt_no_pitch = _full_cloud_no_pitch[lowerInd-jj];
            if(fabs(lowerInd_pt_no_pitch.y - lowerInd_left_pt_no_pitch.y)>0.05){
              valid_point = true;
              break;
            }
          }
          for(int jj=1;jj<horizontal_search_number;jj++){
            size_t compensateInd = 0;
            if(lowerInd+jj>_horizontal_scans-1){
              compensateInd = (lowerInd+jj) - _horizontal_scans;
            }
            else{
              compensateInd = lowerInd+jj;
            }
            lowerInd_right_pt_no_pitch = _full_cloud_no_pitch[compensateInd];
            if(fabs(lowerInd_pt_no_pitch.y - lowerInd_right_pt_no_pitch.y)>0.05){
              valid_point = true;
              break;
            }
          }
        }
        else{
          lowerInd_pt_no_pitch = _full_cloud_no_pitch[lowerInd];
          for(int jj=1;jj<horizontal_search_number;jj++){
            lowerInd_left_pt_no_pitch = _full_cloud_no_pitch[lowerInd-jj];
            if(fabs(lowerInd_pt_no_pitch.y - lowerInd_left_pt_no_pitch.y)>0.05){
              valid_point = true;
              break;
            }
          }
          for(int jj=1;jj<horizontal_search_number;jj++){
            lowerInd_right_pt_no_pitch = _full_cloud_no_pitch[lowerInd+jj];
            if(fabs(lowerInd_pt_no_pitch.y - lowerInd_right_pt_no_pitch.y)>0.05){
              valid_point = true;
              break;
            } 
          }
        }
        
        double dz_left = fabs(lowerInd_pt_no_pitch.z - lowerInd_left_pt_no_pitch.z);
        double dz_right = fabs(lowerInd_pt_no_pitch.z - lowerInd_right_pt_no_pitch.z);
        double dz_left2right = fabs(lowerInd_left_pt_no_pitch.z - lowerInd_right_pt_no_pitch.z);

        if(!valid_point || dz_left>0.05 || dz_right>0.05 || dz_left2right>0.05){
          do_patch = false;
          //_segmented_cloud_pure->push_back(_full_cloud->points[lowerInd]);
          continue;
        }

        //@ filter by z from ground to sensor, this is only useful when in a flat environment.
        if(use_sensor_height_to_filter_out_ground_){
          if(trans_b2s_.transform.translation.z+lowerInd_left_pt_no_pitch.z>0.1 || 
            trans_b2s_.transform.translation.z+lowerInd_right_pt_no_pitch.z>0.1 || 
            trans_b2s_.transform.translation.z+lowerInd_pt_no_pitch.z>0.1 ){
            do_patch = false;
            continue;
          }
        }


        if(i<closest_ring_edge) //we dont casting the last one
          closest_ring_edge = i;

        lowerInd_pt_no_pitch = _full_cloud_no_pitch[lowerInd];
        upperInd_pt_no_pitch = _full_cloud_no_pitch[upperInd];

        PointType lowerInd_pt = _full_cloud->points[lowerInd];
        PointType upperInd_pt = _full_cloud->points[upperInd];

        float dXg = upperInd_pt_no_pitch.x - lowerInd_pt_no_pitch.x;
        float dYg = upperInd_pt_no_pitch.y - lowerInd_pt_no_pitch.y;
        float dZg = upperInd_pt_no_pitch.z - lowerInd_pt_no_pitch.z;

        float vertical_angle = std::atan2(dZg , sqrt(dXg * dXg + dYg * dYg));
        if ( fabs(vertical_angle) > ground_slope_tolerance_ && fabs(dZg)>ground_dz_tolerance_) {
          do_patch = false;
          _segmented_cloud_pure->push_back(_full_cloud->points[lowerInd]);
          continue;
        }
        float ds = sqrt(dX*dX + dY*dY + dZ*dZ);
        
        //@ if distance between is too large, we do not patch, because of too many unknown betweem rings
        if(ds<distance_for_patch_between_rings_){
          ring_edge = i+1;
          float dt = 1.0/(ds/0.1+1);
          for(float t=0; t<=1.0; t+=dt){
            PointType a_pt;
            a_pt.intensity = 0.0;
            a_pt.x = lowerInd_pt.x + dX*t;
            a_pt.y = lowerInd_pt.y + dY*t;
            a_pt.z = lowerInd_pt.z + dZ*t;
            patched_ground_->push_back(a_pt);
          }
          PointType a_pt;
          a_pt.intensity = 0.0;
          a_pt.x = lowerInd_pt.x + dX;
          a_pt.y = lowerInd_pt.y + dY;
          a_pt.z = lowerInd_pt.z + dZ;
          patched_ground_->push_back(a_pt);
          do_patch = true;
        }
      }
      else{
        if(i>0)
          _segmented_cloud_pure->push_back(_full_cloud->points[lowerInd]);
      }
    }
    
    //@ we have ring edge, mark intensity for those edge points
    size_t ringEdgeInd = j + (ring_edge)*_horizontal_scans;
    PointType a_pt = _full_cloud->points[ringEdgeInd];
    a_pt.intensity = 1000;
    patched_ground_edge_->push_back(a_pt);

    if(patch_first_ring_to_baselink_ && do_patch && first_frame_processed_<5 && closest_ring_edge < _vertical_scans-1){
      //@ patch ground from closest ring edge to base_link
      size_t closest_ring_edgeInd = j + (closest_ring_edge)*_horizontal_scans;
      PointType a_pt = _full_cloud->points[closest_ring_edgeInd];
      float dXf = -a_pt.x;
      float dYf = -a_pt.y;
      float dZf = -a_pt.z;

      for(float t=0; t<=1.0; t+=0.05){
        PointType a_ptf;
        a_ptf.intensity = 0.0;
        a_ptf.x = a_pt.x + dXf*t;
        a_ptf.y = a_pt.y + dYf*t;
        a_ptf.z = a_pt.z; // mitigate height difference
        patched_ground_->push_back(a_ptf);
      }
      PointType a_ptf;
      a_ptf.intensity = 0.0;
      a_ptf.x = a_pt.x + dXf;
      a_ptf.y = a_pt.y + dYf;
      a_ptf.z = a_pt.z;
      patched_ground_->push_back(a_ptf);
    }
  }

  patched_ground_->is_dense = false;
  patched_ground_edge_->is_dense = false;
  std::vector<int> tmp_indices, tmp_indices2;
  pcl::removeNaNFromPointCloud(*patched_ground_, *patched_ground_, tmp_indices);
  pcl::removeNaNFromPointCloud(*patched_ground_edge_, *patched_ground_edge_, tmp_indices2);
  
  //dsf_patched_ground_.setInputCloud(patched_ground_);
  //dsf_patched_ground_.filter(*patched_ground_);
  dsf_patched_ground_omp_.setInputCloud(patched_ground_);
  dsf_patched_ground_omp_.setFinalFilter(true);
  dsf_patched_ground_omp_.filter(*patched_ground_);

  //dsf_patched_ground_.setInputCloud(patched_ground_edge_);
  //dsf_patched_ground_.filter(*patched_ground_edge_); 
  dsf_patched_ground_edge_omp_.setInputCloud(patched_ground_edge_);
  dsf_patched_ground_edge_omp_.setFinalFilter(true);
  dsf_patched_ground_edge_omp_.filter(*patched_ground_edge_); 
}

void ImageProjection::cloudSegmentation() {
  // segmentation process
  for (size_t i = 0; i < _vertical_scans; ++i)
    for (size_t j = 0; j < _horizontal_scans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < _vertical_scans; ++i) {
    _seg_msg.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (j % 5 == 0) {
            _outlier_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
          }
          continue;
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        // remove nan

        if(!pcl::isFinite(_full_cloud->points[j + i * _horizontal_scans]))
          continue;
        // mark ground points so they will not be considered as edge features
        // later
        _seg_msg.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmented_cloud_range[sizeOfSegCloud] = _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    _seg_msg.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  /*
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
  */
}

void ImageProjection::labelComponents(int row, int col) {

  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _vertical_scans){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (all_pushed.size() >= _segment_valid_point_num) {
    int lineCount = 0;
    for (size_t i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {
  const auto& cloudHeader = _seg_msg.header;

  sensor_msgs::msg::PointCloud2 laserCloudTemp;

  auto PublishCloud = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (true) {
      pcl::toROSMsg(*cloud, laserCloudTemp);
      laserCloudTemp.header = cloudHeader;
      laserCloudTemp.header.stamp = _seg_msg.header.stamp;
      pub->publish(laserCloudTemp);
    }
  };

  //PublishCloud(_pub_outlier_cloud, _outlier_cloud);
  //PublishCloud(_pub_segmented_cloud, _segmented_cloud);
  PublishCloud(_pub_ground_cloud, patched_ground_);
  PublishCloud(_pub_segmented_cloud_pure, _segmented_cloud_pure);
  //PublishCloud(_pub_full_info_cloud, _full_info_cloud);
  if (_pub_segmented_cloud_info->get_subscription_count() != 0) {
    _pub_segmented_cloud_info->publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());
  out.patched_ground.reset(new pcl::PointCloud<PointType>());
  out.patched_ground_edge.reset(new pcl::PointCloud<PointType>());
  out.trans_c2s = trans_c2s_;
  out.trans_b2s = trans_b2s_;
  out.trans_m2ci = trans_m2ci_;
  out.odom_type = odom_type_;
  out.vertical_scans = _vertical_scans;
  out.horizontal_scans = _horizontal_scans;
  out.scan_period = _scan_period;
  
  std::swap(out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);
  std::swap(out.patched_ground, patched_ground_);
  std::swap(out.patched_ground_edge, patched_ground_edge_);
  if(to_fa_)
    _output_channel.send( std::move(out) );
  first_frame_processed_++;

}


