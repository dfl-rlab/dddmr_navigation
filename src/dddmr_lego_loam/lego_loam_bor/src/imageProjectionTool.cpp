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
#include "imageProjectionTool.h"

using std::placeholders::_1;

ImageProjectionTool::ImageProjectionTool(std::string name, Channel<ProjectionOut>& output_channel)
    : Node(name), _output_channel(output_channel), first_frame_processed_(0), got_baselink2sensor_tf_(false)
{

  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  clock_ = this->get_clock();
  yaw_correction_ = 0.0;
  pitch_correction_ = 0.0;
  last_save_depth_img_time_ = 0;
  sensor_install_pitch_ = 0.0;
  is_trt_engine_exist_ = false;
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  _pub_projected_image = this->create_publisher<sensor_msgs::msg::Image>("projected_image", 1);
  _pub_projected_selected_image = this->create_publisher<sensor_msgs::msg::Image>("projected_selected_image", 1);

  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lslidar_point_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), 
        std::bind(&ImageProjectionTool::cloudHandler, this, std::placeholders::_1));
  
  pub_annotated_img_ = this->create_publisher<sensor_msgs::msg::Image>("annotated_image", 1);
  
  declare_parameter("laser.num_vertical_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_vertical_scans", _vertical_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_vertical_scans: %d", _vertical_scans);
  
  declare_parameter("laser.num_horizontal_scans", rclcpp::ParameterValue(0));
  this->get_parameter("laser.num_horizontal_scans", _horizontal_scans);
  RCLCPP_INFO(this->get_logger(), "laser.num_horizontal_scans: %d", _horizontal_scans);
  
  //@get this assign as earliest
  cloud_size_ = _vertical_scans * _horizontal_scans;

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
  for(int i=0; i<projected_image_stack_size_; i++){
    std::string topic_name = "projected_point_cloud_" + std::to_string(i);
    point_cloud_publishers_vector_.push_back(
      this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()));
  }

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _z_pitch_roll_decisive_feature_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());
  patched_ground_.reset(new pcl::PointCloud<PointType>());
  patched_ground_edge_.reset(new pcl::PointCloud<PointType>());
  ds_patched_ground_.reset(new pcl::PointCloud<PointType>());
  ds_patched_ground_edge_.reset(new pcl::PointCloud<PointType>());
  _full_cloud->points.resize(cloud_size_);
  _full_info_cloud->points.resize(cloud_size_);
  
  dsf_patched_ground_.setLeafSize(0.1, 0.1, 0.1);

  yolo_labelled_point_cloud_.reset(new pcl::PointCloud<PointType>());
  selected_cloud_.reset(new pcl::PointCloud<PointType>());
}


void ImageProjectionTool::resetParameters() {

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

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size_, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size_, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size_, 0);
}


void ImageProjectionTool::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){

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
  
  if(fabs(yaw_correction_)>0){
    geometry_msgs::msg::TransformStamped trans_lidar2horizontal;
    tf2::Quaternion q;
    q.setRPY( 0, 0, yaw_correction_);
    trans_lidar2horizontal.transform.rotation.x = q.x(); trans_lidar2horizontal.transform.rotation.y = q.y();
    trans_lidar2horizontal.transform.rotation.z = q.z(); trans_lidar2horizontal.transform.rotation.w = q.w();
    Eigen::Affine3d trans_lidar2horizontal_af3 = tf2::transformToEigen(trans_lidar2horizontal);
    pcl::transformPointCloud(*_laser_cloud_in, *_laser_cloud_in, trans_lidar2horizontal_af3);
  }

  projectPointCloud();

}


void ImageProjectionTool::projectPointCloud() {
  
  //cv image
  range_mat_removing_moving_object_ = cv::Mat::zeros(_vertical_scans, _horizontal_scans, CV_8UC3); 
  cv::Mat projected_image(_vertical_scans, _horizontal_scans, CV_8UC3, cv::Scalar(0,0,0));

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

    if (rowIdn >= 0 && rowIdn < _vertical_scans && viscolumnIdn >= 0 && viscolumnIdn < _horizontal_scans) {
      float range_span = 20.0f;
      float norm = (range - _minimum_detection_range) / range_span;
      norm = std::max(0.0f, std::min(1.0f, norm));

      // Continuous 4-segment BGR gradient: Blue -> Cyan -> Green -> Yellow -> Red
      uint8_t b = 0, g = 0, r = 0;
      if (norm < 0.25f) {
        float f = norm / 0.25f;
        b = 255;
        g = static_cast<uint8_t>(f * 255.0f);
        r = 0;
      } else if (norm < 0.5f) {
        float f = (norm - 0.25f) / 0.25f;
        b = static_cast<uint8_t>((1.0f - f) * 255.0f);
        g = 255;
        r = 0;
      } else if (norm < 0.75f) {
        float f = (norm - 0.5f) / 0.25f;
        b = 0;
        g = 255;
        r = static_cast<uint8_t>(f * 255.0f);
      } else {
        float f = (norm - 0.75f) / 0.25f;
        b = 0;
        g = static_cast<uint8_t>((1.0f - f) * 255.0f);
        r = 255;
      }

      int img_row = _vertical_scans - 1 - rowIdn;
      projected_image.at<cv::Vec3b>(img_row, viscolumnIdn) = cv::Vec3b(b, g, r);
    }

    thisPoint.intensity = (float)rowIdn + (float)viscolumnIdn / 10000.0;
    size_t index = viscolumnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }
  
  //@ queue projected image and point cloud
  if(projected_image_queue_.size()<projected_image_stack_size_){
    projected_image_queue_.push_back(projected_image);
    projected_pc_queue_.push_back(*_full_info_cloud);
  }
  else{
    projected_image_queue_.push_back(projected_image);
    projected_image_queue_.pop_front();
    projected_pc_queue_.push_back(*_full_info_cloud);
    projected_pc_queue_.pop_front();
  }

  //@ publish each point cloud frame in the stack
  for(size_t i=0; i<projected_pc_queue_.size(); i++){
    sensor_msgs::msg::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(projected_pc_queue_[i], laserCloudTemp);
    laserCloudTemp.header = _seg_msg.header;
    point_cloud_publishers_vector_[i]->publish(laserCloudTemp);
  }

  //@ stack all projected image to get time elapse
  projected_stack_result_ = projected_image_queue_.front();
  for(auto it=projected_image_queue_.begin()+1; it!=projected_image_queue_.end(); it++){
    cv::vconcat(projected_stack_result_, (*it), projected_stack_result_);
  }

  cv_bridge::CvImage img_bridge;
  img_bridge.image = projected_stack_result_;
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
    cv::imwrite(file_name, projected_stack_result_);
    last_save_depth_img_time_ = image_time;
  }

}

void ImageProjectionTool::projectImageBySelection() {
  if (selected_cloud_->points.empty() || projected_pc_queue_.empty()) {
    return;
  }

  pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
  kdtree->setInputCloud(selected_cloud_);

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  std::deque<cv::Mat> selected_projected_image_queue;

  for (size_t q_idx = 0; q_idx < projected_pc_queue_.size(); ++q_idx) {
    cv::Mat projected_image(_vertical_scans, _horizontal_scans, CV_8UC3, cv::Scalar(0, 0, 0));
    auto& pc = projected_pc_queue_[q_idx];

    for (size_t i = 0; i < pc.points.size(); ++i) {
      const PointType& thisPoint = pc.points[i];
      if (!std::isfinite(thisPoint.x) || !std::isfinite(thisPoint.y) || !std::isfinite(thisPoint.z)) {
        continue;
      }

      if (kdtree->nearestKSearch(thisPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        if (pointNKNSquaredDistance[0] < 1e-4) {
          float range = sqrt(thisPoint.x * thisPoint.x +
                             thisPoint.y * thisPoint.y +
                             thisPoint.z * thisPoint.z);

          float verticalAngle = std::asin(thisPoint.z / range);
          int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
          if (rowIdn < 0 || rowIdn >= _vertical_scans) {
            continue;
          }

          float horizonAngle = std::atan2(thisPoint.y, thisPoint.x);
          int viscolumnIdn = -round((horizonAngle) / _ang_resolution_X) + _horizontal_scans * 0.5;
          if (viscolumnIdn >= _horizontal_scans) {
            viscolumnIdn = viscolumnIdn - _horizontal_scans;
          }
          if (viscolumnIdn < 0 || viscolumnIdn >= _horizontal_scans) {
            continue;
          }

          if (range < _minimum_detection_range || range > _maximum_detection_range) {
            continue;
          }

          float range_span = 20.0f;
          float norm = (range - _minimum_detection_range) / range_span;
          norm = std::max(0.0f, std::min(1.0f, norm));

          uint8_t b = 0, g = 0, r = 0;
          if (norm < 0.25f) {
            float f = norm / 0.25f;
            b = 255;
            g = static_cast<uint8_t>(f * 255.0f);
            r = 0;
          } else if (norm < 0.5f) {
            float f = (norm - 0.25f) / 0.25f;
            b = static_cast<uint8_t>((1.0f - f) * 255.0f);
            g = 255;
            r = 0;
          } else if (norm < 0.75f) {
            float f = (norm - 0.5f) / 0.25f;
            b = 0;
            g = 255;
            r = static_cast<uint8_t>(f * 255.0f);
          } else {
            float f = (norm - 0.75f) / 0.25f;
            b = 0;
            g = static_cast<uint8_t>((1.0f - f) * 255.0f);
            r = 255;
          }

          int img_row = _vertical_scans - 1 - rowIdn;
          projected_image.at<cv::Vec3b>(img_row, viscolumnIdn) = cv::Vec3b(b, g, r);
        }
      }
    }

    selected_projected_image_queue.push_back(projected_image);
  }

  projected_selected_stack_result_ = selected_projected_image_queue.front();
  for (auto it = selected_projected_image_queue.begin() + 1; it != selected_projected_image_queue.end(); ++it) {
    cv::vconcat(projected_selected_stack_result_, (*it), projected_selected_stack_result_);
  }

  cv_bridge::CvImage img_bridge;
  img_bridge.image = projected_selected_stack_result_;
  img_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  sensor_msgs::msg::Image::SharedPtr msg = img_bridge.toImageMsg();
  _pub_projected_selected_image->publish(*msg);
}

void ImageProjectionTool::exportProjectedImages() {
  std::stringstream ss;
  ss << _seg_msg.header.stamp.sec << "_" << std::setw(9) << std::setfill('0') << _seg_msg.header.stamp.nanosec;
  std::string timestamp = ss.str();
  std::string spec =
    std::to_string(_vertical_scans) + "x" + std::to_string(_horizontal_scans) +
    "_stitch" + std::to_string(stitcher_num_) +
    "_vstack" + std::to_string(projected_image_stack_size_);

  if (!projected_stack_result_.empty()) {
    std::string file_name = mapping_dir_string_ + "/" + timestamp + "_" + spec + ".png";
    cv::imwrite(file_name, projected_stack_result_);
    RCLCPP_INFO(this->get_logger(), "Saved projected image: %s", file_name.c_str());
  }

  if (!projected_selected_stack_result_.empty()) {
    std::string selected_file_name = mapping_dir_string_ + "/" + timestamp + "_" + spec + "_selected.png";
    cv::imwrite(selected_file_name, projected_selected_stack_result_);
    RCLCPP_INFO(this->get_logger(), "Saved selected projected image: %s", selected_file_name.c_str());
  }
}





