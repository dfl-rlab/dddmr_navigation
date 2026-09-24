/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <perception_3d/speed_limit_layer.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::SpeedLimitLayer, perception_3d::Sensor)

namespace perception_3d
{

SpeedLimitLayer::SpeedLimitLayer(){
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

SpeedLimitLayer::~SpeedLimitLayer(){

}

void SpeedLimitLayer::onInitialize()
{ 

  rclcpp::QoS map_qos(10);  // initialize to default
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);
  zone_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("speed_limit_zones", map_qos);
  speed_pc_zone_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("speed_limit_pc_zones", map_qos);

  cbs_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  /*
  pcl_map_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
    std::bind(&SpeedLimitLayer::cbMap, this, std::placeholders::_1), sub_options);

  pcl_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "mapground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
    std::bind(&SpeedLimitLayer::cbGround, this, std::placeholders::_1), sub_options);
  */

  node_->declare_parameter(name_ + "." + "speed_zone_pcd_file_dir", rclcpp::ParameterValue(""));
  node_->get_parameter(name_ + "." + "speed_zone_pcd_file_dir", speed_zone_pcd_file_dir_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "speed_zone_pcd_file_dir: %s" , speed_zone_pcd_file_dir_.c_str());
  
  parseYAML(speed_zone_pcd_file_dir_);
  
  knn_num_of_ground_search_ = 1;
  node_->declare_parameter(name_ + "." + "knn_num_of_ground_search", rclcpp::ParameterValue(1));
  node_->get_parameter(name_ + "." + "knn_num_of_ground_search", knn_num_of_ground_search_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "knn_num_of_ground_search: %d" , knn_num_of_ground_search_);
  
  current_zone_ = "";

}

void SpeedLimitLayer::parseYAML(std::string path){

  //@Start zone yaml stuff
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  rcl_params_t * params_hdl = rcl_yaml_node_struct_init(allocator);

  char* cpath = const_cast<char*>(path.c_str());

  if(rcl_parse_yaml_file(cpath, params_hdl)){
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Found: %s", path.c_str());
  }
  
  rclcpp::ParameterMap PM = rclcpp::parameter_map_from(params_hdl);
  if(PM.find("/speed_limit_layer") == PM.end()){
    RCLCPP_INFO(node_->get_logger().get_child(name_), "/speed_limit_layer not found, disable the layer.");
    return;
  }
  else{
  }

  auto zones = PM.at("/speed_limit_layer");

  bool zone_read = false;
  bool speed_read = false;
  std::string zone_name = "";
  std::string zone_dir = "";
  double unit_zone_speed = 0.0;

  //@ Following loop:
  // zone1.pcd
  // zone1.speed
  // zone2.pcd
  // zone2.speed
  for(auto i=zones.begin();i!=zones.end();i++){
    
    if(strstr((*i).get_name().c_str(), std::string(".pcd").c_str())){
      std::string unit_zone_string = (*i).value_to_string();
      zone_read = true;
      //split by /
      std::stringstream unit_zone_string_stream(unit_zone_string);
      std::vector<std::string> unit_zone_string_list;
      std::string unit_split;
      while(std::getline(unit_zone_string_stream, unit_split, '/'))
      {
        unit_zone_string_list.push_back(unit_split);
      }
      zone_name = (*i).get_name(); //zone1.pcd
      zone_name += " : " + unit_zone_string_list.back();
      zone_dir = unit_zone_string;
    }
    else if(strstr((*i).get_name().c_str(), std::string(".speed").c_str())){
      unit_zone_speed = (*i).as_double();
      speed_read = true;
    }

    if(zone_read && speed_read){
      RCLCPP_INFO(node_->get_logger().get_child(name_), "Read speed limit zone: %s with speed: %.2f", zone_name.c_str(), unit_zone_speed);
      perception_3d::SpeedZone a_zone(name_, node_->get_node_logging_interface(), zone_name, zone_dir, unit_zone_speed);
      getBoundingBox(a_zone);
      speed_zones_.insert({zone_name, a_zone});
      zone_marker_array_.markers.push_back(a_zone.bb_marker_);
      zone_intensity_pc_ += (*a_zone.zone_intensity_pc_);
      //reset
      zone_read = false;
      speed_read = false;
      zone_name = "";
      zone_dir = "";
    }
    
  }
  zone_pub_->publish(zone_marker_array_);

  sensor_msgs::msg::PointCloud2 ros_pc2_msg;
  pcl::toROSMsg(zone_intensity_pc_, ros_pc2_msg);
  ros_pc2_msg.header.frame_id = gbl_utils_->getRobotFrame();
  speed_pc_zone_pub_->publish(ros_pc2_msg);

}

void SpeedLimitLayer::getBoundingBox(perception_3d::SpeedZone& a_zone){
  
  // Generate an oriented bounding box around the selected points in RVIZ
  // Compute principal direction
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*a_zone.zone_pc_, centroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*a_zone.zone_pc_, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  // Move the points to the reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ> cPoints;
  pcl::transformPointCloud(*a_zone.zone_pc_, cPoints, p2w);

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cPoints, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
  // Final transform and bounding box size
  const Eigen::Quaternionf qfinal(eigDx);
  const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
  double bb_size_x = max_pt.x - min_pt.x;
  double bb_size_y = max_pt.y - min_pt.y;
  double bb_size_z = max_pt.z - min_pt.z;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  a_zone.bb_marker_.header.frame_id = gbl_utils_->getGblFrame();
  a_zone.bb_marker_.ns = a_zone.zone_name_;
  a_zone.bb_marker_.id = 0;
  a_zone.bb_marker_.type = visualization_msgs::msg::Marker::CUBE;
  a_zone.bb_marker_.action = visualization_msgs::msg::Marker::ADD;
  a_zone.bb_marker_.pose.position.x = tfinal.x();
  a_zone.bb_marker_.pose.position.y = tfinal.y();
  a_zone.bb_marker_.pose.position.z = tfinal.z();
  a_zone.bb_marker_.pose.orientation.x = qfinal.x();
  a_zone.bb_marker_.pose.orientation.y = qfinal.y();
  a_zone.bb_marker_.pose.orientation.z = qfinal.z();
  a_zone.bb_marker_.pose.orientation.w = qfinal.w();
  a_zone.bb_marker_.scale.x = bb_size_x;
  a_zone.bb_marker_.scale.y = bb_size_y;
  a_zone.bb_marker_.scale.z = bb_size_z;
  a_zone.bb_marker_.color.r = 1.0f;
  a_zone.bb_marker_.color.g = 1.0f;
  a_zone.bb_marker_.color.b = 0.0f;
  a_zone.bb_marker_.color.a = 0.5;


}

void SpeedLimitLayer::updateLethalPointCloud(){
  
}

void SpeedLimitLayer::selfMark(){
  
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  
  if(!shared_data_->is_static_layer_ready_){
    return;
  }

  geometry_msgs::msg::TransformStamped trans_g2b;
  try
  {
    trans_g2b = gbl_utils_->tf2Buffer()->lookupTransform(
        gbl_utils_->getGblFrame(), gbl_utils_->getRobotFrame(), tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Failed to get transform: %s", e.what());
    return;
  }
  
  //@ First we find knn points from ground and then we loop zones to check them, we use knn to double check existence, because of sparse ground situation
  //@ Find knn of the robot in ground and check are all points in zones
  pcl::PointXYZI robot_pt;
  robot_pt.x = trans_g2b.transform.translation.x;
  robot_pt.y = trans_g2b.transform.translation.y;
  robot_pt.z = trans_g2b.transform.translation.z;
  std::vector<int> pointIdxKNNSearch(knn_num_of_ground_search_);
  std::vector<float> pointKNNSquaredDistance(knn_num_of_ground_search_);
  if(shared_data_->kdtree_ground_->nearestKSearch(robot_pt, knn_num_of_ground_search_, pointIdxKNNSearch, pointKNNSquaredDistance)){

  }
  else{
    shared_data_->current_allowed_max_linear_speed_ = -1.0;
    RCLCPP_ERROR(node_->get_logger().get_child(name_), "No ground has been found, disable speed limit.");
  }

  //@Loop zones here
  bool in_zones = false;
  for(auto it=speed_zones_.begin(); it!=speed_zones_.end(); it++){

    int matched_number = 0;
    for(auto knn_it = pointIdxKNNSearch.begin(); knn_it!=pointIdxKNNSearch.end(); knn_it++){
      
      pcl::PointXYZ knn_pt;
      knn_pt.x = shared_data_->pcl_ground_->points[(*knn_it)].x;
      knn_pt.y = shared_data_->pcl_ground_->points[(*knn_it)].y;
      knn_pt.z = shared_data_->pcl_ground_->points[(*knn_it)].z;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if((*it).second.kdtree_zone_->radiusSearch(knn_pt, 0.05, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
        matched_number++;
      }

    }
    if(matched_number == knn_num_of_ground_search_){
      if(current_zone_!=(*it).first){
        current_zone_ = (*it).first;
        RCLCPP_INFO(node_->get_logger().get_child(name_), "Enter Zone: %s", current_zone_.c_str());
        shared_data_->current_allowed_max_linear_speed_ = (*it).second.speed_limit_;
      }
      in_zones = true;
      break;
    }

  }
  if(!in_zones){
    if(current_zone_!=""){
      current_zone_ = "";
      RCLCPP_INFO(node_->get_logger().get_child(name_), "Exit Zones");
      shared_data_->current_allowed_max_linear_speed_ = -1.0;
    }
  }

}

void SpeedLimitLayer::selfClear(){
  shared_data_->dgraph_update_request_[name_] = false;
}

void SpeedLimitLayer::resetdGraph(){}

double SpeedLimitLayer::get_dGraphValue(const unsigned int index){
  //@ No dGraph value in static layer, return 100 km
  return 99999.9;
}

bool SpeedLimitLayer::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SpeedLimitLayer::getObservation(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr SpeedLimitLayer::getObservationRobotFrame(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr SpeedLimitLayer::getLethal(){
  return current_lethal_;
}
}//end of name space
