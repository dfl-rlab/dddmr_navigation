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

#include "pose_graph_editor.h"
#include <future>
#include <sstream>

using namespace gtsam;


PoseGraphEditor::PoseGraphEditor(std::string name, std::string pose_graph_dir) : 
  Node(name), pose_graph_dir_(pose_graph_dir), name_(name), current_focus_pg_("")
{
  
  map_.reset(new pcl::PointCloud<PointType>());
  node_1_pointcloud_.reset(new pcl::PointCloud<PointType>());
  node_2_pointcloud_.reset(new pcl::PointCloud<PointType>());

  //subscriber with string
  clock_ = this->get_clock();

  parameters_.relinearizeThreshold = 0.01;
  parameters_.relinearizeSkip = 1;
  isam_ = new ISAM2(parameters_);

  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  node_1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/node_1_pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  node_2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/node_2_pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  poses_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/poses", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  edges_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/edges", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  edge_selection_sub_ = this->create_subscription<std_msgs::msg::String>(
      "edge_selection_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphEditor::edgeSelectionCB, this, std::placeholders::_1));

  node_selection_sub_ = this->create_subscription<std_msgs::msg::String>(
      "node_selection_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphEditor::nodeSelectionCB, this, std::placeholders::_1));

  operation_command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphEditor::operationCommandCB, this, std::placeholders::_1));
  
  current_focus_pg_sub_ = this->create_subscription<std_msgs::msg::String>(
      "current_focus_pg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&PoseGraphEditor::currentFocusCB, this, std::placeholders::_1));
  
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  readPoseGraph();
}

void PoseGraphEditor::currentFocusCB(const std_msgs::msg::String::SharedPtr msg){
  current_focus_pg_ = msg->data;
}

gtsam::Vector PoseGraphEditor::getOdometryNoise(double m_value){
  gtsam::Vector Vector6(6);
  Vector6 << m_value, m_value, m_value, m_value, m_value, m_value;
  return Vector6;
}

void PoseGraphEditor::add2History(){
  std::pair<point_cloud_t, edge_t> operation(poses_, edges_);
  history_operation_.push_back(operation);
}

void PoseGraphEditor::popHistory(){
  poses_.clear();
  edges_.clear();
  auto a_tmp_pair = history_operation_.back();
  poses_ = a_tmp_pair.first;
  edges_ = a_tmp_pair.second;
  history_operation_.pop_back();
  generateISAMwiPosesEdges();
  RCLCPP_INFO(this->get_logger(),"Go to last step.");
}

void PoseGraphEditor::readPoseGraph(){
  /*
  pcd_edges_ is original data.
  pcd_poses_ is original data. It is map frame to base_link, each pcd is base_link frame
  We edit edges_ and poses_
  */
  pcd_edges_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pose_graph_dir_ + "/edges.pcd", *pcd_edges_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read edges PCD file fail: %s", pose_graph_dir_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Edges read: %lu", pcd_edges_->points.size());
  
  pcd_poses_.reset(new pcl::PointCloud<PointTypePose>());
  if (pcl::io::loadPCDFile<PointTypePose> (pose_graph_dir_ + "/poses.pcd", *pcd_poses_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read poses PCD file fail: %s", pose_graph_dir_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Poses read: %lu", pcd_poses_->points.size());
  
  pubTF();

  for(unsigned int it=0; it<pcd_poses_->points.size(); it++){
    //@ something like 0_feature.pcd
    std::string feature_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_feature.pcd";
    pcl::PointCloud<PointType>::Ptr a_feature_pcd(new pcl::PointCloud<PointType>());
    if (pcl::io::loadPCDFile<PointType> (feature_file_dir, *a_feature_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read feature PCD file fail: %s", feature_file_dir.c_str());
    }
    
    cornerCloudKeyFrames_baselink_.push_back(a_feature_pcd);

    std::string ground_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_ground.pcd";
    pcl::PointCloud<PointType>::Ptr a_ground_pcd(new pcl::PointCloud<PointType>());
    if (pcl::io::loadPCDFile<PointType> (ground_file_dir, *a_ground_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read ground PCD file fail: %s", ground_file_dir.c_str());
    }
    surfCloudKeyFrames_baselink_.push_back(a_ground_pcd);
  }

  //@ Generate isam to get poses/edges_ and isam
  generateISAM();
  selected_edges_.clear();
  pubGraph();
}

void PoseGraphEditor::generateISAM(){
  
  current_operation_nodes_.first = -1;
  current_operation_nodes_.second = -1;
  edges_.clear();
  poses_ = *pcd_poses_;

  if (isam_ != NULL)
  {
    delete isam_;
    isam_ = new ISAM2(parameters_);
  }
  //@Start to generate graph and isam
  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  priorNoise = noiseModel::Diagonal::Variances(Vector6);
  odometryNoise = noiseModel::Diagonal::Variances(getOdometryNoise(1e-2));
  
  std::set<int> poses_set; //@ if we already has pose in graph we can not provie initialEstimate, so we store them and check them
  
  RCLCPP_INFO(this->get_logger(), "Generate ISAM with edges: %lu", pcd_edges_->points.size());
  for(auto it=pcd_edges_->points.begin();it!=pcd_edges_->points.end();it++){
    
    if((*it).y==(*it).z){


      std::pair<int, int> tmp_pair((*it).y, (*it).y);
      if( !edges_.insert(tmp_pair).second )
      {   
        continue;
      }

      gtSAMgraph_.add(PriorFactor<Pose3>(
        (*it).y,
        Pose3(Rot3::RzRyRx(pcd_poses_->points[(*it).y].roll, pcd_poses_->points[(*it).y].pitch, pcd_poses_->points[(*it).y].yaw),
              Point3(pcd_poses_->points[(*it).y].x, pcd_poses_->points[(*it).y].y, pcd_poses_->points[(*it).y].z)),
        priorNoise));
      initialEstimate.insert(
        (*it).y, Pose3(Rot3::RzRyRx(pcd_poses_->points[(*it).y].roll, pcd_poses_->points[(*it).y].pitch, pcd_poses_->points[(*it).y].yaw),
                  Point3(pcd_poses_->points[(*it).y].x, pcd_poses_->points[(*it).y].y, pcd_poses_->points[(*it).y].z)));
      
    
      isam_->update(gtSAMgraph_, initialEstimate);
      isam_->update();
      
      gtSAMgraph_.resize(0);
      initialEstimate.clear();
      poses_set.insert((*it).y);

    }
    else{
      int index = (*it).x;
      int m_from = (*it).y;
      int m_to = (*it).z;

      std::pair<int, int> tmp_pair(m_from, m_to);
      if( !edges_.insert(tmp_pair).second )
      {   
        continue;
      }
      
      //@add edge
      gtsam::Pose3 poseFrom = Pose3(
          Rot3::RzRyRx(pcd_poses_->points[m_from].roll, pcd_poses_->points[m_from].pitch, pcd_poses_->points[m_from].yaw),
          Point3(pcd_poses_->points[m_from].x, pcd_poses_->points[m_from].y, pcd_poses_->points[m_from].z));
      gtsam::Pose3 poseTo = Pose3(
          Rot3::RzRyRx(pcd_poses_->points[m_to].roll, pcd_poses_->points[m_to].pitch, pcd_poses_->points[m_to].yaw),
          Point3(pcd_poses_->points[m_to].x, pcd_poses_->points[m_to].y, pcd_poses_->points[m_to].z));
      
      //RCLCPP_INFO(this->get_logger(), "%d, %d", m_from, m_to);
      gtSAMgraph_.add(BetweenFactor<Pose3>(
          m_from, m_to,
          poseFrom.between(poseTo), odometryNoise));

      initialEstimate.insert(
          m_to,
          Pose3(
            Rot3::RzRyRx(pcd_poses_->points[m_to].roll, pcd_poses_->points[m_to].pitch, pcd_poses_->points[m_to].yaw),
            Point3(pcd_poses_->points[m_to].x, pcd_poses_->points[m_to].y, pcd_poses_->points[m_to].z))
      );
      
      poses_set.insert(m_from);
      if(!poses_set.insert(m_to).second ){
        isam_->update(gtSAMgraph_);
      }
      else{
        isam_->update(gtSAMgraph_, initialEstimate);
      }
      
      isam_->update();
      
      gtSAMgraph_.resize(0);
      initialEstimate.clear();
    }


  }

  
  isamCurrentEstimate = isam_->calculateEstimate();
  for(auto it=poses_.points.begin(); it!=poses_.points.end(); it++){
    Pose3 est;
    est = isamCurrentEstimate.at<Pose3>((*it).intensity);
    (*it).x = est.translation().x();
    (*it).y = est.translation().y();
    (*it).z = est.translation().z();
    (*it).roll = est.rotation().roll();
    (*it).pitch = est.rotation().pitch();
    (*it).yaw = est.rotation().yaw();

  }
  
}

void PoseGraphEditor::generateISAMwiPosesEdges(){
  
  if (isam_ != NULL)
  {
    delete isam_;
    isam_ = new ISAM2(parameters_);
  }

  //@Start to generate graph and isam
  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  priorNoise = noiseModel::Diagonal::Variances(Vector6);
  odometryNoise = noiseModel::Diagonal::Variances(getOdometryNoise(1e-2));
  
  std::set<int> poses_set; //@ if we already has pose in graph we can not provie initialEstimate, so we store them and check them
  
  RCLCPP_INFO(this->get_logger(), "Generate ISAM with edges: %lu", edges_.size());
  for(auto it=edges_.begin();it!=edges_.end();it++){
    
    if((*it).first==(*it).second){

      gtSAMgraph_.add(PriorFactor<Pose3>(
      (*it).first,
      Pose3(Rot3::RzRyRx(poses_.points[(*it).first].roll, poses_.points[(*it).first].pitch, poses_.points[(*it).first].yaw),
            Point3(poses_.points[(*it).first].x, poses_.points[(*it).first].y, poses_.points[(*it).first].z)),
      priorNoise));

      initialEstimate.insert(
        (*it).first, Pose3(Rot3::RzRyRx(poses_.points[(*it).first].roll, poses_.points[(*it).first].pitch, poses_.points[(*it).first].yaw),
                  Point3(poses_.points[(*it).first].x, poses_.points[(*it).first].y, poses_.points[(*it).first].z)));
      
    
      isam_->update(gtSAMgraph_, initialEstimate);
      isam_->update();
      
      gtSAMgraph_.resize(0);
      initialEstimate.clear();
      poses_set.insert((*it).first);
    }
    else{
      int m_from = (*it).first;
      int m_to = (*it).second;
      RCLCPP_DEBUG(this->get_logger(), "%d-->%d", m_from, m_to);

      if((*it).first==0 && (*it).second==0)
        continue;
      
      //@add edge
      gtsam::Pose3 poseFrom = Pose3(
          Rot3::RzRyRx(poses_.points[m_from].roll, poses_.points[m_from].pitch, poses_.points[m_from].yaw),
          Point3(poses_.points[m_from].x, poses_.points[m_from].y, poses_.points[m_from].z));
      gtsam::Pose3 poseTo = Pose3(
          Rot3::RzRyRx(poses_.points[m_to].roll, poses_.points[m_to].pitch, poses_.points[m_to].yaw),
          Point3(poses_.points[m_to].x, poses_.points[m_to].y, poses_.points[m_to].z));
      
      //RCLCPP_INFO(this->get_logger(), "%d, %d", m_from, m_to);
      gtSAMgraph_.add(BetweenFactor<Pose3>(
          m_from, m_to,
          poseFrom.between(poseTo), odometryNoise));

      initialEstimate.insert(
          m_to,
          Pose3(
            Rot3::RzRyRx(poses_.points[m_to].roll, poses_.points[m_to].pitch, poses_.points[m_to].yaw),
            Point3(poses_.points[m_to].x, poses_.points[m_to].y, poses_.points[m_to].z))
      );
      
      poses_set.insert(m_from);
      if(!poses_set.insert(m_to).second ){
        isam_->update(gtSAMgraph_);
      }
      else{
        isam_->update(gtSAMgraph_, initialEstimate);
      }
      
      isam_->update();
      
      gtSAMgraph_.resize(0);
      initialEstimate.clear();
    }


  }

  
  isamCurrentEstimate = isam_->calculateEstimate();
  for(auto it=poses_.points.begin(); it!=poses_.points.end(); it++){
    Pose3 est;
    est = isamCurrentEstimate.at<Pose3>((*it).intensity);
    (*it).x = est.translation().x();
    (*it).y = est.translation().y();
    (*it).z = est.translation().z();
    (*it).roll = est.rotation().roll();
    (*it).pitch = est.rotation().pitch();
    (*it).yaw = est.rotation().yaw();

  }
  pubGraph();
}

void PoseGraphEditor::pubTF(){

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = clock_->now();
  t.header.frame_id = "map";
  t.child_frame_id = name_;

  t.transform.translation.x = 0;
  t.transform.translation.y = 0;
  t.transform.translation.z = 0;
  tf2::Quaternion q;
  q.setRPY(0,0,0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_static_broadcaster_->sendTransform(t);
}

void PoseGraphEditor::pubGraph(){

  //@hide edges by alpha=0
  visualization_msgs::msg::MarkerArray markerArray_edges;
  for(auto it=removed_edges_.begin();it!=removed_edges_.end();it++){
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = name_;
    markerEdge.header.stamp = clock_->now();
    markerEdge.action = visualization_msgs::msg::Marker::MODIFY;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.pose.orientation.w = 1.0;
    markerEdge.ns = name_ + "_edge_" + std::to_string((*it).first);
    markerEdge.id = 0;
    markerEdge.scale.x = 0.05;
    markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
    markerEdge.color.a = 0.0;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    p1.x = poses_.points[(*it).first].x;
    p1.y = poses_.points[(*it).first].y;
    p1.z = poses_.points[(*it).first].z; //make edge under node, make visualization easier
    p2.x = poses_.points[(*it).second].x;
    p2.y = poses_.points[(*it).second].y;
    p2.z = poses_.points[(*it).second].z;//make edge under node, make visualization easier
    markerEdge.points.push_back(p1);
    markerEdge.points.push_back(p2);
    markerEdge.id = (*it).second;
    markerArray_edges.markers.push_back(markerEdge);
  }

  //@mark selected edge as blue
  for(auto it=selected_edges_.begin();it!=selected_edges_.end();it++){

    const bool is_in = removed_edges_.find((*it)) != removed_edges_.end();
    if(is_in){
      RCLCPP_DEBUG(this->get_logger(), "%d-->%d", (*it).first, (*it).second);
      continue;
    }

    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = name_;
    markerEdge.header.stamp = clock_->now();
    markerEdge.action = visualization_msgs::msg::Marker::MODIFY;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.pose.orientation.w = 1.0;
    markerEdge.ns = name_ + "_edge_" + std::to_string((*it).first);
    markerEdge.id = 0;
    markerEdge.scale.x = 0.05;
    markerEdge.color.r = 0.0; markerEdge.color.g = 0.961; markerEdge.color.b = 1.0;
    markerEdge.color.a = 0.8;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    p1.x = poses_.points[(*it).first].x;
    p1.y = poses_.points[(*it).first].y;
    p1.z = poses_.points[(*it).first].z; //make edge under node, make visualization easier
    p2.x = poses_.points[(*it).second].x;
    p2.y = poses_.points[(*it).second].y;
    p2.z = poses_.points[(*it).second].z;//make edge under node, make visualization easier
    markerEdge.points.push_back(p1);
    markerEdge.points.push_back(p2);
    markerEdge.id = (*it).second;
    markerArray_edges.markers.push_back(markerEdge);
  }

  for(auto it=edges_.begin();it!=edges_.end();it++){
    
    const bool is_in = selected_edges_.find((*it)) != selected_edges_.end();
    if(is_in){
      RCLCPP_DEBUG(this->get_logger(), "%d-->%d", (*it).first, (*it).second);
      continue;
    }
      
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = name_;
    markerEdge.header.stamp = clock_->now();
    markerEdge.action = visualization_msgs::msg::Marker::MODIFY;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.pose.orientation.w = 1.0;
    markerEdge.ns = name_ + "_edge_" + std::to_string((*it).first);
    markerEdge.id = 0;
    markerEdge.scale.x = 0.05;
    markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
    markerEdge.color.a = 0.6;
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    p1.x = poses_.points[(*it).first].x;
    p1.y = poses_.points[(*it).first].y;
    p1.z = poses_.points[(*it).first].z; //make edge under node, make visualization easier
    p2.x = poses_.points[(*it).second].x;
    p2.y = poses_.points[(*it).second].y;
    p2.z = poses_.points[(*it).second].z;//make edge under node, make visualization easier
    markerEdge.points.push_back(p1);
    markerEdge.points.push_back(p2);
    markerEdge.id = (*it).second;
    markerArray_edges.markers.push_back(markerEdge);
  }
  edges_pub_->publish(markerArray_edges);

  //@visualize poses
  visualization_msgs::msg::MarkerArray markerArray_poses;
  int cnt = 0;
  for(auto it=poses_.points.begin(); it!=poses_.points.end(); it++){

    visualization_msgs::msg::Marker markerPoint;
    markerPoint.header.frame_id = name_;
    markerPoint.header.stamp = clock_->now();
    markerPoint.action = visualization_msgs::msg::Marker::MODIFY;
    markerPoint.type = visualization_msgs::msg::Marker::SPHERE;
    markerPoint.ns = name_ + "_node";
    markerPoint.id = 0;
    markerPoint.scale.x = 0.15; markerPoint.scale.y = 0.15; markerPoint.scale.z = 0.15;
    markerPoint.color.r = 0.9; markerPoint.color.g = 0.1; markerPoint.color.b = 0.1;
    markerPoint.color.a = 0.8; 

    tf2::Quaternion q;
    q.setRPY( (*it).roll, (*it).pitch, (*it).yaw);
    geometry_msgs::msg::Pose a_pose;
    a_pose.position.x = (*it).x;
    a_pose.position.y = (*it).y;
    a_pose.position.z = (*it).z;
    a_pose.orientation.x = q.x();
    a_pose.orientation.y = q.y();
    a_pose.orientation.z = q.z();
    a_pose.orientation.w = q.w();
    markerPoint.pose = a_pose;
    markerPoint.id = cnt;
    markerArray_poses.markers.push_back(markerPoint);
    cnt++;
  }
  poses_pub_->publish(markerArray_poses);
  
  //@----- update keyframe poses ---
  cornerCloudKeyFrames_.clear();
  map_.reset(new pcl::PointCloud<PointType>());
  ground_.reset(new pcl::PointCloud<PointType>());
  for(unsigned int it=0; it<poses_.points.size(); it++){
    //@ something like 0_feature.pcd

    pcl::PointCloud<PointType>::Ptr a_feature_pcd(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType> a_feature_pcd_baselink = *cornerCloudKeyFrames_baselink_[it];
    geometry_msgs::msg::TransformStamped trans_m2b;
    Eigen::Affine3d trans_m2b_af3;
    trans_m2b.transform.translation.x = poses_.points[it].x;
    trans_m2b.transform.translation.y = poses_.points[it].y;
    trans_m2b.transform.translation.z = poses_.points[it].z;
    tf2::Quaternion q;
    q.setRPY( poses_.points[it].roll, poses_.points[it].pitch, poses_.points[it].yaw);
    trans_m2b.transform.rotation.x = q.x(); trans_m2b.transform.rotation.y = q.y();
    trans_m2b.transform.rotation.z = q.z(); trans_m2b.transform.rotation.w = q.w();
    trans_m2b_af3 = tf2::transformToEigen(trans_m2b);
    //@transform to map frame
    pcl::transformPointCloud(a_feature_pcd_baselink, *a_feature_pcd, trans_m2b_af3);
    *map_ += (*a_feature_pcd);
    cornerCloudKeyFrames_.push_back(a_feature_pcd);

    pcl::PointCloud<PointType>::Ptr a_ground_pcd(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType> a_ground_pcd_baselink = *surfCloudKeyFrames_baselink_[it];
    pcl::transformPointCloud(a_ground_pcd_baselink, *a_ground_pcd, trans_m2b_af3);
    *ground_+= (*a_ground_pcd);
  }

  sensor_msgs::msg::PointCloud2 feature_cloudMsgTemp;
  pcl::toROSMsg(*map_, feature_cloudMsgTemp);
  feature_cloudMsgTemp.header.stamp = clock_->now();
  feature_cloudMsgTemp.header.frame_id = name_;
  map_pub_->publish(feature_cloudMsgTemp);

  sensor_msgs::msg::PointCloud2 ground_cloudMsgTemp;
  pcl::toROSMsg(*ground_, ground_cloudMsgTemp);
  ground_cloudMsgTemp.header.stamp = clock_->now();
  ground_cloudMsgTemp.header.frame_id = name_;
  ground_pub_->publish(ground_cloudMsgTemp);
}

void PoseGraphEditor::edgeSelectionCB(const std_msgs::msg::String::SharedPtr msg){
  //Parse edges msg and visualize them to make sure they are selected in this node and ready for the operation
  //@pg_0_edge_155/90;pg_0_edge_156/90;pg_0_edge_155/92
  if(msg->data == "d" || msg->data == "D"){
    if(selected_edges_.size()>0){
      //@ Some edges have been selected, we can delete them
      for(auto it=selected_edges_.begin(); it!=selected_edges_.end();it++){
        edges_.erase((*it));
        removed_edges_.insert((*it));
      }
      selected_edges_.clear();
      generateISAMwiPosesEdges();
      return;
    }
  }


  selected_edges_.clear();
  std::stringstream edge_full_name(msg->data);
  std::string one_edge_full_name;
  
  while(std::getline(edge_full_name, one_edge_full_name, ';'))
  {
    if(!strstr(name_.c_str(), one_edge_full_name.c_str()))
      continue;
    std::stringstream one_edge_full_name_stream(one_edge_full_name);
    std::string edge_number;
    std::vector<std::string> edge_number_list;
    while(std::getline(one_edge_full_name_stream, edge_number, '_'))
    {
      edge_number_list.push_back(edge_number);
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", edge_number_list[3].c_str()); //@ --> 155/90
    std::stringstream edge_num_stream(edge_number_list[3]);
    std::string m_number;
    std::vector<std::string> m_number_list;
    while(std::getline(edge_num_stream, m_number, '/'))
    {
      m_number_list.push_back(m_number);
    }
    int node_1 = stoi(m_number_list[0]);
    int node_2 = stoi(m_number_list[1]);
    std::pair<int, int> a_edge(node_1, node_2);
    RCLCPP_DEBUG(this->get_logger(), "%d-->%d", node_1, node_2); //@ --> 155/90
    selected_edges_.insert(a_edge);
  }
  add2History();
  pubGraph();

}

void PoseGraphEditor::nodeSelectionCB(const std_msgs::msg::String::SharedPtr msg){
  
  if(current_focus_pg_!=name_)
    return;
        
  //@ pg_0_node/88;pg_0_node/89;
  std::stringstream node_full_name(msg->data);
  std::string one_node_full_name;
  bool is_anchor = false;
  int selected_node = 0;
  while(std::getline(node_full_name, one_node_full_name, ';'))
  {

    if(one_node_full_name=="anchor"){
      is_anchor = true;
      continue;
    }

    if (one_node_full_name.find(name_) == std::string::npos) {
      continue;
    }

    std::stringstream one_node_full_name_stream(one_node_full_name);
    std::string node_number;
    std::vector<std::string> node_number_list;
    while(std::getline(one_node_full_name_stream, node_number, '/'))
    {
      node_number_list.push_back(node_number);
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", node_number_list[1].c_str());

    selected_node = stoi(node_number_list[1]);
    break;

  }
  
  if(is_anchor){
    current_operation_nodes_.first = selected_node;
  }
  else{
    current_operation_nodes_.second = selected_node;
    first_frame_2_second_frame_af3_ = Eigen::Affine3f::Identity();
  }

  if(current_operation_nodes_.first>=0){
    node_1_pointcloud_.reset(new pcl::PointCloud<PointType>());
    for(int i=current_operation_nodes_.first-3; i<current_operation_nodes_.first+4; i++){
      if(i<0 || i > cornerCloudKeyFrames_.size()-1)
        continue;
      *node_1_pointcloud_ += (*cornerCloudKeyFrames_[i]);
    }
    sensor_msgs::msg::PointCloud2 cloud1MsgTemp;
    pcl::toROSMsg(*node_1_pointcloud_, cloud1MsgTemp);
    cloud1MsgTemp.header.stamp = clock_->now();
    cloud1MsgTemp.header.frame_id = name_;
    node_1_pub_->publish(cloud1MsgTemp);
  }

  if(current_operation_nodes_.second>=0){
    node_2_pointcloud_.reset(new pcl::PointCloud<PointType>());
    node_2_pointcloud_ = cornerCloudKeyFrames_[current_operation_nodes_.second];
    sensor_msgs::msg::PointCloud2 cloud2MsgTemp;
    pcl::toROSMsg(*node_2_pointcloud_, cloud2MsgTemp);
    cloud2MsgTemp.header.stamp = clock_->now();
    cloud2MsgTemp.header.frame_id = name_;
    node_2_pub_->publish(cloud2MsgTemp);  
  }

  RCLCPP_INFO(this->get_logger(), "Current operation pair: %d, %d", current_operation_nodes_.first, current_operation_nodes_.second);

}


void PoseGraphEditor::operationCommandCB(const std_msgs::msg::String::SharedPtr msg){

  //@ Since we might have node1_2_node2_af3d_ not been cacluated, but some command arrived
  bool publish_icped_pc = false;
    
  if(current_focus_pg_!=name_)
    return;
  
  std::stringstream ss(msg->data);
  std::string cmd;
  double custom_step = 0.0;
  ss >> cmd;
  bool has_custom_step = false;
  if (ss >> custom_step) {
    has_custom_step = true;
  }

  if (cmd == "trans_step:" || cmd == "trans_step") {
    if (has_custom_step) {
      trans_step_ = custom_step;
    } else {
      std::string val_str = msg->data.substr(msg->data.find(":") + 1);
      try { trans_step_ = std::stod(val_str); } catch (...) {}
    }
    RCLCPP_INFO(this->get_logger(), "Updated trans_step to %.3f", trans_step_);
    return;
  }

  if (cmd == "rot_step:" || cmd == "rot_step") {
    if (has_custom_step) {
      rot_step_ = custom_step;
    } else {
      std::string val_str = msg->data.substr(msg->data.find(":") + 1);
      try { rot_step_ = std::stod(val_str); } catch (...) {}
    }
    RCLCPP_INFO(this->get_logger(), "Updated rot_step to %.3f", rot_step_);
    return;
  }

  //@-----Last step-----
  if(cmd=="last"){
    if(!history_operation_.empty())
      popHistory();
    return;
  }
  
  //@-----Export map-----
  if(cmd=="export_map"){
    //Create sub folder
    std::string export_dir_string = pose_graph_dir_ + "/" + currentDateTime();
    std::filesystem::create_directory(export_dir_string);
    pcl::io::savePCDFileASCII(export_dir_string + "/map.pcd", *map_);
    pcl::io::savePCDFileASCII(export_dir_string + "/ground.pcd", *ground_);
    //@ -----Write poses----- 
    pcl::io::savePCDFileASCII(export_dir_string + "/poses.pcd", poses_);
    
    //@ -----Write graph-----
    pcl::PointCloud<pcl::PointXYZ> edges;
    int edge_number = 0;
    //insert 0,0
    pcl::PointXYZ pt;
    pt.x = edge_number;
    pt.y = 0;
    pt.z = 0;
    edges.push_back(pt);
    edge_number++;
    for(auto it = edges_.begin(); it!=edges_.end(); it++){
      pcl::PointXYZ pt;
      pt.x = edge_number;
      pt.y = (*it).first;
      pt.z = (*it).second;
      edges.push_back(pt);
      edge_number++;
    }
    pcl::io::savePCDFileASCII(export_dir_string + "/edges.pcd", edges);

    //@ -----Write pcd-----
    std::string pcd_dir = export_dir_string + "/pcd";
    std::filesystem::create_directory(pcd_dir);
    for (int i = 0; i < poses_.points.size(); ++i) {
      int thisKeyInd = (int)poses_.points[i].intensity;
      pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_feature.pcd", *cornerCloudKeyFrames_baselink_[thisKeyInd]);
      pcl::io::savePCDFileASCII(pcd_dir + "/" + std::to_string(thisKeyInd) + "_ground.pcd", *surfCloudKeyFrames_baselink_[thisKeyInd]);
    }
    RCLCPP_INFO(this->get_logger(), "Maps are saved: %s", export_dir_string.c_str());
  }  
  
  //@-----ICP-----
  if(cmd=="icp"){
    if(node_1_pointcloud_->points.empty() || node_2_pointcloud_->points.empty()){
      RCLCPP_INFO(this->get_logger(), "Nodes are not selected yet.");
      return;
    }
    if(current_operation_nodes_.first<0 || current_operation_nodes_.second<0){
      RCLCPP_INFO(this->get_logger(), "Nodes are not selected yet.");
      return;      
    }
    //TODO: voxel and then icp, can make result better, because icp will focus clustered area
    // ICP Settings
    /*
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    // Align clouds
    icp.setInputSource(node_2_pointcloud_);
    icp.setInputTarget(node_1_pointcloud_);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    //icp.hasConverged() == false
    //icp.getFitnessScore() > _history_keyframe_fitness_score)
    icp_score_ = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp_score_);
    */
    
    pcl::PointCloud<PointType>::Ptr cloud_source_opti_transformed_ptr;
    cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<PointType>());
    Eigen::Matrix4f T_predict, T_final;
    T_predict = first_frame_2_second_frame_af3_.matrix();

    OptimizedICPGN icp_opti;
    icp_opti.SetTargetCloud(node_1_pointcloud_);
    icp_opti.SetTransformationEpsilon(1e-4);
    icp_opti.SetMaxIterations(100);
    icp_opti.SetMaxCorrespondDistance(20.0);
    icp_opti.Match(node_2_pointcloud_, T_predict, cloud_source_opti_transformed_ptr, T_final);
    icp_score_ = icp_opti.GetFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP score: %.2f", icp_score_);
    
    auto first_frame_2_second_frame_af3 = T_final;//icp.getFinalTransformation();
    node1_2_node2_af3d_ = first_frame_2_second_frame_af3.cast<double>();
    node_2_pointcloud_icp_.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*node_2_pointcloud_, *node_2_pointcloud_icp_, node1_2_node2_af3d_);

    sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
    pcl::toROSMsg(*node_2_pointcloud_icp_, icpedcloud2MsgTemp);
    icpedcloud2MsgTemp.header.stamp = clock_->now();
    icpedcloud2MsgTemp.header.frame_id = name_;
    node_2_pub_->publish(icpedcloud2MsgTemp);
    return;
  }
  
  //@-----Accept the icp result-----
  if(cmd=="accept"){
    add2History();
    PointTypePose node2 = poses_.points[current_operation_nodes_.second];
    //Pose transform
    geometry_msgs::msg::TransformStamped tf2_ros_node1_2_node2 = tf2::eigenToTransform(node1_2_node2_af3d_);
    tf2::Stamped<tf2::Transform> tf2_node1_2_node2;
    tf2::fromMsg(tf2_ros_node1_2_node2, tf2_node1_2_node2);
    RCLCPP_INFO(this->get_logger(), "ICP transform: %.2f, %.2f, %.2f", tf2_node1_2_node2.getOrigin().x(), tf2_node1_2_node2.getOrigin().y(), tf2_node1_2_node2.getOrigin().z());
    
    tf2::Stamped<tf2::Transform> tf2_node2_pose;
    tf2::Quaternion q;
    q.setRPY( node2.roll, node2.pitch, node2.yaw);
    tf2_node2_pose.setRotation(q);
    tf2_node2_pose.setOrigin(tf2::Vector3(node2.x, node2.y, node2.z));
    RCLCPP_INFO(this->get_logger(), "original node 2 pose: %.2f, %.2f, %.2f", tf2_node2_pose.getOrigin().x(), tf2_node2_pose.getOrigin().y(), tf2_node2_pose.getOrigin().z());

    tf2::Transform tf2_icped_node2;
    tf2_icped_node2.mult(tf2_node1_2_node2, tf2_node2_pose); //@ here is very important, it is icp_trans*node_2 pose
    RCLCPP_INFO(this->get_logger(), "icped node 2 pose: %.2f, %.2f, %.2f", tf2_icped_node2.getOrigin().x(), tf2_icped_node2.getOrigin().y(), tf2_icped_node2.getOrigin().z());
    
    std::pair<int, int> edge;
    edge.first = current_operation_nodes_.first;
    edge.second = current_operation_nodes_.second;

    if(!edges_.insert(edge).second)
    { 
      RCLCPP_WARN(this->get_logger(), "Edge: %d --> %d exists.", edge.first, edge.second);
      //return;
    }

    PointTypePose node1 = poses_.points[current_operation_nodes_.first];
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(node1.roll, node1.pitch, node1.yaw), Point3(node1.x, node1.y, node1.z));

    tf2::Matrix3x3 m(tf2_icped_node2.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(roll, pitch, yaw), 
                                Point3(tf2_icped_node2.getOrigin().x(), tf2_icped_node2.getOrigin().y(), tf2_icped_node2.getOrigin().z()));

    float noiseScore = icp_score_;
    gtsam::Vector Vector6(6);
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    gtSAMgraph_.add(
        BetweenFactor<Pose3>(current_operation_nodes_.first, current_operation_nodes_.second,
                            poseFrom.between(poseTo), constraintNoise));
    isam_->update(gtSAMgraph_);
    isam_->update();
    gtSAMgraph_.resize(0);

    isamCurrentEstimate = isam_->calculateEstimate();
    for(auto it=poses_.points.begin(); it!=poses_.points.end(); it++){
      Pose3 est;
      est = isamCurrentEstimate.at<Pose3>((*it).intensity);
      (*it).x = est.translation().x();
      (*it).y = est.translation().y();
      (*it).z = est.translation().z();
      (*it).roll = est.rotation().roll();
      (*it).pitch = est.rotation().pitch();
      (*it).yaw = est.rotation().yaw();
    }
    pubGraph();
    return;
  }

  double trans_step = has_custom_step ? custom_step : trans_step_;
  double rot_step = has_custom_step ? custom_step : rot_step_;

  if(cmd == "px+"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << trans_step, 0.0, 0.0;
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "px-"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -trans_step, 0.0, 0.0;
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "py+"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, trans_step, 0.0;
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "py-"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, -trans_step, 0.0;
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "pz+"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, trans_step;
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "pz-"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, -trans_step;
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "roll+"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (rot_step, Eigen::Vector3f::UnitX()));  
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "roll-"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (-rot_step, Eigen::Vector3f::UnitX()));  
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "pitch+"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (rot_step, Eigen::Vector3f::UnitY()));  
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "pitch-"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (-rot_step, Eigen::Vector3f::UnitY()));  
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "yaw+"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (rot_step, Eigen::Vector3f::UnitZ()));  
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }

  if(cmd == "yaw-"){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate (Eigen::AngleAxisf (-rot_step, Eigen::Vector3f::UnitZ()));  
    first_frame_2_second_frame_af3_ = transform*first_frame_2_second_frame_af3_;
    publish_icped_pc = true;
  }


  if(publish_icped_pc){
    //@ The if statement is to prevent no icp has been done
    node_2_pointcloud_icp_.reset(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*node_2_pointcloud_, *node_2_pointcloud_icp_, first_frame_2_second_frame_af3_);

    sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
    pcl::toROSMsg(*node_2_pointcloud_icp_, icpedcloud2MsgTemp);
    icpedcloud2MsgTemp.header.stamp = clock_->now();
    icpedcloud2MsgTemp.header.frame_id = name_;
    node_2_pub_->publish(icpedcloud2MsgTemp);
  }

}



PoseGraphEditor::~PoseGraphEditor()
{
  
}