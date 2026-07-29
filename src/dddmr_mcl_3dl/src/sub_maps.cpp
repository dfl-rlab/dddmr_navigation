/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mcl_3dl/sub_maps.h>
namespace mcl_3dl
{
SubMaps::SubMaps(std::string name) : Node(name), is_current_ready_(false), 
  prepare_warm_up_(false), is_warm_up_ready_(false), is_initial_(false), key_poses_received_(false),
  knn_num_of_ground_normals_(20){

  map_voxel_omp_.setNumberOfThreads(4);
  map_voxel_omp_.setLeafSize (0.2, 0.2, 0.2);
  map_voxel_omp_.setSaveLeafLayout(false);

  ground_voxel_omp_.setNumberOfThreads(4);
  ground_voxel_omp_.setLeafSize (0.2, 0.2, 0.2);
  ground_voxel_omp_.setSaveLeafLayout(false);

  map_current_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  ground_current_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  map_warmup_ = std::make_shared<pcl::PointCloud<pcl_t>>();
  ground_warmup_ = std::make_shared<pcl::PointCloud<pcl_t>>();

  clock_ = this->get_clock();
  
  access_ = new sub_maps_mutex_t();

  declare_parameter("pg_map_server_name", rclcpp::ParameterValue(""));
  this->get_parameter("pg_map_server_name", pg_map_server_name_);
  RCLCPP_INFO(this->get_logger(), "pg_map_server_name: %s", pg_map_server_name_.c_str());

  declare_parameter("sub_map_search_radius", rclcpp::ParameterValue(50.0));
  this->get_parameter("sub_map_search_radius", sub_map_search_radius_);
  RCLCPP_INFO(this->get_logger(), "sub_map_search_radius: %.1f", sub_map_search_radius_);

  declare_parameter("sub_map_warmup_trigger_distance", rclcpp::ParameterValue(20.0));
  this->get_parameter("sub_map_warmup_trigger_distance", sub_map_warmup_trigger_distance_);
  RCLCPP_INFO(this->get_logger(), "sub_map_warmup_trigger_distance: %.1f", sub_map_warmup_trigger_distance_);
  
  srv_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  get_key_frame_cloud_client_ = this->create_client<dddmr_sys_core::srv::GetKeyFrameCloud>(
    pg_map_server_name_ + "/get_key_frame_cloud", rmw_qos_profile_services_default, srv_group_);

  sub_key_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    pg_map_server_name_ + "/key_poses", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&SubMaps::keyPosesCb, this, std::placeholders::_1));

  //@ Latched topic, Create a publisher using the QoS settings to emulate a ROS1 latched topic
  pub_sub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_map_warmup_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapcloud_warmup",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_ground_warmup_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapground_warmup",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sync_map_timer_ = this->create_wall_timer(1ms, std::bind(&SubMaps::syncMapThread, this), timer_group_);
  warm_up_timer_ = this->create_wall_timer(200ms, std::bind(&SubMaps::warmUpThread, this), timer_group_);
}

SubMaps::~SubMaps(){
  delete access_;
}

void SubMaps::keyPosesCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  poses_pcl_t_.reset(new pcl::PointCloud<pcl_t>());
  for(auto i=msg->poses.begin();i!=msg->poses.end();i++){
    pcl_t pt;
    pt.x = (*i).position.x;
    pt.y = (*i).position.y;
    pt.z = (*i).position.z;
    poses_pcl_t_->push_back(pt);
  }
  kdtree_poses_.reset(new pcl::KdTreeFLANN<pcl_t>());
  kdtree_poses_->setInputCloud(poses_pcl_t_);
  key_poses_received_ = true;
}

void SubMaps::syncMapThread(){

  if(!key_poses_received_)
    return;

  auto request = std::make_shared<dddmr_sys_core::srv::GetKeyFrameCloud::Request>();
  request->key_frame_number = cornerCloudKeyFrames_.size();
  
  if(request->key_frame_number>=poses_pcl_t_->size())
  {
    RCLCPP_WARN(this->get_logger(), "Exceed request key frame number, shutdown thread.");
    RCLCPP_INFO(this->get_logger(), "Sync cloud key frame number: %lu with total size: %lu", cornerCloudKeyFrames_.size(), poses_pcl_t_->size());
    RCLCPP_INFO(this->get_logger(), "Sync surface key frame number: %lu with total size: %lu", surfCloudKeyFrames_.size(), poses_pcl_t_->size());
    RCLCPP_INFO(this->get_logger(), "Sync ground key frame number: %lu with total size: %lu", groundCloudKeyFrames_.size(), poses_pcl_t_->size());
    is_initial_ = true;
    sync_map_timer_->cancel();
    return; 
  }

  if (!get_key_frame_cloud_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Service %s/get_key_frame_cloud not available", pg_map_server_name_.c_str());
    return;
  }

  get_key_frame_cloud_client_->async_send_request(request, 
    [this](rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedFuture future) {
      try {
        auto result = future.get();
        pcl::PointCloud<pcl_t> pcl_cloud;
        pcl::PointCloud<pcl_t> pcl_surface_cloud;
        pcl::PointCloud<pcl_t> pcl_ground_cloud;

        pcl::fromROSMsg(result->key_frame_cloud, pcl_cloud);
        if(pcl_cloud.points.size()<1){
          RCLCPP_DEBUG(this->get_logger(), "Empty key frame cloud");
        }
        pcl::fromROSMsg(result->key_frame_surface, pcl_surface_cloud);
        if(pcl_surface_cloud.points.size()<1){
          RCLCPP_DEBUG(this->get_logger(), "Empty key frame surface");
        }

        pcl::fromROSMsg(result->key_frame_ground, pcl_ground_cloud);
        if(pcl_ground_cloud.points.size()<1){
          RCLCPP_DEBUG(this->get_logger(), "Empty key frame ground");
        }

        pcl::PointCloud<pcl_t> pcl_cloud_base_link;
        pcl::PointCloud<pcl_t> pcl_surface_cloud_base_link;
        pcl::PointCloud<pcl_t> pcl_ground_cloud_base_link;

        pcl::fromROSMsg(result->key_frame_cloud, pcl_cloud_base_link);
        if(pcl_cloud_base_link.points.size()<1){
          RCLCPP_DEBUG(this->get_logger(), "Empty key frame cloud base_link");
        }
        pcl::fromROSMsg(result->key_frame_surface, pcl_surface_cloud_base_link);
        if(pcl_surface_cloud.points.size()<1){
          RCLCPP_DEBUG(this->get_logger(), "Empty key frame surface base_link");
        }

        pcl::fromROSMsg(result->key_frame_ground, pcl_ground_cloud_base_link);
        if(pcl_ground_cloud_base_link.points.size()<1){
          RCLCPP_DEBUG(this->get_logger(), "Empty key frame ground base_link");
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Sync key frame number: %lu with total size: %lu", cornerCloudKeyFrames_.size(), poses_pcl_t_->size());
        cornerCloudKeyFrames_.push_back(pcl_cloud.makeShared());
        surfCloudKeyFrames_.push_back(pcl_surface_cloud.makeShared());
        groundCloudKeyFrames_.push_back(pcl_ground_cloud.makeShared());
        cornerCloudKeyFrames_baselink_.push_back(pcl_cloud_base_link.makeShared());
        surfCloudKeyFrames_baselink_.push_back(pcl_surface_cloud_base_link.makeShared());
        groundCloudKeyFrames_baselink_.push_back(pcl_ground_cloud_base_link.makeShared());

      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    });

}

void SubMaps::warmUpThread(){
  
  if(!is_initial_)
    return;
  
  if(!is_current_ready_){
    mcl_3dl::pcl_t target_pose;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    target_pose.x = robot_pose_.pose.pose.position.x;
    target_pose.y = robot_pose_.pose.pose.position.y;
    target_pose.z = robot_pose_.pose.pose.position.z;
    if(kdtree_poses_->radiusSearch(target_pose, sub_map_search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance,0)<1)
    {
      is_current_ready_ = false;
      return;
    }
    map_current_->clear();
    ground_current_->clear();
    for(auto it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); it++){
      *map_current_ += (*cornerCloudKeyFrames_[*it]);
      //*map_current_ += (*surfCloudKeyFrames_[*it]);
      *ground_current_ += (*groundCloudKeyFrames_[*it]);
    }

    map_voxel_omp_.setInputCloud(map_current_);
    map_voxel_omp_.setFinalFilter(true);
    map_voxel_omp_.filter(*map_current_);  

    ground_voxel_omp_.setInputCloud(ground_current_);
    ground_voxel_omp_.setFinalFilter(true);
    ground_voxel_omp_.filter(*ground_current_);  

    kdtree_ground_current_.setInputCloud(ground_current_);
    kdtree_map_current_.setInputCloud(map_current_);   
    //@Normal estimation for ground
    pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n;
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    tree->setInputCloud (ground_current_);
    n.setInputCloud (ground_current_);
    n.setSearchMethod (tree);
    n.setKSearch (knn_num_of_ground_normals_);
    n.compute (normals_ground_current_);

    sensor_msgs::msg::PointCloud2 map_pc;
    pcl::toROSMsg(*map_current_, map_pc);
    map_pc.header.frame_id = "map";
    pub_sub_map_->publish(map_pc);

    sensor_msgs::msg::PointCloud2 ground_pc;
    pcl::toROSMsg(*ground_current_, ground_pc);
    ground_pc.header.frame_id = "map";
    pub_sub_ground_->publish(ground_pc);
    
    current_sub_map_pose_ = robot_pose_;
    RCLCPP_INFO(this->get_logger(), "All essential KD-tree are generated.");
    
    is_current_ready_ = true;
  }
  
  else if(prepare_warm_up_ && !is_warm_up_ready_){
    mcl_3dl::pcl_t target_pose;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    target_pose.x = warm_up_pose_.pose.pose.position.x;
    target_pose.y = warm_up_pose_.pose.pose.position.y;
    target_pose.z = warm_up_pose_.pose.pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Prepare warm up at: %.2f, %.2f, %.2f", target_pose.x, target_pose.y, target_pose.z);
    if(kdtree_poses_->radiusSearch(target_pose, sub_map_search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance,0)<1)
    {
      prepare_warm_up_ = false;
      return;
    }
    map_warmup_->clear();
    ground_warmup_->clear();
    for(auto it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); it++){
      *map_warmup_ += (*cornerCloudKeyFrames_[*it]);
      //*map_warmup_ += (*surfCloudKeyFrames_[*it]);
      *ground_warmup_ += (*groundCloudKeyFrames_[*it]);
    }

    map_voxel_omp_.setInputCloud(map_warmup_);
    map_voxel_omp_.setFinalFilter(true);
    map_voxel_omp_.filter(*map_warmup_);  

    ground_voxel_omp_.setInputCloud(ground_warmup_);
    ground_voxel_omp_.setFinalFilter(true);
    ground_voxel_omp_.filter(*ground_warmup_);  

    kdtree_ground_warmup_.setInputCloud(ground_warmup_);
    kdtree_map_warmup_.setInputCloud(map_warmup_);   
    //@Normal estimation for ground
    pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n;
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    tree->setInputCloud (ground_warmup_);
    n.setInputCloud (ground_warmup_);
    n.setSearchMethod (tree);
    n.setKSearch (knn_num_of_ground_normals_);
    n.compute (normals_ground_warmup_);
    sensor_msgs::msg::PointCloud2 map_pc;
    pcl::toROSMsg(*map_warmup_, map_pc);
    map_pc.header.frame_id = "map";
    pub_sub_map_warmup_->publish(map_pc);

    sensor_msgs::msg::PointCloud2 ground_pc;
    pcl::toROSMsg(*ground_warmup_, ground_pc);
    ground_pc.header.frame_id = "map";
    pub_sub_ground_warmup_->publish(ground_pc);
    
    RCLCPP_INFO(this->get_logger(), "All essential Warmup KD-tree are generated.");
    prepare_warm_up_ = false;
    is_warm_up_ready_ = true;
  }
}

bool SubMaps::isWarmUpReady(){
  return is_warm_up_ready_;
}

void SubMaps::swapKdTree(){
  
  map_current_->clear();
  ground_current_->clear();
  *map_current_ = *map_warmup_;
  *ground_current_ = *ground_warmup_;
  kdtree_map_current_ = kdtree_map_warmup_;
  kdtree_ground_current_ = kdtree_ground_warmup_;
  normals_ground_current_ = normals_ground_warmup_;
  current_sub_map_pose_ = warm_up_pose_;

  sensor_msgs::msg::PointCloud2 map_pc;
  pcl::toROSMsg(*map_warmup_, map_pc);
  map_pc.header.frame_id = "map";
  pub_sub_map_->publish(map_pc);

  sensor_msgs::msg::PointCloud2 ground_pc;
  pcl::toROSMsg(*ground_warmup_, ground_pc);
  ground_pc.header.frame_id = "map";
  pub_sub_ground_->publish(ground_pc);

  is_warm_up_ready_ = false;
  RCLCPP_INFO(this->get_logger(), "KD-tree swapped.");
}

void SubMaps::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose){
  RCLCPP_INFO(this->get_logger(), "Receive initial pose at: %.2f, %.2f, %.2f", pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  prepare_warm_up_ = true;
  warm_up_pose_ = pose;
}

void SubMaps::setPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose){
  robot_pose_ = pose;
  double dx = pose.pose.pose.position.x - current_sub_map_pose_.pose.pose.position.x;
  double dy = pose.pose.pose.position.y - current_sub_map_pose_.pose.pose.position.y;
  double dz = pose.pose.pose.position.z - current_sub_map_pose_.pose.pose.position.z;
  if(!is_warm_up_ready_ && sqrt(dx*dx + dy*dy + dz*dz)>=sub_map_warmup_trigger_distance_){
    prepare_warm_up_ = true;
    warm_up_pose_ = pose;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Warming up sub maps");
  }
}

}