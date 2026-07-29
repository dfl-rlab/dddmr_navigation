#include <cmath>
#include "lego_loam_map_visualization.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <pcl/filters/voxel_grid.h>


using namespace std::chrono_literals;
LegoLoamVisualization::LegoLoamVisualization(std::string name) : Node(name)
{
  has_m2ci_ = false;
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
  clock_ = this->get_clock();
  key_frame_clouds_.clear();

  declare_parameter("ground_voxel_size", rclcpp::ParameterValue(0.3f));
  this->get_parameter("ground_voxel_size", ground_voxel_size_);
  RCLCPP_INFO(this->get_logger(), "ground_voxel_size: %.2f", ground_voxel_size_);
  downSizeFilterGlobalGroundKeyFrames_Copy_omp.setNumberOfThreads(6);
  downSizeFilterGlobalGroundKeyFrames_Copy_omp.setLeafSize (ground_voxel_size_, ground_voxel_size_, ground_voxel_size_);
  downSizeFilterGlobalGroundKeyFrames_Copy_omp.setSaveLeafLayout(false);
  
  ds_patched_ground_omp_.setLeafSize(0.1, 0.5, 0.1); //we are in camera frame, z pointing to moving direction, y pointing to sky 
  ds_patched_ground_omp_.setNumberOfThreads(6);
  ds_patched_ground_omp_.setSaveLeafLayout(false);

  declare_parameter("ground_edge_threshold_num", rclcpp::ParameterValue(50));
  this->get_parameter("ground_edge_threshold_num", ground_edge_threshold_num_);
  RCLCPP_INFO(this->get_logger(), "ground_edge_threshold_num: %d", ground_edge_threshold_num_);
  
  pubMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_map", 1);  
  pubGround = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground", 1);  
  pubGroundEdge = this->create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground_edge", 1);

  rclcpp::SubscriptionOptions sub_options;
  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_options.callback_group = cbs_group_;
  // Initialize PoseArray subscriber
  cloudKeyPoses6D_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_keypose_6d", 1, std::bind(&LegoLoamVisualization::cloudKeyPoses6D_callback, this, std::placeholders::_1), sub_options);
  m2ci_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "lego_loam/m2ci", 1, std::bind(&LegoLoamVisualization::m2ci_callback, this, std::placeholders::_1), sub_options);

  // Initialize Service Client
 
  get_key_frame_cloud_client_ = this->create_client<dddmr_sys_core::srv::GetKeyFrameCloud>("get_key_frame_cloud", rmw_qos_profile_services_default, cbs_group_);

  // Initialize Timer
  sync_map_and_ground_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&LegoLoamVisualization::syncMapAndGroundThread, this), cbs_group_);

  // Initialize Ground Publish Timer
  map_publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&LegoLoamVisualization::pubMapThread, this), cbs_group_);
  
  timer_ground_edge_detection_ = this->create_wall_timer(200ms, std::bind(&LegoLoamVisualization::groundEdgeDetectionThread, this), cbs_group_);
}

void LegoLoamVisualization::m2ci_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  trans_m2ci_af3_ = tf2::transformToEigen(*msg);
  has_m2ci_ = true;
}

void LegoLoamVisualization::cloudKeyPoses6D_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *cloudKeyPoses6D);
  for(auto it=cloudKeyPoses6D->points.begin();it!=cloudKeyPoses6D->points.end();it++){
    PointType pt;
    pt.x = it->x;
    pt.y = it->y;
    pt.z = it->z;
    cloudKeyPoses3D->push_back(pt);
  }
  
}

void LegoLoamVisualization::syncMapAndGroundThread()
{

  auto request = std::make_shared<dddmr_sys_core::srv::GetKeyFrameCloud::Request>();
  request->key_frame_number = key_frame_clouds_.size();
  
  if(request->key_frame_number>=cloudKeyPoses6D->size())
  {
   return; 
  }

  if (!get_key_frame_cloud_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Service get_key_frame_cloud not available");
    return;
  }

  get_key_frame_cloud_client_->async_send_request(request, 
    [this](rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedFuture future) {
      try {
        auto result = future.get();
        pcl::PointCloud<PointType> pcl_cloud;
        pcl::PointCloud<PointType> pcl_ground_cloud;
        pcl::PointCloud<PointType> pcl_ground_edge_cloud;

        pcl::fromROSMsg(result->key_frame_cloud, pcl_cloud);
        if(pcl_cloud.points.size()<1){
          RCLCPP_INFO(this->get_logger(), "Empty key frame1");
          return;
        }
        pcl::fromROSMsg(result->key_frame_ground, pcl_ground_cloud);
        if(pcl_ground_cloud.points.size()<1){
          RCLCPP_INFO(this->get_logger(), "Empty key frame2");
          return;
        }

        pcl::fromROSMsg(result->key_frame_ground_edge, pcl_ground_edge_cloud);
        if(pcl_ground_edge_cloud.points.size()<1){
          RCLCPP_INFO(this->get_logger(), "Empty key frame3");
          return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Sync key frame number: %lu with total size: %lu", key_frame_clouds_.size(), cloudKeyPoses6D->size());
        key_frame_clouds_.push_back(pcl_cloud.makeShared());
        patchedGroundKeyFrames.push_back(pcl_ground_cloud.makeShared());
        patchedGroundEdgeKeyFrames.push_back(pcl_ground_edge_cloud.makeShared());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    });

}

void LegoLoamVisualization::pubMapThread()
{
  if(!has_m2ci_)
    return;

  pcl::PointCloud<PointType> map_cloud;
  pcl::PointCloud<PointType> ground_cloud;
  size_t cnt = 0;
  for(auto it=key_frame_clouds_.begin();it!=key_frame_clouds_.end();it++){
    pcl::PointCloud<PointType> one_frame_map_cloud;
    one_frame_map_cloud= *transformPointCloud(*it, &cloudKeyPoses6D->points[cnt]);
    pcl::transformPointCloud(one_frame_map_cloud, one_frame_map_cloud, trans_m2ci_af3_);
    map_cloud+=one_frame_map_cloud;
    cnt++;
  }

  for(size_t it=0;it<patchedGroundKeyFrames.size();it++){
    pcl::PointCloud<PointType> one_frame_map_cloud;
    one_frame_map_cloud = *transformPointCloud(patchedGroundKeyFrames[it], &cloudKeyPoses6D->points[it]);
    pcl::transformPointCloud(one_frame_map_cloud, one_frame_map_cloud, trans_m2ci_af3_);
    ground_cloud+=one_frame_map_cloud;
    if(it>=patchedGroundEdgeProcessedKeyFrames.size())
      continue;
    pcl::PointCloud<PointType> one_frame_map_cloud2;
    one_frame_map_cloud2 = *transformPointCloud(patchedGroundEdgeProcessedKeyFrames[it], &cloudKeyPoses6D->points[it]);
    pcl::transformPointCloud(one_frame_map_cloud2, one_frame_map_cloud2, trans_m2ci_af3_);
    ground_cloud+=one_frame_map_cloud2;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg_map;
  pcl::toROSMsg(map_cloud, cloud_msg_map);
  cloud_msg_map.header.stamp = clock_->now();
  cloud_msg_map.header.frame_id = "map";
  pubMap->publish(cloud_msg_map);

  sensor_msgs::msg::PointCloud2 cloud_msg_ground;
  pcl::PointCloud<PointType>::Ptr content_ground_ptr(new pcl::PointCloud<PointType>(ground_cloud));
  pcl::PointCloud<PointType> content_ground_filtered;
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(content_ground_ptr);
  sor.setLeafSize(ground_voxel_size_, ground_voxel_size_, ground_voxel_size_);
  sor.filter(content_ground_filtered);

  pcl::toROSMsg(content_ground_filtered, cloud_msg_ground);
  cloud_msg_ground.header.stamp = clock_->now();
  cloud_msg_ground.header.frame_id = "map";
  pubGround->publish(cloud_msg_ground);
}

void LegoLoamVisualization::groundEdgeDetectionThread() {

  if(!has_m2ci_)
    return;

  if(cloudKeyPoses3D->points.empty()) 
    return;
  
  if(ground_edge_processed_.size()>=cloudKeyPoses6D->points.size()){
    return;
  }
  if(ground_edge_processed_.size()>=patchedGroundEdgeKeyFrames.size()){
    return;
  }
  if(ground_edge_processed_.size()>=patchedGroundKeyFrames.size()){
    return;
  }  

  PointType current_processing_ground_edge_num;
  current_processing_ground_edge_num = cloudKeyPoses3D->points[ground_edge_processed_.size()];

  pcl::PointCloud<PointType>::Ptr patched_ground;
  patched_ground.reset(new pcl::PointCloud<PointType>());
  pcl::KdTreeFLANN<PointType> kdtree_key_pose;
  kdtree_key_pose.setInputCloud(cloudKeyPoses3D);
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  kdtree_key_pose.radiusSearch(current_processing_ground_edge_num, 10.0, pointSearchInd, pointSearchSqDis);
  for(auto i=pointSearchInd.begin();i!=pointSearchInd.end();i++)
  {  
    //RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f", cloudKeyPoses6D->points[*i].x, cloudKeyPoses6D->points[*i].y, cloudKeyPoses6D->points[*i].z);
    if(fabs(current_processing_ground_edge_num.y-cloudKeyPoses6D->points[*i].y)<1.0){
      if(*i>=patchedGroundKeyFrames.size())
        continue;
      *patched_ground += *transformPointCloud(patchedGroundKeyFrames[*i], &cloudKeyPoses6D->points[*i]);
    }
  }

  //@ generate ground kdtree for edge to search

  ds_patched_ground_omp_.setInputCloud(patched_ground);
  ds_patched_ground_omp_.setFinalFilter(true);
  ds_patched_ground_omp_.filter(*patched_ground);
  pcl::KdTreeFLANN<PointType> kdtree_ground;
  kdtree_ground.setInputCloud(patched_ground);
  
  //@ reprocess old edge frame, because we had new ground information, so new edge will be pushed back to keypose frames
  pcl::PointCloud<PointType>::Ptr patched_ground_edge_camera_frame;
  
  pointSearchInd.clear();
  pointSearchSqDis.clear();
  kdtree_key_pose.radiusSearch(current_processing_ground_edge_num, 5.0, pointSearchInd, pointSearchSqDis);

  for(auto i=pointSearchInd.begin();i!=pointSearchInd.end();i++)
  {
    if(fabs(current_processing_ground_edge_num.y - cloudKeyPoses3D->points[*i].y)>1.0){
      continue;
    }
    
    if((*i)>=patchedGroundEdgeProcessedKeyFrames.size())
      continue;

    pcl::PointCloud<PointType>::Ptr processed_ground_edge_last;
    processed_ground_edge_last.reset(new pcl::PointCloud<PointType>());
    patched_ground_edge_camera_frame.reset(new pcl::PointCloud<PointType>());
    *patched_ground_edge_camera_frame = *transformPointCloud(patchedGroundEdgeKeyFrames[*i], &cloudKeyPoses6D->points[*i]);
    for(size_t j=0;j!=patched_ground_edge_camera_frame->points.size();j++){
      PointType current_pt = patched_ground_edge_camera_frame->points[j];
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(!pcl::isFinite(current_pt))
        continue;
      
      kdtree_ground.radiusSearch (current_pt, 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(pointIdxRadiusSearch.size()<ground_edge_threshold_num_){
        patched_ground_edge_camera_frame->points[j].intensity = 1000;
        processed_ground_edge_last->push_back(patched_ground_edge_camera_frame->points[j]);
        
        for(auto k=0;k!=pointIdxRadiusSearch.size();k++){
          auto index = pointIdxRadiusSearch[k];
          double dx = patched_ground->points[index].x - patched_ground_edge_camera_frame->points[j].x;
          //double dy = patched_ground->points[index].y - patched_ground_edge_camera_frame->points[j].y;
          double dz = patched_ground->points[index].z - patched_ground_edge_camera_frame->points[j].z;
          double distance = sqrt(dx*dx+dz*dz);
          patched_ground->points[index].intensity = 100.0/(1.0+distance);
          processed_ground_edge_last->push_back(patched_ground->points[index]);
        }
        
      }

    }

    pcl::VoxelGrid<PointType> ds_processed_ground_edge_last;
    ds_processed_ground_edge_last.setLeafSize(0.1, 0.2, 0.1); //we are in camera frame, z pointing to moving direction, y pointing to sky 
    ds_processed_ground_edge_last.setInputCloud(processed_ground_edge_last);
    ds_processed_ground_edge_last.filter(*processed_ground_edge_last);  
    *processed_ground_edge_last = *transformPointCloudInverse(processed_ground_edge_last, &cloudKeyPoses6D->points[*i]);  
    patchedGroundEdgeProcessedKeyFrames[*i] = processed_ground_edge_last;
  
  }
  
  //@ process current frame
  pcl::PointCloud<PointType>::Ptr processed_ground_edge_current;
  processed_ground_edge_current.reset(new pcl::PointCloud<PointType>());
  patched_ground_edge_camera_frame.reset(new pcl::PointCloud<PointType>());
  *patched_ground_edge_camera_frame = *transformPointCloud(patchedGroundEdgeKeyFrames[patchedGroundEdgeKeyFrames.size()-1], &cloudKeyPoses6D->points[patchedGroundEdgeKeyFrames.size()-1]);
  for(size_t it=0;it!=patched_ground_edge_camera_frame->points.size();it++){
    PointType current_pt = patched_ground_edge_camera_frame->points[it];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!pcl::isFinite(current_pt))
      continue;
    if(kdtree_ground.radiusSearch (current_pt, 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance)<ground_edge_threshold_num_){
      patched_ground_edge_camera_frame->points[it].intensity = 1000;
      processed_ground_edge_current->push_back(patched_ground_edge_camera_frame->points[it]);
    }
  }
  *processed_ground_edge_current = *transformPointCloudInverse(processed_ground_edge_current, &cloudKeyPoses6D->points[patchedGroundEdgeKeyFrames.size()-1]);
  patchedGroundEdgeProcessedKeyFrames.push_back(processed_ground_edge_current);
  ground_edge_processed_.push_back(true);
  
  pcl::PointCloud<PointType>::Ptr  globalGroundEdgeKeyFrames;
  globalGroundEdgeKeyFrames.reset(new pcl::PointCloud<PointType>());
  for (int i = 0; i < ground_edge_processed_.size(); ++i) {
    *globalGroundEdgeKeyFrames += *transformPointCloud(
        patchedGroundEdgeProcessedKeyFrames[i], &cloudKeyPoses6D->points[i]);
  }

  //@ transform to map frame --> z pointing to sky
  pcl::transformPointCloud(*globalGroundEdgeKeyFrames, *globalGroundEdgeKeyFrames, trans_m2ci_af3_);
  //RCLCPP_INFO(this->get_logger(),"%lu, %lu", ground_edge_processed_.size(), globalGroundEdgeKeyFrames->points.size());
  //downSizeFilterGlobalGroundKeyFrames_Copy.setInputCloud(globalGroundEdgeKeyFrames);
  //downSizeFilterGlobalGroundKeyFrames_Copy.filter(*globalGroundEdgeKeyFrames);
  downSizeFilterGlobalGroundKeyFrames_Copy_omp.setInputCloud(globalGroundEdgeKeyFrames);
  downSizeFilterGlobalGroundKeyFrames_Copy_omp.setFinalFilter(true);
  downSizeFilterGlobalGroundKeyFrames_Copy_omp.filter(*globalGroundEdgeKeyFrames);
  sensor_msgs::msg::PointCloud2 cloud_msg_ground_edge;
  pcl::toROSMsg(*globalGroundEdgeKeyFrames, cloud_msg_ground_edge);
  cloud_msg_ground_edge.header.stamp = clock_->now();
  cloud_msg_ground_edge.header.frame_id = "map";
  pubGroundEdge->publish(cloud_msg_ground_edge);
  
}

pcl::PointCloud<PointType>::Ptr LegoLoamVisualization::transformPointCloud(
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
  pcl_opt::transformPointCloudSequentially(*cloudIn, *cloudOut2, af3_yaw.matrix(), af3_roll.matrix(), af3_pitch.matrix(), af3_translation.matrix());

  return cloudOut2;
}

pcl::PointCloud<PointType>::Ptr LegoLoamVisualization::transformPointCloudInverse(
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