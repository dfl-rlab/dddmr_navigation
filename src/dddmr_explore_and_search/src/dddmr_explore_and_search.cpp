#include <cmath>
#include "dddmr_explore_and_search/dddmr_explore_and_search.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <pcl/filters/voxel_grid.h>


using namespace std::chrono_literals;
DddmrExploreAndSearch::DddmrExploreAndSearch() : Node("dddmr_explore_and_search")
{
  has_m2ci_ = false;
  has_c2s_ = false;
  has_b2s_ = false;
  is_goal_reached_ = true;
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  clock_ = this->get_clock();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  key_frame_clouds_.clear();
  base_frame_ = "base_link";
  
  rclcpp::SubscriptionOptions sub_options;
  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_options.callback_group = cbs_group_;
  // Initialize PoseArray subscriber
  cloudKeyPoses6D_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_keypose_6d", 1, std::bind(&DddmrExploreAndSearch::cloudKeyPoses6D_callback, this, std::placeholders::_1), sub_options);
  m2ci_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "lego_loam/m2ci", 1, std::bind(&DddmrExploreAndSearch::m2ci_callback, this, std::placeholders::_1), sub_options);
  c2s_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "lego_loam/c2s", 1, std::bind(&DddmrExploreAndSearch::c2s_callback, this, std::placeholders::_1), sub_options);
  b2s_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "lego_loam/b2s", 1, std::bind(&DddmrExploreAndSearch::b2s_callback, this, std::placeholders::_1), sub_options);    

  // Initialize Publisher
  all_edge_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("explore_and_search/all_edge_ground", 1);

  get_key_frame_cloud_client_ = this->create_client<dddmr_sys_core::srv::GetKeyFrameCloud>("get_key_frame_cloud", rmw_qos_profile_services_default, cbs_group_);
  
  srv_cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Initialize Timer
  sync_ground_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&DddmrExploreAndSearch::syncGroundCb, this), srv_cbs_group_);

  p2p_move_base_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  p2p_move_base_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::PToPMoveBase>(
      this,
      "p2p_move_base", p2p_move_base_client_group_);

  // Initialize Ground Publish Timer
  ground_publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&DddmrExploreAndSearch::findExplorationGoalCb, this), srv_cbs_group_);
}

void DddmrExploreAndSearch::m2ci_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  trans_m2ci_af3_ = tf2::transformToEigen(*msg);
  tf2_trans_m2ci_.setRotation(tf2::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w));
  tf2_trans_m2ci_.setOrigin(tf2::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  has_m2ci_ = true;
}
void DddmrExploreAndSearch::c2s_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  trans_c2s_af3_ = tf2::transformToEigen(*msg);
  tf2_trans_c2s_.setRotation(tf2::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w));
  tf2_trans_c2s_.setOrigin(tf2::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  has_c2s_ = true;
}
void DddmrExploreAndSearch::b2s_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  trans_b2s_af3_ = tf2::transformToEigen(*msg);
  tf2_trans_b2s_.setRotation(tf2::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w));
  tf2_trans_b2s_.setOrigin(tf2::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  has_b2s_ = true;
}

void DddmrExploreAndSearch::cloudKeyPoses6D_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
  cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *cloudKeyPoses6D);
  for(auto it=cloudKeyPoses6D->points.begin(); it!=cloudKeyPoses6D->points.end(); it++){
    PointType pt;
    pt.x = (*it).x; pt.y = (*it).y; pt.z = (*it).z;
    cloudKeyPoses3D->push_back(pt);
  }
}

pcl::PointCloud<PointType>::Ptr DddmrExploreAndSearch::transformPointCloud(
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

void DddmrExploreAndSearch::syncGroundCb()
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
          //RCLCPP_INFO(this->get_logger(), "Empty key frame");
          return;
        }
        pcl::fromROSMsg(result->key_frame_ground, pcl_ground_cloud);
        if(pcl_ground_cloud.points.size()<1){
          //RCLCPP_INFO(this->get_logger(), "Empty key frame");
          return;
        }

        pcl::fromROSMsg(result->key_frame_ground_edge, pcl_ground_edge_cloud);
        if(pcl_ground_edge_cloud.points.size()<1){
          //RCLCPP_INFO(this->get_logger(), "Empty key frame");
          return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Sync key frame number: %lu with total size: %lu", key_frame_clouds_.size(), cloudKeyPoses6D->size());
        key_frame_clouds_.push_back(pcl_cloud);
        key_frame_ground_clouds_.push_back(pcl_ground_cloud);
        key_frame_ground_edge_clouds_.push_back(pcl_ground_edge_cloud);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }
    });

}

void DddmrExploreAndSearch::findExplorationGoalCb()
{

  /*
  Implement your search and explore strategy here.
  Our default exploration strategy is to randomly search a frontier from the ground edge
  */
  if (key_frame_ground_clouds_.empty()) {
    return;
  }

  pcl::PointCloud<PointType>::Ptr all_edge_cloud(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr all_ground_cloud(new pcl::PointCloud<PointType>());
  for (size_t it_id=0;it_id<key_frame_ground_edge_clouds_.size();it_id++) {

    if(it_id>=key_frame_ground_edge_clouds_.size())
      continue;    
    
    pcl::PointCloud<PointType> one_frame_edge_cloud;
    one_frame_edge_cloud= *transformPointCloud(key_frame_ground_edge_clouds_[it_id].makeShared(), &cloudKeyPoses6D->points[it_id]);
    pcl::transformPointCloud(one_frame_edge_cloud, one_frame_edge_cloud, trans_m2ci_af3_);
    *all_edge_cloud += one_frame_edge_cloud;

    pcl::PointCloud<PointType> one_frame_ground_cloud;
    one_frame_ground_cloud= *transformPointCloud(key_frame_ground_clouds_[it_id].makeShared(), &cloudKeyPoses6D->points[it_id]);
    pcl::transformPointCloud(one_frame_ground_cloud, one_frame_ground_cloud, trans_m2ci_af3_);
    *all_ground_cloud += one_frame_ground_cloud;
  }
  
  kdtree_ground_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*all_edge_cloud, output_msg);
  
  output_msg.header.frame_id = "map";
  output_msg.header.stamp = this->now();

  all_edge_cloud_pub_->publish(output_msg);

  if(is_goal_reached_){
    std::random_device rd; 
    std::mt19937 gen(rd()); 
    std::uniform_int_distribution<> distr(0, all_edge_cloud->points.size()-1); 
    auto selected_random_point = all_edge_cloud->points[distr(gen)];

    //@ find a ground near edge for a safety navigation
    size_t ground_safety_index = 0; 
    kdtree_ground_->setInputCloud(all_ground_cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree_ground_->radiusSearch(selected_random_point, 2.0, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    for(size_t pit=0; pit<pointIdxRadiusSearch.size(); pit++){
      if(sqrt(pointRadiusSquaredDistance[pit])>0.5)
        ground_safety_index = pit;
    }

    geometry_msgs::msg::PoseStamped m_target_pose;
    m_target_pose.pose.orientation.w = 1.0;
    m_target_pose.pose.position.x = all_ground_cloud->points[ground_safety_index].x;
    m_target_pose.pose.position.y = all_ground_cloud->points[ground_safety_index].y;
    m_target_pose.pose.position.z = all_ground_cloud->points[ground_safety_index].z;
    RCLCPP_INFO(this->get_logger(), "Send a goal to %.2f, %.2f ,%.2f", m_target_pose.pose.position.x, m_target_pose.pose.position.y, m_target_pose.pose.position.z);
    sendP2P(m_target_pose);
  }
}

void DddmrExploreAndSearch::sendP2P(geometry_msgs::msg::PoseStamped m_target_pose){

  is_goal_reached_ = false;
  auto goal_msg = dddmr_sys_core::action::PToPMoveBase::Goal();
  goal_msg.target_pose = m_target_pose;

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::PToPMoveBase>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&DddmrExploreAndSearch::p2p_move_base_client_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&DddmrExploreAndSearch::p2p_move_base_client_result_callback, this, std::placeholders::_1);
  
  p2p_move_base_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void DddmrExploreAndSearch::p2p_move_base_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::PToPMoveBase>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by p2p server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by p2p server, waiting for result");
  }
}

void DddmrExploreAndSearch::p2p_move_base_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::PToPMoveBase>::WrappedResult & result)
{

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      is_goal_reached_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      is_goal_reached_ = true;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      is_goal_reached_ = true;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      is_goal_reached_ = true;
      break;
  }
  
}