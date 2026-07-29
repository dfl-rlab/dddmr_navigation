#ifndef DDDMR_EXPLORE_AND_SEARCH_HPP_
#define DDDMR_EXPLORE_AND_SEARCH_HPP_

#include <random> 
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <dddmr_sys_core/srv/get_key_frame_cloud.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dddmr_explore_and_search/camera_frustum.hpp"
//optimized pcl transform
#include "transforms.hpp"
#include "dddmr_sys_core/action/p_to_p_move_base.hpp"

#include <pcl/kdtree/kdtree_flann.h>


/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointXYZI  PointType;

class DddmrExploreAndSearch : public rclcpp::Node
{
public:
  DddmrExploreAndSearch();

private:
  
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudKeyPoses6D_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m2ci_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr c2s_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr b2s_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_edge_cloud_pub_;
  

  void m2ci_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void c2s_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void b2s_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void cloudKeyPoses6D_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void syncGroundCb();
  void findExplorationGoalCb();
  
  rclcpp::TimerBase::SharedPtr sync_ground_timer_;
  rclcpp::TimerBase::SharedPtr ground_publish_timer_;
  rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedPtr get_key_frame_cloud_client_;
  rclcpp::CallbackGroup::SharedPtr cbs_group_;
  rclcpp::CallbackGroup::SharedPtr srv_cbs_group_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::vector<std::string> camera_frames_;

  std::map<std::string, CameraFrustum> camera_fov_map_;

  std::string base_frame_;
  pcl::PointCloud<PointType> key_poses_pcl_;
  std::map<int, pcl::PointCloud<PointType>> processed_ground_cloud_;

  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  Eigen::Affine3d trans_m2ci_af3_, trans_c2s_af3_, trans_b2s_af3_;
  tf2::Stamped<tf2::Transform> tf2_trans_m2ci_;
  tf2::Stamped<tf2::Transform> tf2_trans_c2s_; //camera2sensorlink
  tf2::Stamped<tf2::Transform> tf2_trans_b2s_; //baselink2sensor to get ground distance
  bool has_m2ci_;
  bool has_c2s_;
  bool has_b2s_;

  std::vector<pcl::PointCloud<PointType>> key_frame_clouds_;
  std::vector<pcl::PointCloud<PointType>> key_frame_ground_clouds_;
  std::vector<pcl::PointCloud<PointType>> key_frame_ground_edge_clouds_;

  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  
  rclcpp::CallbackGroup::SharedPtr p2p_move_base_client_group_;
  rclcpp_action::Client<dddmr_sys_core::action::PToPMoveBase>::SharedPtr p2p_move_base_client_ptr_;
  void sendP2P(geometry_msgs::msg::PoseStamped m_target_pose);
  void p2p_move_base_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::PToPMoveBase>::SharedPtr & goal_handle);
  void p2p_move_base_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::PToPMoveBase>::WrappedResult & result);
  bool is_goal_reached_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_; 
};

#endif  // DDDMR_EXPLORE_AND_SEARCH_HPP_
