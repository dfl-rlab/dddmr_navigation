#ifndef LEGO_LOAM_VISUALIZATION_HPP_
#define LEGO_LOAM_VISUALIZATION_HPP_


#include <dddmr_sys_core/srv/get_key_frame_cloud.hpp>
#include "utility.h"
//optimized pcl transform
#include "transforms.hpp"

// omp voxel
#include "dddmr_pcl/voxel_omp/voxel_grid_omp.h"

class LegoLoamVisualization : public rclcpp::Node
{
public:
  LegoLoamVisualization(std::string name);
  Eigen::Affine3d trans_m2ci_af3_;
  bool has_m2ci_;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  void pubMapThread();
  void groundEdgeDetectionThread();
  std::vector<pcl::PointCloud<PointType>::Ptr> key_frame_clouds_;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundEdgeKeyFrames;

private:
  
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudKeyPoses6D_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m2ci_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGround;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGroundEdge;

  void m2ci_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void cloudKeyPoses6D_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void syncMapAndGroundThread();
  pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  pcl::PointCloud<PointType>::Ptr transformPointCloudInverse(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

  rclcpp::TimerBase::SharedPtr sync_map_and_ground_timer_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::TimerBase::SharedPtr timer_ground_edge_detection_;
  rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedPtr get_key_frame_cloud_client_;
  rclcpp::CallbackGroup::SharedPtr cbs_group_;
  
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundEdgeProcessedKeyFrames;
  
  double ground_voxel_size_;
  std::vector<bool> ground_edge_processed_;
  int ground_edge_threshold_num_;

  pcl::VoxelGridOMP downSizeFilterGlobalGroundKeyFrames_Copy_omp;  // for global map visualization
  pcl::VoxelGridOMP ds_patched_ground_omp_;

};

#endif  // LEGO_LOAM_VISUALIZATION_HPP_
