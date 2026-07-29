/*
 * Copyright (c) 2016-2018, the mcl_3dl authors
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
#ifndef MCL_3DL_SUBMAPS_H
#define MCL_3DL_SUBMAPS_H

#include "utilities.h"

#include <mutex>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"

/*For ros msg*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <dddmr_sys_core/srv/get_key_frame_cloud.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

// This is for euclidean distance segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

// RANSAC
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

// omp voxel
#include "dddmr_pcl/voxel_omp/voxel_grid_omp.h"

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

namespace mcl_3dl
{
class SubMaps  : public rclcpp::Node 
{

private:
  
  void keyPosesCb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void syncMapThread();

  std::string pg_map_server_name_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr srv_group_;
  rclcpp::TimerBase::SharedPtr warm_up_timer_;
  rclcpp::TimerBase::SharedPtr sync_map_timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_map_warmup_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sub_ground_warmup_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_key_poses_;
  rclcpp::Client<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedPtr get_key_frame_cloud_client_;

  geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped warm_up_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_sub_map_pose_;
  
  std::vector<pcl::PointCloud<pcl_t>::Ptr> cornerCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<pcl_t>::Ptr> cornerCloudKeyFrames_baselink_; //original frame from files are base_link
  std::vector<pcl::PointCloud<pcl_t>::Ptr> surfCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<pcl_t>::Ptr> surfCloudKeyFrames_baselink_; //original frame from files are base_link
  std::vector<pcl::PointCloud<pcl_t>::Ptr> groundCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<pcl_t>::Ptr> groundCloudKeyFrames_baselink_; //original frame from files are base_link
  
  //@
  bool is_initial_;
  bool is_current_ready_;
  bool key_poses_received_;
  pcl::PointCloud<pcl_t>::Ptr poses_pcl_t_;
  pcl::KdTreeFLANN<mcl_3dl::pcl_t>::Ptr kdtree_poses_;

  pcl::PointCloud<pcl_t>::Ptr map_warmup_;
  pcl::PointCloud<pcl_t>::Ptr ground_warmup_;

  //@
  bool prepare_warm_up_;
  bool is_warm_up_ready_;
  double sub_map_search_radius_;
  double sub_map_warmup_trigger_distance_;
  
  pcl::VoxelGridOMP map_voxel_omp_;
  pcl::VoxelGridOMP ground_voxel_omp_;

public:

  SubMaps(std::string name);
  ~SubMaps();

  pcl::PointCloud<pcl_t>::Ptr map_current_;
  pcl::PointCloud<pcl_t>::Ptr ground_current_;
  
  void setPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose);
  void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose);
  bool isWarmUpReady();
  void swapKdTree();
  
  void warmUpThread();
  bool isCurrentReady(){return is_current_ready_;};

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex sub_maps_mutex_t;
  sub_maps_mutex_t * getMutex()
  {
    return access_;
  }
  sub_maps_mutex_t * access_;

  pcl::KdTreeFLANN<mcl_3dl::pcl_t> kdtree_map_current_;
  pcl::KdTreeFLANN<mcl_3dl::pcl_t> kdtree_ground_current_;
  pcl::PointCloud<pcl::Normal> normals_ground_current_;

  pcl::KdTreeFLANN<mcl_3dl::pcl_t> kdtree_map_warmup_;
  pcl::KdTreeFLANN<mcl_3dl::pcl_t> kdtree_ground_warmup_;
  pcl::PointCloud<pcl::Normal> normals_ground_warmup_;
  
  int knn_num_of_ground_normals_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_SUBMAPS_H
