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

#ifndef DDDMR_PG_MAP_SERVER_H_
#define DDDMR_PG_MAP_SERVER_H_

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

#include "dddmr_sys_core/srv/get_key_frame_cloud.hpp"

// omp voxel
#include "dddmr_pcl/voxel_omp/voxel_grid_omp.h"

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

namespace dddmr_pg_map_server
{
class DDDMRPGMapServer  : public rclcpp::Node 
{

private:

  std::string pose_graph_dir_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Service<dddmr_sys_core::srv::GetKeyFrameCloud>::SharedPtr srv_get_key_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_key_pose_arr_;

  pcl::PointCloud<PointTypePose>::Ptr pcd_poses_; //original poses from files
  pcl::PointCloud<PointTypePose> poses_; //Operating poses, we maitain this poses all the time
  std::vector<pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr> cornerCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr> cornerCloudKeyFrames_baselink_; //original frame from files are base_link
  std::vector<pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr> surfCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr> surfCloudKeyFrames_baselink_; //original frame from files are base_link
  std::vector<pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr> groundCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<dddmr_pg_map_server::pcl_t>::Ptr> groundCloudKeyFrames_baselink_; //original frame from files are base_link


  void readPoseGraph();

  void getKeyFrameCloud(
      const std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Request>
          request,
      std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Response>
          response);

  float complete_map_voxel_size_;
  geometry_msgs::msg::PoseArray key_poses_;
  
public:

  DDDMRPGMapServer(std::string name);
  ~DDDMRPGMapServer();

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex sub_maps_mutex_t;
  sub_maps_mutex_t * getMutex()
  {
    return access_;
  }
  sub_maps_mutex_t * access_;

};
}  // namespace dddmr_pg_map_server

#endif  // DDDMR_PG_MAP_SERVER_H_
