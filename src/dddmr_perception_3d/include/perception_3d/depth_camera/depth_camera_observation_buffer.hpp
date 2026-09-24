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
#ifndef DEPTH_CAMERA_OBSERVATION_BUFFER_H_
#define DEPTH_CAMERA_OBSERVATION_BUFFER_H_

// STL
#include <vector>
#include <string>
#include <chrono>
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Observation
#include <perception_3d/depth_camera/depth_camera_observation.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/filters/voxel_grid.h>

// This is for euclidean distance segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

namespace perception_3d
{

class DepthCameraObservationBuffer
{
public:

  DepthCameraObservationBuffer(
      std::string topic_name,
      const std::shared_ptr<tf2_ros::Buffer>& tf2Buffer,
      rclcpp::Logger logger,
      const rclcpp::Clock::SharedPtr& clock,
      std::string global_frame,
      std::string robot_frame,
      std::string sensor_frame,
      double min_detect_distance,
      double max_detect_distance,
      double min_obstacle_height,
      double max_obstacle_height,
      double FOV_W,
      double FOV_V,
      double expected_update_rate,
      double observation_persistence);


  ~DepthCameraObservationBuffer();

  void bufferCloud(const sensor_msgs::msg::PointCloud2& cloud);

  void getObservations(std::vector<perception_3d::DepthCameraObservation>& observations);

  bool isCurrent() const;

  void resetLastUpdated();
  
  std::string getGlobalFrame(){return global_frame_;};
  
  bool isFirstScanReady(){return first_scan_received_;};
  
private:
  
  void purgeStaleObservations();
  void cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
  double observation_persistence_;
  double expected_update_rate_;
  rclcpp::Time last_updated_;
  std::string robot_frame_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::string topic_name_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double obstacle_range_;
  double FOV_V_;
  double FOV_W_;
  double min_detect_distance_;
  double max_detect_distance_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;


  std::vector<perception_3d::DepthCameraObservation> observation_vector_;
  std::mutex lock_;  ///< @brief A lock for accessing data in callbacks safely

  /// Adaptive height change
  rclcpp::Time adapt_height_cout_prev_time_;
  unsigned long adpat_height_cout_time_nsec_ = 10e9;
  bool adapt_height_init_ = false;
  bool got_b2s_;
  geometry_msgs::msg::TransformStamped b2s_; //baselink2sensor
  bool first_scan_received_;

};
}
#endif  // DEPTH_CAMERA_OBSERVATION_BUFFER_H_