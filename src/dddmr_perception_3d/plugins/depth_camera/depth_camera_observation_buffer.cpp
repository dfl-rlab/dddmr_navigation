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
#include <rclcpp/rclcpp.hpp>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>

using namespace std::chrono_literals;
namespace perception_3d
{

DepthCameraObservationBuffer::DepthCameraObservationBuffer(
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
      double observation_persistence)
: topic_name_(topic_name),
  tf2Buffer_(tf2Buffer),
  clock_(clock),
  global_frame_(global_frame),
  robot_frame_(robot_frame),
  sensor_frame_(sensor_frame),
  min_detect_distance_(min_detect_distance),
  max_detect_distance_(max_detect_distance),
  min_obstacle_height_(min_obstacle_height),
  max_obstacle_height_(max_obstacle_height),
  FOV_W_(FOV_W), FOV_V_(FOV_V),
  expected_update_rate_(expected_update_rate),
  observation_persistence_(observation_persistence),
  logger_(logger)
{
  first_scan_received_ = false;
  resetLastUpdated();
  got_b2s_ = false;
}

DepthCameraObservationBuffer::~DepthCameraObservationBuffer()
{
}

void DepthCameraObservationBuffer::bufferCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  
  std::unique_lock<std::mutex> lock(lock_);

  observation_vector_.push_back(DepthCameraObservation(cloud));

  std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  if(sensor_frame_ == "")
  {
    RCLCPP_WARN_STREAM(logger_,"Warning: Sensor frame is not provided in yaml file. Using pointcloud header frame id");
  }

  try
  {
    if(!got_b2s_){
      b2s_ = tf2Buffer_->lookupTransform(robot_frame_, cloud.header.frame_id , tf2::TimePointZero, tf2::durationFromSec(0.5));
      got_b2s_ = true;
    }
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(logger_, "Failed to transform pointcloud to %s frame: %s", robot_frame_.c_str(), e.what());
  }

  //@ get af3 to convert observation to baselink frame
  Eigen::Affine3d trans_b2s_af3 = tf2::transformToEigen(b2s_);
  //@ raw_cloud is XYZ, convert it to XYZI so in the future we can extend features leverage intensity
  pcl::transformPointCloud(*observation_vector_.back().raw_cloud_, *observation_vector_.back().raw_cloud_, trans_b2s_af3);
  
  for (auto it=observation_vector_.back().raw_cloud_->points.begin();it!=observation_vector_.back().raw_cloud_->points.end();it++)
  {
    //@ super near filter, basically, filter out 0,0.0 point
    if ((*it).z <= max_obstacle_height_ && (*it).z >= min_obstacle_height_)
    {
      pcl::PointXYZI tmp_pt;
      tmp_pt.x = (*it).x;
      tmp_pt.y = (*it).y;
      tmp_pt.z = (*it).z;
      tmp_pt.intensity = 0;
      observation_vector_.back().cloud_->push_back(tmp_pt);
      observation_vector_.back().raw_cloud_i_->push_back(tmp_pt);
    }
  }

  //@ Check the cloud size 
  long unsigned int cloud_size_after_min_max_obstacle = observation_vector_.back().cloud_->size();
  if(cloud_size_after_min_max_obstacle >20000)
  {
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (observation_vector_.back().cloud_);
    sor.setLeafSize (0.05, 0.05, 0.05);
    sor.filter (*observation_vector_.back().cloud_);
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Point cloud size filtered from min-max obstacle height is: %lu, voxelize it to: %lu", cloud_size_after_min_max_obstacle, observation_vector_.back().cloud_->points.size());
  }

  //@ given these observations come from sensors... we'll need to store the origin pt of the sensor, i.e.: camera_link
  geometry_msgs::msg::TransformStamped m2s; //map2sensor for frustum
  geometry_msgs::msg::TransformStamped m2b; //map2baselink for converting b2s to m2s
  
  try
  {
    m2s = tf2Buffer_->lookupTransform(global_frame_, origin_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));
    m2b = tf2Buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.2));
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(logger_, "Failed to transform pointcloud: %s", e.what());
  }

  observation_vector_.back().origin_.x = m2s.transform.translation.x;
  observation_vector_.back().origin_.y = m2s.transform.translation.y;
  observation_vector_.back().origin_.z = m2s.transform.translation.z;

  //@ Update camera parameters
  observation_vector_.back().min_detect_distance_ = min_detect_distance_;
  observation_vector_.back().max_detect_distance_ = max_detect_distance_;
  observation_vector_.back().FOV_W_ = FOV_W_;
  observation_vector_.back().FOV_V_ = FOV_V_;

  //@ Find frustum vertex (8 points) and transform it to global.
  //@ !!! Frustum vertex is usually based on camera_link frame (realsense).
  observation_vector_.back().findFrustumVertex();

  pcl_conversions::toPCL(cloud.header.stamp, observation_vector_.back().frustum_->header.stamp);
  observation_vector_.back().frustum_->header.frame_id = origin_frame;

  //@ get af3 to convert frustum to globl frame
  Eigen::Affine3d trans_m2s_af3 = tf2::transformToEigen(m2s);
  pcl::transformPointCloud(*observation_vector_.back().frustum_, *observation_vector_.back().frustum_, trans_m2s_af3);

  observation_vector_.back().frustum_->header.frame_id = global_frame_;
  
  //@ Find frustum normal and plane, note that the planes/normals are in global frame
  //@ !!! findFrustumNormal() will assign BRNear_&&TLFar_  which are both in global frame
  observation_vector_.back().findFrustumNormal();
  observation_vector_.back().findFrustumPlane();

  //@ Convert cloud_ from robot_frame_ to global frame
  Eigen::Affine3d trans_m2b_af3 = tf2::transformToEigen(m2b);
  pcl::transformPointCloud(*observation_vector_.back().cloud_, *observation_vector_.back().cloud_, trans_m2b_af3);
  pcl_conversions::toPCL(clock_->now(), observation_vector_.back().cloud_->header.stamp);
  observation_vector_.back().cloud_->header.frame_id = global_frame_;

  //@ if the update was successful, we want to update the last updated time
  resetLastUpdated();
  //@ we'll also remove any stale observations from the vector
  purgeStaleObservations();
  first_scan_received_ = true;
}

// returns a copy of the observations
void DepthCameraObservationBuffer::getObservations(std::vector<perception_3d::DepthCameraObservation>& observations)
{
  std::unique_lock<std::mutex> lock(lock_);
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();
  // now we'll just copy the observations for the caller
  for (auto obs_it = observation_vector_.begin(); obs_it != observation_vector_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }

}

void DepthCameraObservationBuffer::purgeStaleObservations()
{

  if (!observation_vector_.empty())
  {
    // if we're keeping observations for no time... then we'll only keep one observation
    if (rclcpp::Duration::from_seconds(observation_persistence_) == rclcpp::Duration::from_seconds(0.0))
    {
      auto last_observation = observation_vector_.back();
      observation_vector_.clear();
      observation_vector_.push_back(last_observation);
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (auto obs_it = observation_vector_.begin(); obs_it != observation_vector_.end();)
    {
      perception_3d::DepthCameraObservation& obs = *obs_it;
      const rclcpp::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > rclcpp::Duration::from_seconds(observation_persistence_))
      {
        observation_vector_.erase(obs_it++);
      }
      else{
        ++obs_it;
      }
    }
  }
}

bool DepthCameraObservationBuffer::isCurrent() const
{
  
  if (rclcpp::Duration::from_seconds(expected_update_rate_) == rclcpp::Duration::from_seconds(0.0))
    return true;

  const rclcpp::Duration update_time = clock_->now() - last_updated_;
  bool current = update_time.seconds() <= expected_update_rate_;

  if (!current)
  {
    RCLCPP_WARN(logger_, "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
      topic_name_.c_str(), update_time.seconds(), expected_update_rate_);
  }

  return current;
}

void DepthCameraObservationBuffer::resetLastUpdated()
{
  last_updated_ = clock_->now();
}


}