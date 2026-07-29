/*
 * Copyright (c) 2018, the mcl_3dl authors
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

#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>

namespace mcl_3dl
{
void LidarMeasurementModelLikelihood::loadConfig(
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter)
{

  logger_ = m_logger;
  parameter_ = m_parameter;

  parameter_->declare_parameter("likelihood.num_points_global", rclcpp::ParameterValue(0));
  rclcpp::Parameter num_points_global = parameter_->get_parameter("likelihood.num_points_global");
  num_points_global_ = num_points_global.as_int();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.num_points_global: %d", num_points_global_);

  num_points_default_ = num_points_;

  parameter_->declare_parameter("likelihood.threshold_for_trusted_ground", rclcpp::ParameterValue(0));
  rclcpp::Parameter threshold_for_trusted_ground = parameter_->get_parameter("likelihood.threshold_for_trusted_ground");
  threshold_for_trusted_ground_ = threshold_for_trusted_ground.as_int();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.threshold_for_trusted_ground: %d", threshold_for_trusted_ground_);

  parameter_->declare_parameter("likelihood.radius_of_ground_search", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter radius_of_ground_search = parameter_->get_parameter("likelihood.radius_of_ground_search");
  radius_of_ground_search_ = radius_of_ground_search.as_double();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.radius_of_ground_search: %.2f", radius_of_ground_search_);

  parameter_->declare_parameter("likelihood.match_dist_min", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter match_dist_min = parameter_->get_parameter("likelihood.match_dist_min");
  match_dist_min_ = match_dist_min.as_double();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.match_dist_min: %.2f", match_dist_min_);

  parameter_->declare_parameter("likelihood.match_dist_flat", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter match_dist_flat = parameter_->get_parameter("likelihood.match_dist_flat");
  match_dist_flat_ = match_dist_flat.as_double();
  RCLCPP_INFO(logger_->get_logger(), "likelihood.match_dist_flat: %.2f", match_dist_flat_);

}
void LidarMeasurementModelLikelihood::setGlobalLocalizationStatus(
    const int num_particles,
    const int current_num_particles)
{
  if (current_num_particles <= num_particles)
  {
    num_points_ = num_points_default_;
    return;
  }
  int num = num_points_default_ * num_particles / current_num_particles;
  if (num < num_points_global_)
    num = num_points_global_;

  num_points_ = num;
}

LidarMeasurementResult LidarMeasurementModelLikelihood::measure(
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree,
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree_ground,
    pcl::PointCloud<pcl::Normal>& normals,
    std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> pcl_segmentations,
    const State6DOF& s) const
{

  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_flat_new_type(new pcl::PointCloud<mcl_3dl::pcl_t>);
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_sharp_new_type(new pcl::PointCloud<mcl_3dl::pcl_t>);

  float score_like = 0;
  *pc_flat_new_type = (*pcl_segmentations[std::string("flat")]);
  s.transform(*pc_flat_new_type);

  *pc_less_sharp_new_type = (*pcl_segmentations[std::string("less_sharp")]);
  s.transform(*pc_less_sharp_new_type);

  /*Must find a pose stick with ground*/
  mcl_3dl::pcl_t particle_pose;
  particle_pose.x = s.pos_.x_;
  particle_pose.y = s.pos_.y_;
  particle_pose.z = s.pos_.z_;
  float pos_weight = 1.0;

  /*Kd-tree to find nn point for planar equation*/
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  kdtree_ground.radiusSearch(particle_pose, radius_of_ground_search_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  RCLCPP_DEBUG(logger_->get_logger(), "%lu", pointIdxRadiusSearch.size());
  bool is_ground_health = false;
  if(pointIdxRadiusSearch.size()>=threshold_for_trusted_ground_){

    is_ground_health = true;
    float avg_nx, avg_ny, avg_nz;
    avg_nx = avg_ny = avg_nz = 0;
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      avg_nx += normals.points[(*it)].normal_x;
      avg_ny += normals.points[(*it)].normal_y;
      avg_nz += fabs(normals.points[(*it)].normal_z); //@ z might not point to sky but underground 
    }
    //@ Normalize by number of points
    avg_nx/=pointIdxRadiusSearch.size();
    avg_ny/=pointIdxRadiusSearch.size();
    avg_nz/=pointIdxRadiusSearch.size();
    RCLCPP_DEBUG(logger_->get_logger(), "Nx: %f, Ny: %f, Nz: %f", avg_nx, avg_ny, avg_nz);
    //Check the normal of closest point is pointing sky
    if(fabs(avg_nx)>=3.*fabs(avg_nz) || fabs(avg_ny)>=3.*fabs(avg_nz) )
      pos_weight = 0.2;//maybe better than not even found?so set it larger than nothing found
    else{

      /*Ready to examine roll and pitch*/
      tf2::Vector3 axis_vector(avg_nx, avg_ny, avg_nz);

      tf2::Vector3 up_vector(0.0, 0.0, 1.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      right_vector.normalized();
      tf2::Quaternion q_normal(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
      q_normal.normalize();

      tf2::Quaternion q_pose(s.rot_.x_, s.rot_.y_, s.rot_.z_, s.rot_.w_);
      tf2::Quaternion q_new;

      q_new = q_pose*q_normal;  // Calculate the new orientation
      q_new.normalize();

      /*transform to RPY for weighting*/
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_new).getRPY(roll, pitch, yaw);
      RCLCPP_DEBUG(logger_->get_logger(), "R: %f, P: %f, Y: %f", roll, pitch, yaw);
      double roll_diff = 0;
      if(fabs(roll)>2.6 && fabs(roll)<3.1415926)
        roll_diff = 3.1415926-fabs(roll);
      else if(fabs(roll)>=0 and fabs(roll)<0.5)
        roll_diff = fabs(roll);
      else
        roll_diff = 0.55;
      RCLCPP_DEBUG(logger_->get_logger(), "roll diff: %f", roll_diff);
      /*better score when roll_diff close to zero*/

      /*kdtree ground --> Now find the point on the ground to make sure the pose is stick with ground*/
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      if(kdtree_ground.nearestKSearch(particle_pose, 1, pointIdxNKNSearch, pointNKNSquaredDistance))
        pos_weight = (1.0-std::sqrt(pointNKNSquaredDistance[0])) * (1-roll_diff);
      else{
        RCLCPP_ERROR(logger_->get_logger(), "mcl lidar_measurement_model_likelihood.cpp: not possible!!!");
        pos_weight = 0.1;
      }
      if(pos_weight<0)
        pos_weight = 0.01;
    }
  }
  else{
    RCLCPP_DEBUG(logger_->get_logger(), 
        "Not enough ground information: %lu, expected at least: %d, @ location: %.2f, %.2f, %.2f, ignore ground weighting mechanism.", 
        pointIdxRadiusSearch.size(), threshold_for_trusted_ground_, particle_pose.x, particle_pose.y, particle_pose.z);
    /*kdtree --> Now find the point on the ground to make sure the pose is stick with ground*/
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if(kdtree.nearestKSearch(particle_pose, 1, pointIdxNKNSearch, pointNKNSquaredDistance))
      pos_weight = (1.0-std::sqrt(pointNKNSquaredDistance[0]));
    else
      pos_weight = 0.01;
    if(pos_weight<0)
      pos_weight = 0.01;
  }

  size_t num = 0;
  std::vector<int> id(1);
  std::vector<float> sqdist(1);

  for (auto& p : pc_flat_new_type->points)
  {
    if(is_ground_health){
      //kdtree_ground.radiusSearch(p, match_dist_min_, id, sqdist, 1);
      kdtree_ground.nearestKSearch(p, 1, id, sqdist);
    }
    else{
      //kdtree.radiusSearch(p, match_dist_min_, id, sqdist, 1);
      kdtree_ground.nearestKSearch(p, 1, id, sqdist);
    }

    if (sqdist.size()>0)
    {
      float dist = match_dist_min_ - std::sqrt(sqdist[0]);
      if (dist < 0.0)
        dist = 0.001;

      score_like += dist * dist;
      num++;
    }
  }
  /*
  id.clear();
  sqdist.clear();
  for (auto& p : pc_less_sharp_new_type->points)
  {
    if (kdtree->radiusSearch(p, match_dist_min_, id, sqdist, 1))
    {
      const float dist = match_dist_min_ - std::max(std::sqrt(sqdist[0]), match_dist_flat_);
      if (dist < 0.0)
        continue;

      score_like += dist * dist;
      num++;
    }
  }
  */
  id.clear();
  sqdist.clear();
  for (auto& p : pc_less_sharp_new_type->points)
  {
    float pt_segmentation_weight = p.intensity;

    //kdtree.radiusSearch(p, match_dist_min_, id, sqdist, 1)
    kdtree.nearestKSearch(p, 1, id, sqdist);
    float dist = match_dist_min_ - std::sqrt(sqdist[0]);
    if (dist < 0.0)
      dist = 0.001;

    score_like += dist * dist / pt_segmentation_weight;
    num++;
    
  }
  const float match_ratio = static_cast<float>(num) / (pc_flat_new_type->points.size() + pc_less_sharp_new_type->points.size());

  return LidarMeasurementResult(score_like*pos_weight, match_ratio);
}
}  // namespace mcl_3dl
