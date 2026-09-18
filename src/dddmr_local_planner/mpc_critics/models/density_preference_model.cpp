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
#include <mpc_critics/density_preference_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::DensityPreferenceModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

DensityPreferenceModel::DensityPreferenceModel(){
  density_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  density_cloud_kd_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  return;
  
}

void DensityPreferenceModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  node_->declare_parameter(name_ + ".topic", rclcpp::ParameterValue("density_cloud"));
  node_->get_parameter(name_ + ".topic", topic_name_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "topic: %s", topic_name_.c_str());

  density_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name_, 2, std::bind(&DensityPreferenceModel::densityCloudCallback, this, std::placeholders::_1));
}

void DensityPreferenceModel::densityCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  
  std::lock_guard<std::mutex> lock(shared_data_->scoring_mutex_);
  //@ density cloud should be in global frame
  pcl::fromROSMsg(*msg, *density_cloud_);
  std::vector<int> indices;
  density_cloud_->is_dense = false;
  pcl::removeNaNFromPointCloud(*density_cloud_, *density_cloud_, indices);
  density_cloud_kd_tree_->setInputCloud(density_cloud_);
  
  //RCLCPP_INFO(node_->get_logger().get_child(name_), "Density cloud size: %lu", density_cloud_->points.size());
}

double DensityPreferenceModel::scoreTrajectory(base_trajectory::Trajectory &traj){
  
  std::lock_guard<std::mutex> lock(shared_data_->scoring_mutex_);

  if(shared_data_->prune_plan_.poses.empty() || traj.getPosesSize()<2){
    return -5.0;  
  }
  
  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPose(traj.getPosesSize()-1);
  pcl::PointXYZ last_traj_point;
  last_traj_point.x = last_traj_pose.pose.position.x;
  last_traj_point.y = last_traj_pose.pose.position.y;
  last_traj_point.z = last_traj_pose.pose.position.z;
  float distance_from_last_traj2centroid = 0.0;

  if(density_cloud_->points.size()>5){
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    density_cloud_kd_tree_->radiusSearch(last_traj_point, 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if(!pointIdxRadiusSearch.empty()){
      //@calculate centroid of the cloud
      pcl::PointXYZ centroid;
      for(const auto& index : pointIdxRadiusSearch){
        centroid.x += density_cloud_->points[index].x;
        centroid.y += density_cloud_->points[index].y;
        centroid.z += density_cloud_->points[index].z;
      }
      centroid.x /= pointIdxRadiusSearch.size();
      centroid.y /= pointIdxRadiusSearch.size();
      centroid.z /= pointIdxRadiusSearch.size();
      float dx = last_traj_point.x - centroid.x;
      float dy = last_traj_point.y - centroid.y;
      float dz = last_traj_point.z - centroid.z;
      distance_from_last_traj2centroid = 1/sqrt(dx*dx+dy*dy+dz*dz);
    }
    else{
      //@ no neighbor is found.
      //@ the long trajectory
      RCLCPP_INFO(node_->get_logger().get_child(name_), "No neighbor is found");
      distance_from_last_traj2centroid = -1.0;
    }
  }
  else{
    pcl::PointXYZ centroid;
    for(const auto& pt : density_cloud_->points){
      centroid.x += pt.x;
      centroid.y += pt.y;
      centroid.z += pt.z;
    }
    centroid.x /= density_cloud_->points.size();
    centroid.y /= density_cloud_->points.size();
    centroid.z /= density_cloud_->points.size();
    float dx = last_traj_point.x - centroid.x;
    float dy = last_traj_point.y - centroid.y;
    float dz = last_traj_point.z - centroid.z;
    distance_from_last_traj2centroid = 1/sqrt(dx*dx+dy*dy+dz*dz);
  }
  
  //@ expert scoring say minimum score is better and higher than 0

  return weight_ * distance_from_last_traj2centroid;
}

}//end of name space
