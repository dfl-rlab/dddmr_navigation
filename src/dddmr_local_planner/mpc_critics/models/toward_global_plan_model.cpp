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
#include <mpc_critics/toward_global_plan_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::TowardGlobalPlanModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

TowardGlobalPlanModel::TowardGlobalPlanModel(){
  return;
  
}

void TowardGlobalPlanModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}


double TowardGlobalPlanModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->pcl_prune_plan_->points.size()<3){
    RCLCPP_DEBUG(node_->get_logger().get_child(name_), "TowardGlobalPlanModel: size of prune plan is smaller than 3.");
    //@ if we return negative, the goal behavior might be strange (it is my guess, it should be justified)
    return 10.0;
  }
  
  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPose(traj.getPosesSize()-1);

  pcl::KdTreeFLANN<pcl::PointXYZI> prune_plan_kdtree;
  prune_plan_kdtree.setInputCloud(shared_data_->pcl_prune_plan_);
  int K = 1;
  
  //@ get last traj pose
  pcl::PointXYZI pcl_traj_pose = traj.getPCLPoint(traj.getPosesSize()-1);
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if ( prune_plan_kdtree.nearestKSearch (pcl_traj_pose, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    return sqrt(pointNKNSquaredDistance[0]) * weight_;
  }
  else{
    return -12.0;
  }
  

}

}//end of name space