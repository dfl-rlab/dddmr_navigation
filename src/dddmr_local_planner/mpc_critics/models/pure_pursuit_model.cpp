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
#include <mpc_critics/pure_pursuit_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::PurePursuitModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

PurePursuitModel::PurePursuitModel(){
  return;
  
}

void PurePursuitModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  node_->declare_parameter(name_ + ".translation_weight", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".translation_weight", translation_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "translation_weight: %.2f", translation_weight_);

  node_->declare_parameter(name_ + ".orientation_weight", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".orientation_weight", orientation_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "orientation_weight: %.2f", orientation_weight_);

}


double PurePursuitModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->prune_plan_.poses.empty() || traj.getPosesSize()<2){
    return -4.0;  
  }

  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPose(traj.getPosesSize()-1);
  geometry_msgs::msg::PoseStamped last_prune_plan_pose = shared_data_->prune_plan_.poses.back();

  //@ create tf pose for affine computation
  geometry_msgs::msg::TransformStamped tf_last_traj_pose, tf_last_prune_plan_pose;
  tf_last_traj_pose.header.frame_id = shared_data_->global_frame_;
  tf_last_traj_pose.child_frame_id = shared_data_->base_frame_;
  tf_last_traj_pose.transform.translation.x = last_traj_pose.pose.position.x;
  tf_last_traj_pose.transform.translation.y = last_traj_pose.pose.position.y;
  tf_last_traj_pose.transform.translation.z = last_traj_pose.pose.position.z;
  tf_last_traj_pose.transform.rotation = last_traj_pose.pose.orientation;
  //@ tf of last_prune_plan_pose
  tf_last_prune_plan_pose.header.frame_id = shared_data_->global_frame_;
  tf_last_prune_plan_pose.child_frame_id = shared_data_->base_frame_;
  tf_last_prune_plan_pose.transform.translation.x = last_prune_plan_pose.pose.position.x;
  tf_last_prune_plan_pose.transform.translation.y = last_prune_plan_pose.pose.position.y;
  tf_last_prune_plan_pose.transform.translation.z = last_prune_plan_pose.pose.position.z;
  tf_last_prune_plan_pose.transform.rotation = last_prune_plan_pose.pose.orientation;  

  //tf_last_traj_pose
  Eigen::Affine3d last_traj_pose_af3 = tf2::transformToEigen(tf_last_traj_pose);
  //@ inverse last_traj_pose_af3 to get last_traj_pose->global
  last_traj_pose_af3 = last_traj_pose_af3.inverse();
  //@ we have global->last_prune_plan_pose
  Eigen::Affine3d last_prune_plan_pose_af3 = tf2::transformToEigen(tf_last_prune_plan_pose);

  //@ So we have last_traj_pose to last_prune_plan_pose
  Eigen::Affine3d pose_difference_af3 = last_traj_pose_af3*last_prune_plan_pose_af3;
  geometry_msgs::msg::TransformStamped tf_pose_difference = tf2::eigenToTransform(pose_difference_af3);

  double r,y,p;
  tf2::Quaternion q;
  tf2::convert(tf_pose_difference.transform.rotation , q);
  tf2::Matrix3x3(q).getEulerYPR(y,p,r);

  y = std::fmod((y+3.1416),3.1416);
  //RCLCPP_DEBUG(node_->get_logger().get_child(name_), "yaw: %f",y);
  double distance = sqrt(tf_pose_difference.transform.translation.x*tf_pose_difference.transform.translation.x+
                        tf_pose_difference.transform.translation.y*tf_pose_difference.transform.translation.y+
                        tf_pose_difference.transform.translation.z*tf_pose_difference.transform.translation.z);
  /*
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Roll: %f, Pitch: %f, Yaw: %f",r,p,y);
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "trans: %f,%f,%f", pose_difference.translation().x(), pose_difference.translation().y(), pose_difference.translation().z());
  RCLCPP_STREAM(node_->get_logger().get_child(name_), "Affine: " << pose_difference.rotation());
  */
  //RCLCPP_INFO(node_->get_logger().get_child(name_), "trans: %f,%f,%f", tf_pose_difference.transform.translation.x, tf_pose_difference.transform.translation.y, tf_pose_difference.transform.translation.z);
  //@ normalized translation vs rotation
  return (translation_weight_*distance + orientation_weight_*y);
}

}//end of name space