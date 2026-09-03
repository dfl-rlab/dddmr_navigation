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
#include <mpc_critics/target_ranging_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::TargetRangingModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

TargetRangingModel::TargetRangingModel(){
  return;
  
}

void TargetRangingModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

  node_->declare_parameter(name_ + ".topic", rclcpp::ParameterValue("target_pose"));
  node_->get_parameter(name_ + ".topic", topic_name_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "topic: %s", topic_name_.c_str());

  target_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_name_, 2, std::bind(&TargetRangingModel::targetPoseCallback, this, std::placeholders::_1));

  target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    name_ + "/target_ranging_tracking_pose", 2);

}

void TargetRangingModel::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  target_pose_ = *msg;
  if(!got_tf_){
    try
    {
      target_to_base_tf_ = shared_data_->tf2Buffer()->lookupTransform(
        shared_data_->base_frame_,
        msg->header.frame_id,
        tf2::TimePointZero);
      got_tf_ = true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(node_->get_logger().get_child(name_),
        "Failed to lookup transform from %s to %s: %s",
        msg->header.frame_id.c_str(), shared_data_->base_frame_.c_str(), ex.what());
    }
  }
  //@ compute transform from global frame to target_pose frame using robot_pose_
  Eigen::Affine3d tf_gbl2b_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  Eigen::Affine3d tf_b2target_af3 = tf2::transformToEigen(target_to_base_tf_);
  Eigen::Affine3d tf_gbl2target_af3 = tf_gbl2b_af3 * tf_b2target_af3;
  geometry_msgs::msg::TransformStamped tf_gbl2target = tf2::eigenToTransform(tf_gbl2target_af3);
  tf_gbl2target.header.frame_id = shared_data_->global_frame_;
  tf_gbl2target.child_frame_id = target_pose_.header.frame_id;

  tf2::doTransform(target_pose_, target_pose_, tf_gbl2target);
  target_pose_pub_->publish(target_pose_);
}

double TargetRangingModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->prune_plan_.poses.empty() || traj.getPosesSize()<2){
    return -5.0;  
  }
  
  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPose(traj.getPosesSize()-1);

  //@ create tf pose for affine computation
  geometry_msgs::msg::TransformStamped tf_last_traj_pose;
  tf_last_traj_pose.header.frame_id = shared_data_->global_frame_;
  tf_last_traj_pose.child_frame_id = shared_data_->base_frame_;
  tf_last_traj_pose.transform.translation.x = last_traj_pose.pose.position.x;
  tf_last_traj_pose.transform.translation.y = last_traj_pose.pose.position.y;
  tf_last_traj_pose.transform.translation.z = last_traj_pose.pose.position.z;
  tf_last_traj_pose.transform.rotation = last_traj_pose.pose.orientation;


  return weight_;
}

}//end of name space
