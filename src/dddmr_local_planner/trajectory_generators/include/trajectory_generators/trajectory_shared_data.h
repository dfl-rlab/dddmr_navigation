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
#ifndef TRAJECTORY_SHARED_DATA_H_
#define TRAJECTORY_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*TF listener*/
/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

/*For robot state*/
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

/*Stamped cmd*/
#include "geometry_msgs/msg/twist_stamped.hpp"

/*tf to affine*/
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>

#include <math.h>

namespace trajectory_generators
{

class TrajectoryGeneratorSharedData{
  public:

    TrajectoryGeneratorSharedData(std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer):
    tf2Buffer_(m_tf2Buffer),
    current_allowed_max_linear_speed_(-1.0)
    {};
    
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer(){return tf2Buffer_;}

    geometry_msgs::msg::TransformStamped robot_pose_;
    geometry_msgs::msg::TransformStamped prune_end_pose_base_link_frame_;
    nav_msgs::msg::Odometry robot_state_;
    ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_state_;
    nav_msgs::msg::Path prune_plan_;
    double current_allowed_max_linear_speed_;
    geometry_msgs::msg::TwistStamped ref_twist_for_trajectory_generation_;
    
    std::string global_frame_, base_frame_;


    void updateGoalatRobotFrame(){

      geometry_msgs::msg::TransformStamped pose_prune_end;
      pose_prune_end.header = prune_plan_.header;
      pose_prune_end.transform.translation.x = prune_plan_.poses.back().pose.position.x;
      pose_prune_end.transform.translation.y = prune_plan_.poses.back().pose.position.y;
      pose_prune_end.transform.translation.z = prune_plan_.poses.back().pose.position.z;

      pose_prune_end.transform.rotation.x = prune_plan_.poses.back().pose.orientation.x;
      pose_prune_end.transform.rotation.y = prune_plan_.poses.back().pose.orientation.y;
      pose_prune_end.transform.rotation.z = prune_plan_.poses.back().pose.orientation.z;
      pose_prune_end.transform.rotation.w = prune_plan_.poses.back().pose.orientation.w;

      Eigen::Affine3d pruneend2map_af3 = tf2::transformToEigen(pose_prune_end).inverse();
      Eigen::Affine3d map2baselink_af3 = tf2::transformToEigen(robot_pose_);

      Eigen::Affine3d pruneend2baselink_af3 = pruneend2map_af3*map2baselink_af3;
      Eigen::Affine3d baselink2pruneend_af3 = pruneend2baselink_af3.inverse();

      prune_end_pose_base_link_frame_ = tf2::eigenToTransform (baselink2pruneend_af3);

    }
  
  private:

    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
    

};


}//end of name space

#endif