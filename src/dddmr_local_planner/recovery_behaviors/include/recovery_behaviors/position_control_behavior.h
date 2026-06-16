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
#include <recovery_behaviors/robot_behavior.h>

/*For visualization*/
#include "geometry_msgs/msg/pose_array.hpp"

namespace recovery_behaviors
{

enum EnumFSMStatePositionControl {
  POINTING_ORIENTATION_TO_TARGET,
  STRAIGHT,
  ALIGN_ORIENTATION_FROM_GOAL
};
  
class PositionControlBehavior: public RobotBehavior{


  public:
    
    PositionControlBehavior();
    ~PositionControlBehavior();

  private:

    void trans2Pose(geometry_msgs::msg::TransformStamped& trans, geometry_msgs::msg::PoseStamped& pose);
    double frequency_;
    
    double yaw_tolerance_;
    double distance_tolerance_;

    pcl::PointCloud<pcl::PointXYZ> cuboid_;
    void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr, pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl);

    void getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);

    dddmr_sys_core::RecoveryState runBehavior(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle);
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
    
    rclcpp::Clock::SharedPtr clock_;
    std::string trajectory_generator_name_;
    
    void pubZeroVelocity();
    double getYaw(geometry_msgs::msg::PoseStamped& pose);
    
    double kp_yaw_, ki_yaw_, kd_yaw_;
    double kp_distance_, ki_distance_, kd_distance_;

    double yaw_error_, yaw_error_i_;
    double distance_error_, distance_error_i_;
    
    double last_yaw_diff_, last_distance_diff_;
    
    geometry_msgs::msg::TwistStamped cmd_vel_stamped_;

    void getDiffFromCurrentPoseToTargetPosition(
              geometry_msgs::msg::PoseStamped& first_pose, 
              geometry_msgs::msg::PoseStamped& last_pose,
              double& yaw, double& distance);

    void getYawAndDistanceDiffFromTwoPoses(
            tf2::Stamped<tf2::Transform>& tf2_trans_pin_pose, 
            tf2::Stamped<tf2::Transform>& tf2_trans_target_pose,
            double& yaw_diff, double& distance_diff);

    void generateVelocity(double vx, double vy, double angular_z);
    
    void rosMsg2Tf2Msg(const geometry_msgs::msg::PoseStamped& ros_pose, tf2::Stamped<tf2::Transform>& tf2_pose);

    recovery_behaviors::EnumFSMStatePositionControl fsm_state_;
    
    int yaw_converge_count_, distance_converge_count_, yaw_goal_converge_count_;
  
  protected:

    virtual void onInitialize();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_observation_;
    
    std::string robot_frame_, global_frame_;
    
    std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;


};

}//end of name space
