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
#include "p2p_move_base/p2p_state.h"

namespace p2p_move_base
{

State::State(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
              const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter)
{
  logger_ = m_logger;
  parameter_ = m_parameter;
  current_decision_ = "d_initial",
  last_decision_ = "d_initial",

  //Planning related
  parameter_->declare_parameter("planner_patience", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter planner_patience = parameter_->get_parameter("planner_patience");
  planner_patience_ = planner_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "planner_patience: %.2f", planner_patience_);  

  //Controlling related
  parameter_->declare_parameter("oscillation_distance", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter oscillation_distance = parameter_->get_parameter("oscillation_distance");
  oscillation_angle_ = oscillation_distance.as_double();
  RCLCPP_INFO(logger_->get_logger(), "oscillation_distance: %.2f", oscillation_angle_);  

  parameter_->declare_parameter("oscillation_angle", rclcpp::ParameterValue(0.5));
  rclcpp::Parameter oscillation_angle = parameter_->get_parameter("oscillation_angle");
  oscillation_angle_ = oscillation_angle.as_double();
  RCLCPP_INFO(logger_->get_logger(), "oscillation_angle: %.2f", oscillation_angle_);  

  parameter_->declare_parameter("oscillation_patience", rclcpp::ParameterValue(20.0));
  rclcpp::Parameter oscillation_patience = parameter_->get_parameter("oscillation_patience");
  oscillation_patience_ = oscillation_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "oscillation_patience: %.2f", oscillation_patience_);  

  parameter_->declare_parameter("controller_patience", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter controller_patience = parameter_->get_parameter("controller_patience");
  controller_patience_ = controller_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "controller_patience: %.2f", controller_patience_);  

  parameter_->declare_parameter("no_plan_retry_num", rclcpp::ParameterValue(10));
  rclcpp::Parameter no_plan_retry_num = parameter_->get_parameter("no_plan_retry_num");
  no_plan_retry_num_ = no_plan_retry_num.as_int();
  RCLCPP_INFO(logger_->get_logger(), "no_plan_retry_num: %d", no_plan_retry_num_);  

  parameter_->declare_parameter("waiting_patience", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter waiting_patience = parameter_->get_parameter("waiting_patience");
  waiting_patience_ = waiting_patience.as_double();
  RCLCPP_INFO(logger_->get_logger(), "waiting_patience: %.2f", waiting_patience_); 

  parameter_->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  rclcpp::Parameter controller_frequency = parameter_->get_parameter("controller_frequency");
  controller_frequency_ = controller_frequency.as_double();
  RCLCPP_INFO(logger_->get_logger(), "controller_frequency: %.2f", controller_frequency_); 

  parameter_->declare_parameter("use_twist_stamped", rclcpp::ParameterValue(false));
  rclcpp::Parameter use_twist_stamped = parameter_->get_parameter("use_twist_stamped");
  use_twist_stamped_ = use_twist_stamped.as_bool();
  RCLCPP_INFO(logger_->get_logger(), "use_stamped_twist: %d", use_twist_stamped_); 

  parameter_->declare_parameter("use_position_control_at_goal", rclcpp::ParameterValue(false));
  rclcpp::Parameter use_position_control_at_goal = parameter_->get_parameter("use_position_control_at_goal");
  use_position_control_at_goal_ = use_position_control_at_goal.as_bool();
  RCLCPP_INFO(logger_->get_logger(), "use_position_control_at_goal: %d", use_position_control_at_goal_); 

  parameter_->declare_parameter("main_trajectory_generator", rclcpp::ParameterValue("differential_drive_simple"));
  rclcpp::Parameter main_trajectory_generator = parameter_->get_parameter("main_trajectory_generator");
  main_trajectory_generator_ = main_trajectory_generator.as_string();
  RCLCPP_INFO(logger_->get_logger(), "main_trajectory_generator: %s", main_trajectory_generator_.c_str()); 

}

bool State::isCurrentDecision(std::string m_decision){
  if(m_decision==current_decision_)
    return true;
  return false;
}


void State::initialParams(geometry_msgs::msg::TransformStamped robot_curent_pose, rclcpp::Time current_time){

  last_decision_ = "d_initial";
  current_decision_ = "d_initial";

  oscillation_pose_ = robot_curent_pose;
  last_oscillation_reset_ = current_time;
  last_valid_plan_ = current_time;
  last_valid_control_ = current_time;
  no_plan_recovery_count_ = 0;

}


double State::getDistance(geometry_msgs::msg::TransformStamped& a, geometry_msgs::msg::TransformStamped& b){

  double dx = a.transform.translation.x - b.transform.translation.x;
  double dy = a.transform.translation.y - b.transform.translation.y;
  double dz = a.transform.translation.z - b.transform.translation.z;

  return sqrt(dx*dx + dy*dy + dz*dz);
}

double State::getAngle(geometry_msgs::msg::TransformStamped& a, geometry_msgs::msg::TransformStamped& b){

  tf2::Transform pose_a;
  pose_a.setRotation(tf2::Quaternion(a.transform.rotation.x, a.transform.rotation.y, a.transform.rotation.z, a.transform.rotation.w));
  pose_a.setOrigin(tf2::Vector3(a.transform.translation.x, a.transform.translation.y, a.transform.translation.z));
  
  tf2::Transform pose_b;
  pose_b.setRotation(tf2::Quaternion(b.transform.rotation.x, b.transform.rotation.y, b.transform.rotation.z, b.transform.rotation.w));
  pose_b.setOrigin(tf2::Vector3(b.transform.translation.x, b.transform.translation.y, b.transform.translation.z));
  
  auto pose_a_inverse = pose_a.inverse();

  tf2::Transform pose_a2b;
  pose_a2b.mult(pose_a_inverse, pose_b);
  /*Get RPY*/
  tf2::Matrix3x3 m(pose_a2b.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return fabs(yaw);

}

bool State::setDecision(std::string m_decision){

  last_decision_ = current_decision_;
  current_decision_ = m_decision;

  //if(last_decision_.compare(current_decision_)!=0)
  //  RCLCPP_INFO(logger_->get_logger(), "Decision from -- %s -- to -- %s --", last_decision_.c_str(), current_decision_.c_str()); 
  
  return true;

}

}//end of name space
