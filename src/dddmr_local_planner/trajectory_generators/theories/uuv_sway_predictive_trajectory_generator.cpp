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
#include <trajectory_generators/uuv_sway_predictive_trajectory_generator.h>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::UUVSwayPredictiveTrajectoryGeneratorTheory, trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

UUVSwayPredictiveTrajectoryGeneratorTheory::UUVSwayPredictiveTrajectoryGeneratorTheory(){
  return;
}

void UUVSwayPredictiveTrajectoryGeneratorTheory::configurateActuatorType(){
  actuator_type_ = dddmr_sys_core::ActuatorType::MOTOR;
}


void UUVSwayPredictiveTrajectoryGeneratorTheory::onInitialize(){
  
  //@initialize trajectory generator
  limits_ = std::make_shared<trajectory_generators::UUVTrajectoryGeneratorLimits>();

  node_->declare_parameter(name_ + ".min_vel_x", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_x", limits_->min_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_x: %.2f", limits_->min_vel_x);

  node_->declare_parameter(name_ + ".max_vel_x", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_x", limits_->max_vel_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_x: %.2f", limits_->max_vel_x);

  node_->declare_parameter(name_ + ".min_vel_y", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_y", limits_->min_vel_y);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_y: %.2f", limits_->min_vel_y);

  node_->declare_parameter(name_ + ".max_vel_y", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_y", limits_->max_vel_y);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_y: %.2f", limits_->max_vel_y);

  node_->declare_parameter(name_ + ".min_vel_z", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_z", limits_->min_vel_z);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_z: %.2f", limits_->min_vel_z);

  node_->declare_parameter(name_ + ".max_vel_z", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_z", limits_->max_vel_z);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_z: %.2f", limits_->max_vel_z);

  node_->declare_parameter(name_ + ".min_vel_trans", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".min_vel_trans", limits_->min_vel_trans);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_trans: %.2f", limits_->min_vel_trans);

  node_->declare_parameter(name_ + ".max_vel_trans", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_trans", limits_->max_vel_trans);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_trans: %.2f", limits_->max_vel_trans);

  node_->declare_parameter(name_ + ".min_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".min_vel_theta", limits_->min_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "min_vel_theta: %.2f", limits_->min_vel_theta);

  node_->declare_parameter(name_ + ".max_vel_theta", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".max_vel_theta", limits_->max_vel_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_vel_theta: %.2f", limits_->max_vel_theta);

  node_->declare_parameter(name_ + ".acc_lim_x", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_x", limits_->acc_lim_x);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_x: %.2f", limits_->acc_lim_x);

  node_->declare_parameter(name_ + ".acc_lim_y", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_y", limits_->acc_lim_y);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_y: %.2f", limits_->acc_lim_y);

  node_->declare_parameter(name_ + ".acc_lim_z", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".acc_lim_z", limits_->acc_lim_z);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_z: %.2f", limits_->acc_lim_z);

  node_->declare_parameter(name_ + ".acc_lim_theta", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".acc_lim_theta", limits_->acc_lim_theta);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "acc_lim_theta: %.2f", limits_->acc_lim_theta);

  node_->declare_parameter(name_ + ".prune_forward", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + ".prune_forward", limits_->prune_forward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_forward: %.2f", limits_->prune_forward);

  node_->declare_parameter(name_ + ".prune_backward", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".prune_backward", limits_->prune_backward);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "prune_backward: %.2f", limits_->prune_backward);


  //@deceleration allow the robot to consider the deceleration based on the current_speed/deceleration_ratio
  //@for example, if current speed of robot is 1.2 then the considered min_vel will be 0.6 instead of current_speed-dt*acc
  node_->declare_parameter(name_ + ".deceleration_ratio", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".deceleration_ratio", limits_->deceleration_ratio);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "deceleration_ratio: %.2f", limits_->deceleration_ratio);

  if(limits_->min_vel_x<0)
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "The min velocity of the robot should be positive!");

  /*Motor constraint*/
  node_->declare_parameter(name_ + ".use_power_constraint", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".use_power_constraint", limits_->use_power_constraint);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "use_power_constraint: %d", limits_->use_power_constraint);

  node_->declare_parameter(name_ + ".robot_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".robot_radius", limits_->robot_radius);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "robot_radius: %.2f", limits_->robot_radius);

  //@initial params
  params_ = std::make_shared<trajectory_generators::UUVTrajectoryGeneratorParams>();
  
  node_->declare_parameter(name_ + ".lateral_velocity_incurred_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".lateral_velocity_incurred_weight", lateral_velocity_incurred_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "lateral_velocity_incurred_weight: %.2f", lateral_velocity_incurred_weight_);

  node_->declare_parameter(name_ + ".controller_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency", params_->controller_frequency);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "controller_frequency: %.2f", params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.2f", params_->sim_time);

  node_->declare_parameter(name_ + ".linear_x_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_x_sample", params_->linear_x_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_x_sample: %.2f", params_->linear_x_sample);

  node_->declare_parameter(name_ + ".linear_y_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_y_sample", params_->linear_y_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_y_sample: %.2f", params_->linear_y_sample);

  node_->declare_parameter(name_ + ".linear_z_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".linear_z_sample", params_->linear_z_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "linear_z_sample: %.2f", params_->linear_z_sample);

  node_->declare_parameter(name_ + ".angular_z_sample", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".angular_z_sample", params_->angular_z_sample);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_z_sample: %.2f", params_->angular_z_sample);

  node_->declare_parameter(name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".sim_granularity", params_->sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_granularity: %.2f", params_->sim_granularity);

  node_->declare_parameter(name_ + ".angular_sim_granularity", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".angular_sim_granularity", params_->angular_sim_granularity);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "angular_sim_granularity: %.2f", params_->angular_sim_granularity);

  //@ parse cuboid
  /*
  for(int i=1;i<9;i++){
    std::string s = ".cuboid.p" + std::to_string(i);

    node_->declare_parameter(name_ + s, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param = node_->get_parameter(name_ + s);
    auto p = cuboid_param.as_double_array();
    pcl::PointXYZ pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    params_->cuboid.push_back(pt);
    RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid %.2f, %.2f, %.2f", pt.x, pt.y, pt.z);
  }
  */
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<double> p;

  node_->declare_parameter(name_ + ".cuboid.flb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flb= node_->get_parameter(name_ + ".cuboid.flb");
  p = cuboid_flb.as_double_array();
  pcl::PointXYZ pt_flb;
  pt_flb.x = p[0];pt_flb.y = p[1];pt_flb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid flb: %.2f, %.2f, %.2f", pt_flb.x, pt_flb.y, pt_flb.z);

  node_->declare_parameter(name_ + ".cuboid.frb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frb= node_->get_parameter(name_ + ".cuboid.frb");
  p = cuboid_frb.as_double_array();
  pcl::PointXYZ pt_frb;
  pt_frb.x = p[0];pt_frb.y = p[1];pt_frb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid frb: %.2f, %.2f, %.2f", pt_frb.x, pt_frb.y, pt_frb.z);

  node_->declare_parameter(name_ + ".cuboid.flt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_flt= node_->get_parameter(name_ + ".cuboid.flt");
  p = cuboid_flt.as_double_array();
  pcl::PointXYZ pt_flt;
  pt_flt.x = p[0];pt_flt.y = p[1];pt_flt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid flt: %.2f, %.2f, %.2f", pt_flt.x, pt_flt.y, pt_flt.z);

  node_->declare_parameter(name_ + ".cuboid.frt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_frt= node_->get_parameter(name_ + ".cuboid.frt");
  p = cuboid_frt.as_double_array();
  pcl::PointXYZ pt_frt;
  pt_frt.x = p[0];pt_frt.y = p[1];pt_frt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid frt: %.2f, %.2f, %.2f", pt_frt.x, pt_frt.y, pt_frt.z);

  node_->declare_parameter(name_ + ".cuboid.blb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blb= node_->get_parameter(name_ + ".cuboid.blb");
  p = cuboid_blb.as_double_array();
  pcl::PointXYZ pt_blb;
  pt_blb.x = p[0];pt_blb.y = p[1];pt_blb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid blb: %.2f, %.2f, %.2f", pt_blb.x, pt_blb.y, pt_blb.z);

  node_->declare_parameter(name_ + ".cuboid.brb", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brb= node_->get_parameter(name_ + ".cuboid.brb");
  p = cuboid_brb.as_double_array();
  pcl::PointXYZ pt_brb;
  pt_brb.x = p[0];pt_brb.y = p[1];pt_brb.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid brb: %.2f, %.2f, %.2f", pt_brb.x, pt_brb.y, pt_brb.z);

  node_->declare_parameter(name_ + ".cuboid.blt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_blt= node_->get_parameter(name_ + ".cuboid.blt");
  p = cuboid_blt.as_double_array();
  pcl::PointXYZ pt_blt;
  pt_blt.x = p[0];pt_blt.y = p[1];pt_blt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid blt: %.2f, %.2f, %.2f", pt_blt.x, pt_blt.y, pt_blt.z);

  node_->declare_parameter(name_ + ".cuboid.brt", rclcpp::PARAMETER_DOUBLE_ARRAY);
  rclcpp::Parameter cuboid_brt= node_->get_parameter(name_ + ".cuboid.brt");
  p = cuboid_brt.as_double_array();
  pcl::PointXYZ pt_brt;
  pt_brt.x = p[0];pt_brt.y = p[1];pt_brt.z = p[2];
  RCLCPP_INFO(node_->get_logger().get_child(name_), "Cuboid brt: %.2f, %.2f, %.2f", pt_brt.x, pt_brt.y, pt_brt.z);

  params_->cuboid.push_back(pt_blb);
  params_->cuboid.push_back(pt_brb);
  params_->cuboid.push_back(pt_blt);
  params_->cuboid.push_back(pt_flb);
  params_->cuboid.push_back(pt_brt);
  params_->cuboid.push_back(pt_frt);
  params_->cuboid.push_back(pt_flt);
  params_->cuboid.push_back(pt_frb);
  //@ push the point by following sequence, because when doing the point in cuboid test we leverage blb/brb/blt/flb
  //@ back left top = blt; back right bottom = brb;
  //
  //          -------
  //         /|    /|
  //        / |   / |
  //     blt------- |
  //        | /flb| /
  //        |/    |/
  //     blb-------brb

  if(params_->cuboid.size()!=8){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "Cuboid is essential.");
  }

}

void UUVSwayPredictiveTrajectoryGeneratorTheory::initialise(){
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits_->max_vel_theta;
  double min_vel_th = -1.0 * max_vel_th;
  auto acc_lim = limits_->getAccLimits();
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits_->min_vel_x;
  double max_vel_x = limits_->max_vel_x;

  double min_vel_y = limits_->min_vel_y;
  double max_vel_y = limits_->max_vel_y;

  double min_vel_z = limits_->min_vel_z;
  double max_vel_z = limits_->max_vel_z;

  // if sampling number is zero in any dimension, we don't generate samples generically
  if (params_->linear_x_sample * params_->angular_z_sample > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::VectorXf max_vel6d = Eigen::VectorXf::Zero(6); //xyzrpy
    Eigen::VectorXf min_vel6d = Eigen::VectorXf::Zero(6);


    // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
    double sim_period = 1.0/params_->controller_frequency;
    

    max_vel6d[0] = std::min(max_vel_x, shared_data_->robot_state_.twist.twist.linear.x + acc_lim[0] * sim_period);
    max_vel6d[1] = std::min(max_vel_y, shared_data_->robot_state_.twist.twist.linear.y + acc_lim[1] * sim_period);
    max_vel6d[2] = std::min(max_vel_z, shared_data_->robot_state_.twist.twist.linear.z + acc_lim[2] * sim_period);
    max_vel6d[5] = std::min(max_vel_th, shared_data_->robot_state_.twist.twist.angular.z + acc_lim[5] * sim_period);

    min_vel6d[0] = std::max(min_vel_x, shared_data_->robot_state_.twist.twist.linear.x - acc_lim[0] * sim_period);
    min_vel6d[1] = std::max(min_vel_y, shared_data_->robot_state_.twist.twist.linear.y - acc_lim[1] * sim_period);
    min_vel6d[2] = std::max(min_vel_z, shared_data_->robot_state_.twist.twist.linear.z - acc_lim[2] * sim_period);
    min_vel6d[5] = std::max(min_vel_th, shared_data_->robot_state_.twist.twist.angular.z - acc_lim[5] * sim_period);
    
    
    if(shared_data_->robot_state_.twist.twist.linear.x >= max_vel_x/limits_->deceleration_ratio){
      //@ robot reach max speed at forward/backward
      min_vel6d[0] = std::max(min_vel_x, shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio);
    }
    else if(shared_data_->robot_state_.twist.twist.linear.x <= min_vel_x/limits_->deceleration_ratio){
      max_vel6d[0] = std::min(max_vel_x, shared_data_->robot_state_.twist.twist.linear.x/limits_->deceleration_ratio);
    }

    if(shared_data_->robot_state_.twist.twist.linear.y >= max_vel_y/limits_->deceleration_ratio){
      //@ robot reach max speed at lateral
      min_vel6d[1] = std::max(min_vel_y, shared_data_->robot_state_.twist.twist.linear.y/limits_->deceleration_ratio);
    }
    else if(shared_data_->robot_state_.twist.twist.linear.y <= min_vel_y/limits_->deceleration_ratio){
      max_vel6d[1] = std::min(max_vel_y, shared_data_->robot_state_.twist.twist.linear.y/limits_->deceleration_ratio);
    }

    if(shared_data_->robot_state_.twist.twist.linear.z >= max_vel_z/limits_->deceleration_ratio){
      //@ robot reach max speed at z
      min_vel6d[2] = std::max(min_vel_z, shared_data_->robot_state_.twist.twist.linear.z/limits_->deceleration_ratio);
    }
    else if(shared_data_->robot_state_.twist.twist.linear.z <= min_vel_z/limits_->deceleration_ratio){
      max_vel6d[2] = std::min(max_vel_z, shared_data_->robot_state_.twist.twist.linear.z/limits_->deceleration_ratio);
    }

    Eigen::VectorXf vel_sample_6d = Eigen::VectorXf::Zero(6); //xyzrpy
    trajectory_generators::VelocityIterator x_it(min_vel6d[0], max_vel6d[0], params_->linear_x_sample);
    trajectory_generators::VelocityIterator y_it(min_vel6d[1], max_vel6d[1], params_->linear_y_sample);
    trajectory_generators::VelocityIterator z_it(min_vel6d[2], max_vel6d[2], params_->linear_z_sample);
    trajectory_generators::VelocityIterator th_it(min_vel6d[5], max_vel6d[5], params_->angular_z_sample);
    for(; !x_it.isFinished(); x_it++) {
      vel_sample_6d[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_sample_6d[1] = y_it.getVelocity();
        for(; !z_it.isFinished(); z_it++){
          vel_sample_6d[2] = z_it.getVelocity();
          for(; !th_it.isFinished(); th_it++) {
            vel_sample_6d[5] = th_it.getVelocity();
            sample_params_.push_back(vel_sample_6d);
          }
          th_it.reset();
        }
        z_it.reset();
      }
      y_it.reset();
    }
  }    
}

bool UUVSwayPredictiveTrajectoryGeneratorTheory::isPowerConstraintSatisfied(Eigen::VectorXf& vel6d){
  
  //@ if we dont want motor constraint, return constraint is atisfied
  if(!limits_->use_power_constraint)
    return true;
  return true;
  //@ compute thrust command that will not cause over current
}

size_t UUVSwayPredictiveTrajectoryGeneratorTheory::getSamplingSize(){
  return sample_params_.size();
}

void UUVSwayPredictiveTrajectoryGeneratorTheory::getSamplingTrajectoryByIndex(size_t index, base_trajectory::Trajectory& _traj){
  generateTrajectory(sample_params_[index], _traj);
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool UUVSwayPredictiveTrajectoryGeneratorTheory::generateTrajectory(
      Eigen::VectorXf& sample_target_vel,
      base_trajectory::Trajectory& traj) {

  //@ assign actuator type to trajectory, so that when move base publishing the cmd_vel,
  //@ the correct publisher will be used i.e., geometry/twist or ackermann
  traj.actuator_type_ = actuator_type_;
  
  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  double vmag = sample_target_vel.head<3>().norm();
  double eps = 1e-4;
  traj.cost_ = 0.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoses();

  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[5]) + eps < limits_->min_vel_theta)) {
    return false;
  }
  // make sure we do not exceed max diagonal (x+y+z) translational velocity (if set)
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) {
    return false;
  }

  int num_steps;

  //compute the number of steps we must take along this trajectory to be "safe"
  double sim_time_distance = vmag * params_->sim_time; // the distance the robot would travel in sim_time if it did not change velocity
  double sim_time_angle = fabs(sample_target_vel[5]) * params_->sim_time; // the angle the robot would rotate in sim_time
  num_steps =
      ceil(std::max(sim_time_distance / params_->sim_granularity,
          sim_time_angle / params_->angular_sim_granularity));
  

  if (num_steps == 0) {
    return false;
  }

  //compute a timestep
  double dt = params_->sim_time / num_steps;
  traj.time_delta_ = dt;

  //compute a timestep
  //double dt = 0.1;
  //int num_steps = 30;
  //traj.time_delta_ = dt;

  
  Eigen::VectorXf loop_vel;

  // assuming sample_vel is our target velocity within acc limits for one timestep
  loop_vel = sample_target_vel;
  traj.xv_     = sample_target_vel[0];
  traj.yv_     = sample_target_vel[1];
  traj.zv_     = sample_target_vel[2];
  traj.thetav_ = sample_target_vel[5];

  /*We first create trajectory based on robot_frame, then we use affine to transform it to global frame*/
  Eigen::VectorXf pos = Eigen::VectorXf::Zero(6);
  //simulate the trajectory and check for collisions, updating costs along the way
  for (int i = 0; i < num_steps; ++i) {

    //y incurred by x speed and rotation
    lateral_velocity_incurred_weight_ = std::min(lateral_velocity_incurred_weight_, 1.0);
    loop_vel[1] += loop_vel[0] * cos(1.5707963 + pos[5]) * lateral_velocity_incurred_weight_;

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

    /*transform back to global frame*/
    Eigen::Affine3d trans_gbl2traj_af3;

    
    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[5], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];
    trans_b2traj_af3.translation().z() = pos[2];
    
    /*
    tf2::Quaternion tf2_rotation;
    tf2_rotation.setRPY( 0, 0, pos[2]); 
    tf2_rotation.normalize();
    geometry_msgs::TransformStamped trans_b2traj;
    trans_b2traj.transform.translation.x = pos[0];
    trans_b2traj.transform.translation.y = pos[1];
    trans_b2traj.transform.rotation.x = tf2_rotation.x();
    trans_b2traj.transform.rotation.x = tf2_rotation.y();
    trans_b2traj.transform.rotation.x = tf2_rotation.z();
    trans_b2traj.transform.rotation.x = tf2_rotation.w();
    Eigen::Affine3d trans_b2traj_af3 = tf2::transformToEigen(trans_b2traj);
    */
    trans_gbl2traj_af3 = pos_af3*trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped trans_gbl2traj_ = tf2::eigenToTransform (trans_gbl2traj_af3);
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = trans_gbl2traj_.transform.translation.x;
    ros_pose.pose.position.y = trans_gbl2traj_.transform.translation.y;
    ros_pose.pose.position.z = trans_gbl2traj_.transform.translation.z;
    ros_pose.pose.orientation = trans_gbl2traj_.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(params_->cuboid, pc_out, trans_gbl2traj_af3);
    
    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(pc_out, cuboid_min_max.first, cuboid_min_max.second);

    if(!traj.addPoseCuboid(ros_pose, pc_out, cuboid_min_max)){
      return false;
    }

  } // end for simulation steps

  return true; // trajectory has at least one point
}

Eigen::VectorXf UUVSwayPredictiveTrajectoryGeneratorTheory::computeNewPositions(const Eigen::VectorXf& pos,
    const Eigen::VectorXf& vel6d, double dt) {
  Eigen::VectorXf new_pos = pos;
  new_pos[0] += (vel6d[0] * cos(pos[5]) + vel6d[1] * cos(M_PI_2 + pos[5])) * dt;
  new_pos[1] += (vel6d[0] * sin(pos[5]) + vel6d[1] * sin(M_PI_2 + pos[5])) * dt;
  new_pos[2] += vel6d[2] * dt;
  new_pos[5] += vel6d[5] * dt;
  return new_pos;
}

void UUVSwayPredictiveTrajectoryGeneratorTheory::expertScoring(std::vector<base_trajectory::Trajectory>& accepted_trajectories,
                                            std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories, 
                                              base_trajectory::Trajectory& best_traj){
  //use default scoring
  TrajectoryGeneratorTheory::expertScoring(accepted_trajectories, rejected_trajectories, best_traj); 
}
}//end of name space