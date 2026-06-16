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


/*
This trajectory generator take shared_data_->ref_twist_for_trajectory_generation_ to
generate a trahectory.
shared_data_->ref_twist_for_trajectory_generation_ is set by recovery behavior or planner
*/

#include <trajectory_generators/one_state_trajectory_generator_theory.h>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::OneStateTrajectoryGeneratorTheory, trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

OneStateTrajectoryGeneratorTheory::OneStateTrajectoryGeneratorTheory(){
  return;
}

void OneStateTrajectoryGeneratorTheory::onInitialize(){

  //@initialize trajectory generator
  limits_ = std::make_shared<trajectory_generators::OmniTrajectoryGeneratorLimits>();
  
  //@initial params
  params_ = std::make_shared<trajectory_generators::OmniTrajectoryGeneratorParams>();

  node_->declare_parameter(name_ + ".controller_frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency", params_->controller_frequency);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "controller_frequency: %.2f", params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.2f", params_->sim_time);


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

void OneStateTrajectoryGeneratorTheory::initialise(){

  //@ push override twist to generate a trajectory
  double max_vel_th = limits_->max_vel_theta;
  double min_vel_th = -1.0 * max_vel_th;
  Eigen::Vector3f acc_lim = limits_->getAccLimits();
  next_sample_index_ = 0;
  sample_params_.clear();

  Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
  vel_samp[0] = shared_data_->ref_twist_for_trajectory_generation_.twist.linear.x;
  vel_samp[1] = shared_data_->ref_twist_for_trajectory_generation_.twist.linear.y;
  vel_samp[2] = shared_data_->ref_twist_for_trajectory_generation_.twist.angular.z;
  sample_params_.push_back(vel_samp);
  
}

bool OneStateTrajectoryGeneratorTheory::isMotorConstraintSatisfied(Eigen::Vector3f& vel_samp){
  
  //@ if we dont want motor constraint, return constraint is atisfied
  if(!limits_->use_motor_constraint)
    return true;

  return true;

  //@ TODO: Omni kinematics required
}


bool OneStateTrajectoryGeneratorTheory::hasMoreTrajectories(){
  return next_sample_index_ < sample_params_.size();
}

bool OneStateTrajectoryGeneratorTheory::nextTrajectory(base_trajectory::Trajectory& _traj){
  bool result = false;
  /*
  Because generateTrajectory will return false when sample params are not satisfied, 
  the function will just sweep without generating any trajectory
  */
  bool generated_once = false;
  if (hasMoreTrajectories()) {
    
    if (generateTrajectory(
        sample_params_[next_sample_index_],
        _traj)) {
      result = true;
      generated_once = true;
    }
    else{
      _traj.resetPoints();
    }
    
  }
  else{
    if(!generated_once)
      RCLCPP_ERROR(node_->get_logger().get_child(name_), "None of trajectory is generated, maybe acc is too small or check the generateTrajectory function.");
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool OneStateTrajectoryGeneratorTheory::generateTrajectory(
      Eigen::Vector3f sample_target_vel,
      base_trajectory::Trajectory& traj) {

  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  double eps = 1e-4;
  traj.cost_ = 0.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Trajectory by state x: %.2f, y: %.2f, w: %.2f", sample_target_vel[0], sample_target_vel[1], sample_target_vel[2]);
  
  int num_steps;
  num_steps = params_->sim_time * params_->controller_frequency;
  
  if (num_steps == 0) {
    return false;
  }

  //compute a timestep
  double dt = params_->sim_time / num_steps;
  traj.time_delta_ = dt;
  
  Eigen::Vector3f loop_vel;

  // assuming sample_vel is our target velocity within acc limits for one timestep
  loop_vel = sample_target_vel;
  traj.xv_     = sample_target_vel[0];
  traj.yv_     = sample_target_vel[1];
  traj.thetav_ = sample_target_vel[2];
  
  /*We first create trajectory based on robot_frame, then we use affine to transform it to global frame*/
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  //simulate the trajectory and check for collisions, updating costs along the way
  for (int i = 0; i < num_steps; ++i) {

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

    /*transform back to global frame*/
    Eigen::Affine3d trans_gbl2traj_af3;

    
    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[2], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];
    
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

    if(!traj.addPoint(ros_pose, pc_out, cuboid_min_max)){
      return false;
    }

  } // end for simulation steps

  return true; // trajectory has at least one point
}

Eigen::Vector3f OneStateTrajectoryGeneratorTheory::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}
}//end of name space