/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
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
#include <trajectory_generators/ak_rotate_in_place_theory.h>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::AKRotateInPlaceTheory,
                       trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

AKRotateInPlaceTheory::AKRotateInPlaceTheory()
{
  return;
}

void AKRotateInPlaceTheory::configurateActuatorType()
{
  // P2PMoveBase publishes STEERING trajectories as AckermannDriveStamped.
  actuator_type_ = dddmr_sys_core::ActuatorType::STEERING;
}

void AKRotateInPlaceTheory::onInitialize()
{
  limits_ = std::make_shared<AckermannTrajectoryGeneratorLimits>();
  params_ = std::make_shared<AckermannTrajectoryGeneratorParams>();

  // This is deliberately independent of min_vel_x: it is the speed used only
  // while acquiring an initial/final heading with a maximum-steering arc.
  node_->declare_parameter(name_ + ".rotate_linear_speed",
                           rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".rotate_linear_speed", rotate_linear_speed_);
  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "rotate_linear_speed: %.3f", rotate_linear_speed_);

  // Keep the same vehicle model parameters as ackermann_simple.
  node_->declare_parameter(name_ + ".wheelbase", rclcpp::ParameterValue(3.0));
  node_->get_parameter(name_ + ".wheelbase", limits_->wheelbase);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "wheelbase: %.4f",
              limits_->wheelbase);

  node_->declare_parameter(name_ + ".max_steer_rad",
                           rclcpp::ParameterValue(0.342));
  node_->get_parameter(name_ + ".max_steer_rad", limits_->max_steer_rad);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_steer_rad: %.4f",
              limits_->max_steer_rad);

  node_->declare_parameter(name_ + ".max_lat_accel",
                           rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".max_lat_accel", limits_->max_lat_accel);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_lat_accel: %.3f",
              limits_->max_lat_accel);

  node_->declare_parameter(name_ + ".max_steering_rate",
                           rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".max_steering_rate",
                       limits_->max_steering_rate);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "max_steering_rate: %.3f",
              limits_->max_steering_rate);

  node_->declare_parameter(name_ + ".controller_frequency",
                           rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".controller_frequency",
                       params_->controller_frequency);

  node_->declare_parameter(name_ + ".sim_time", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".sim_time", params_->sim_time);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "sim_time: %.3f",
              params_->sim_time);

  node_->declare_parameter(name_ + ".sim_granularity",
                           rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".sim_granularity", params_->sim_granularity);

  node_->declare_parameter(name_ + ".angular_sim_granularity",
                           rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".angular_sim_granularity",
                       params_->angular_sim_granularity);

  if (rotate_linear_speed_ <= 0.0 || limits_->wheelbase <= 0.0 ||
      limits_->max_steer_rad <= 0.0 || params_->sim_time <= 0.0 ||
      params_->sim_granularity <= 0.0 ||
      params_->angular_sim_granularity <= 0.0) {
    RCLCPP_FATAL(node_->get_logger().get_child(name_),
                 "rotate_linear_speed, wheelbase, max_steer_rad, sim_time, "
                 "and simulation granularities must all be positive.");
  }

  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "Start to parse cuboid.");
  const auto cuboid_point = [this](const std::string &corner) {
    const std::string parameter = name_ + ".cuboid." + corner;
    node_->declare_parameter(parameter, rclcpp::PARAMETER_DOUBLE_ARRAY);
    const auto values = node_->get_parameter(parameter).as_double_array();
    if (values.size() != 3) {
      RCLCPP_FATAL(node_->get_logger().get_child(name_),
                   "Cuboid %s must contain exactly [x, y, z].", corner.c_str());
      return pcl::PointXYZ();
    }

    pcl::PointXYZ point;
    point.x = values[0];
    point.y = values[1];
    point.z = values[2];
    RCLCPP_INFO(node_->get_logger().get_child(name_),
                "Cuboid %s: %.3f, %.3f, %.3f", corner.c_str(), point.x,
                point.y, point.z);
    return point;
  };

  // The point order is required by CollisionModel's oriented-cuboid test.
  params_->cuboid.push_back(cuboid_point("blb"));
  params_->cuboid.push_back(cuboid_point("brb"));
  params_->cuboid.push_back(cuboid_point("blt"));
  params_->cuboid.push_back(cuboid_point("flb"));
  params_->cuboid.push_back(cuboid_point("brt"));
  params_->cuboid.push_back(cuboid_point("frt"));
  params_->cuboid.push_back(cuboid_point("flt"));
  params_->cuboid.push_back(cuboid_point("frb"));
}

void AKRotateInPlaceTheory::initialise()
{
  next_sample_index_ = 0;
  sample_params_.clear();

  if (rotate_linear_speed_ <= 0.0 || limits_->wheelbase <= 0.0 ||
      limits_->max_steer_rad <= 0.0) {
    return;
  }

  // The steering target is full lock.  A configured lateral-acceleration cap
  // may lower it at the fixed maneuver speed, exactly as ackermann_simple does.
  double steer_magnitude = limits_->max_steer_rad;
  if (limits_->max_lat_accel > 0.0) {
    const double lateral_accel_limit = std::atan(
        limits_->wheelbase * limits_->max_lat_accel /
        (rotate_linear_speed_ * rotate_linear_speed_));
    steer_magnitude = std::min(steer_magnitude, lateral_accel_limit);
  }

  Eigen::Vector3f left_command = Eigen::Vector3f::Zero();
  Eigen::Vector3f right_command = Eigen::Vector3f::Zero();
  left_command[0] = rotate_linear_speed_;
  right_command[0] = rotate_linear_speed_;
  left_command[2] = steer_magnitude;
  right_command[2] = -steer_magnitude;
  sample_params_.push_back(left_command);
  sample_params_.push_back(right_command);
}

size_t AKRotateInPlaceTheory::getSamplingSize(){
  return sample_params_.size();
}

void AKRotateInPlaceTheory::getSamplingTrajectoryByIndex(size_t index, base_trajectory::Trajectory& _traj){
  generateTrajectory(sample_params_[index], _traj);
}

bool AKRotateInPlaceTheory::generateTrajectory(
    Eigen::Vector3f command, base_trajectory::Trajectory &traj)
{
  const double speed = command[0];
  const double steering_angle = command[2];
  if (speed <= 0.0 || limits_->wheelbase <= 0.0 ||
      params_->sim_time <= 0.0) {
    return false;
  }

  traj.actuator_type_ = actuator_type_;
  traj.cost_ = 0.0;
  traj.resetPoses();

  const double yaw_rate =
      speed * std::tan(steering_angle) / limits_->wheelbase;
  const double simulated_distance = speed * params_->sim_time;
  const double simulated_angle = std::fabs(yaw_rate) * params_->sim_time;
  const int num_steps = static_cast<int>(std::ceil(std::max(
      simulated_distance / params_->sim_granularity,
      simulated_angle / params_->angular_sim_granularity)));
  if (num_steps <= 0) {
    return false;
  }

  const double dt = params_->sim_time / num_steps;
  traj.time_delta_ = dt;
  traj.xv_ = speed;
  traj.yv_ = 0.0;
  // Keep yaw rate for ShortestAngleModel; command output uses the dedicated
  // Ackermann steering fields below.
  traj.thetav_ = yaw_rate;
  traj.steering_angle_ = steering_angle;
  traj.steering_angle_velocity_ =
      limits_->max_steering_rate > 0.0 ? limits_->max_steering_rate : 0.0;

  const Eigen::Affine3d global_to_base =
      tf2::transformToEigen(shared_data_->robot_pose_);
  Eigen::Vector3f relative_pose = Eigen::Vector3f::Zero();
  for (int step = 0; step < num_steps; ++step) {
    relative_pose = computeNewPositions(relative_pose, command, dt);

    Eigen::Affine3d base_to_trajectory(
        Eigen::AngleAxisd(relative_pose[2], Eigen::Vector3d::UnitZ()));
    base_to_trajectory.translation().x() = relative_pose[0];
    base_to_trajectory.translation().y() = relative_pose[1];
    const Eigen::Affine3d global_to_trajectory =
        global_to_base * base_to_trajectory;

    const auto transform = tf2::eigenToTransform(global_to_trajectory);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = shared_data_->robot_pose_.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> transformed_cuboid;
    pcl::transformPointCloud(params_->cuboid, transformed_cuboid,
                             global_to_trajectory);
    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(transformed_cuboid, cuboid_min_max.first,
                     cuboid_min_max.second);
    if (!traj.addPoseCuboid(pose, transformed_cuboid, cuboid_min_max)) {
      return false;
    }
  }

  return true;
}

Eigen::Vector3f AKRotateInPlaceTheory::computeNewPositions(
    const Eigen::Vector3f &pos, const Eigen::Vector3f &command, double dt)
{
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  const double speed = command[0];
  const double steering_angle = command[2];
  const double yaw_rate =
      speed * std::tan(steering_angle) / limits_->wheelbase;

  new_pos[0] = pos[0] + speed * std::cos(pos[2]) * dt;
  new_pos[1] = pos[1] + speed * std::sin(pos[2]) * dt;
  new_pos[2] = pos[2] + yaw_rate * dt;
  return new_pos;
}

void AKRotateInPlaceTheory::expertScoring(std::vector<base_trajectory::Trajectory>& accepted_trajectories,
                                            std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories,
                                              base_trajectory::Trajectory& best_traj){
  //use default scoring
  TrajectoryGeneratorTheory::expertScoring(accepted_trajectories, rejected_trajectories, best_traj);
}

}  // namespace trajectory_generators
