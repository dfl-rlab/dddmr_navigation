/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include <mcl_3dl/parameters.h>

namespace mcl_3dl
{

Parameters::Parameters(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
              const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter)
{
  logger_ = m_logger;
  parameter_ = m_parameter;
  
  parameter_->declare_parameter("map_frame", rclcpp::ParameterValue(""));
  rclcpp::Parameter map_frame = parameter_->get_parameter("map_frame");
  frame_ids_["map"] = map_frame.as_string();
  RCLCPP_INFO(logger_->get_logger(), "map_frame: %s", frame_ids_["map"].c_str());

  parameter_->declare_parameter("robot_frame", rclcpp::ParameterValue(""));
  rclcpp::Parameter robot_frame = parameter_->get_parameter("robot_frame");
  frame_ids_["base_link"] = robot_frame.as_string();
  RCLCPP_INFO(logger_->get_logger(), "robot_frame: %s", frame_ids_["base_link"].c_str());  
  
  parameter_->declare_parameter("odom_frame", rclcpp::ParameterValue(""));
  rclcpp::Parameter odom_frame = parameter_->get_parameter("odom_frame");
  frame_ids_["odom"] = odom_frame.as_string();
  RCLCPP_INFO(logger_->get_logger(), "odom_frame: %s", frame_ids_["odom"].c_str());  
  
  parameter_->declare_parameter("map_downsample_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_downsample_x = parameter_->get_parameter("map_downsample_x");
  map_downsample_x_ = map_downsample_x.as_double();
  RCLCPP_INFO(logger_->get_logger(), "map_downsample_x: %.2f", map_downsample_x_);  

  parameter_->declare_parameter("map_downsample_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_downsample_y = parameter_->get_parameter("map_downsample_y");
  map_downsample_y_ = map_downsample_y.as_double();
  RCLCPP_INFO(logger_->get_logger(), "map_downsample_y: %.2f", map_downsample_y_);  

  parameter_->declare_parameter("map_downsample_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_downsample_z = parameter_->get_parameter("map_downsample_z");
  map_downsample_z_ = map_downsample_z.as_double();
  RCLCPP_INFO(logger_->get_logger(), "map_downsample_z: %.2f", map_downsample_z_);  

  map_grid_min_ =
      std::min(
          std::min(map_downsample_x_, map_downsample_y_),
          map_downsample_z_);
  map_grid_max_ =
      std::max(
          std::max(map_downsample_x_, map_downsample_y_),
          map_downsample_z_);

  parameter_->declare_parameter("num_particles", rclcpp::ParameterValue(0));
  rclcpp::Parameter num_particles = parameter_->get_parameter("num_particles");
  num_particles_ = num_particles.as_int();
  RCLCPP_INFO(logger_->get_logger(), "num_particles: %d", num_particles_);

  parameter_->declare_parameter("resample_var_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter resample_var_x = parameter_->get_parameter("resample_var_x");
  resample_var_x_ = resample_var_x.as_double();
  RCLCPP_INFO(logger_->get_logger(), "resample_var_x: %.2f", resample_var_x_);

  parameter_->declare_parameter("resample_var_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter resample_var_y = parameter_->get_parameter("resample_var_y");
  resample_var_y_ = resample_var_y.as_double();
  RCLCPP_INFO(logger_->get_logger(), "resample_var_y: %.2f", resample_var_y_);

  parameter_->declare_parameter("resample_var_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter resample_var_z = parameter_->get_parameter("resample_var_z");
  resample_var_z_ = resample_var_z.as_double();
  RCLCPP_INFO(logger_->get_logger(), "resample_var_z: %.2f", resample_var_z_);

  parameter_->declare_parameter("resample_var_roll", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter resample_var_roll = parameter_->get_parameter("resample_var_roll");
  resample_var_roll_ = resample_var_roll.as_double();
  RCLCPP_INFO(logger_->get_logger(), "resample_var_roll: %.2f", resample_var_roll_);
  
  parameter_->declare_parameter("resample_var_pitch", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter resample_var_pitch = parameter_->get_parameter("resample_var_pitch");
  resample_var_pitch_ = resample_var_pitch.as_double();
  RCLCPP_INFO(logger_->get_logger(), "resample_var_pitch: %.2f", resample_var_pitch_);

  parameter_->declare_parameter("resample_var_yaw", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter resample_var_yaw = parameter_->get_parameter("resample_var_yaw");
  resample_var_yaw_ = resample_var_yaw.as_double();
  RCLCPP_INFO(logger_->get_logger(), "resample_var_yaw: %.2f", resample_var_yaw_);

  parameter_->declare_parameter("expansion_var_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter expansion_var_x = parameter_->get_parameter("expansion_var_x");
  expansion_var_x_ = expansion_var_x.as_double();
  RCLCPP_INFO(logger_->get_logger(), "expansion_var_x: %.2f", expansion_var_x_);
  
  parameter_->declare_parameter("expansion_var_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter expansion_var_y = parameter_->get_parameter("expansion_var_y");
  expansion_var_y_ = expansion_var_y.as_double();
  RCLCPP_INFO(logger_->get_logger(), "expansion_var_y: %.2f", expansion_var_y_);

  parameter_->declare_parameter("expansion_var_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter expansion_var_z = parameter_->get_parameter("expansion_var_z");
  expansion_var_z_ = expansion_var_z.as_double();
  RCLCPP_INFO(logger_->get_logger(), "expansion_var_z: %.2f", expansion_var_z_);

  parameter_->declare_parameter("expansion_var_roll", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter expansion_var_roll = parameter_->get_parameter("expansion_var_roll");
  expansion_var_roll_ = expansion_var_roll.as_double();
  RCLCPP_INFO(logger_->get_logger(), "expansion_var_roll: %.2f", expansion_var_roll_);
  
  parameter_->declare_parameter("expansion_var_pitch", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter expansion_var_pitch = parameter_->get_parameter("expansion_var_pitch");
  expansion_var_pitch_ = expansion_var_pitch.as_double();
  RCLCPP_INFO(logger_->get_logger(), "expansion_var_pitch: %.2f", expansion_var_pitch_);

  parameter_->declare_parameter("expansion_var_yaw", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter expansion_var_yaw = parameter_->get_parameter("expansion_var_yaw");
  expansion_var_yaw_ = expansion_var_yaw.as_double();
  RCLCPP_INFO(logger_->get_logger(), "expansion_var_yaw: %.2f", expansion_var_yaw_);

  parameter_->declare_parameter("match_ratio_thresh", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter match_ratio_thresh = parameter_->get_parameter("match_ratio_thresh");
  match_ratio_thresh_ = match_ratio_thresh.as_double();
  RCLCPP_INFO(logger_->get_logger(), "match_ratio_thresh: %.2f", match_ratio_thresh_);
  
  
  //@euc
  parameter_->declare_parameter("euc_cluster_distance", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter euc_cluster_distance = parameter_->get_parameter("euc_cluster_distance");
  euc_cluster_distance_ = euc_cluster_distance.as_double();
  RCLCPP_INFO(logger_->get_logger(), "euc_cluster_distance: %.2f", euc_cluster_distance_);

  parameter_->declare_parameter("euc_cluster_min_size", rclcpp::ParameterValue(0));
  rclcpp::Parameter euc_cluster_min_size = parameter_->get_parameter("euc_cluster_min_size");
  euc_cluster_min_size_ = euc_cluster_min_size.as_int();
  RCLCPP_INFO(logger_->get_logger(), "euc_cluster_min_size: %d", euc_cluster_min_size_);
  
  
  //@For normal
  parameter_->declare_parameter("knn_num_of_ground_normals", rclcpp::ParameterValue(0));
  rclcpp::Parameter knn_num_of_ground_normals = parameter_->get_parameter("knn_num_of_ground_normals");
  knn_num_of_ground_normals_ = knn_num_of_ground_normals.as_int();
  RCLCPP_INFO(logger_->get_logger(), "knn_num_of_ground_normals: %d", knn_num_of_ground_normals_);

  //The threshold to trigger particle measure
  parameter_->declare_parameter("update_min_d", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter update_min_d = parameter_->get_parameter("update_min_d");
  update_min_d_ = update_min_d.as_double();
  RCLCPP_INFO(logger_->get_logger(), "update_min_d: %.2f", update_min_d_);

  parameter_->declare_parameter("update_min_a", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter update_min_a = parameter_->get_parameter("update_min_a");
  update_min_a_ = update_min_a.as_double();
  RCLCPP_INFO(logger_->get_logger(), "update_min_a: %.2f", update_min_a_);

  parameter_->declare_parameter("odom_err_lin_lin", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter odom_err_lin_lin = parameter_->get_parameter("odom_err_lin_lin");
  odom_err_lin_lin_ = odom_err_lin_lin.as_double();
  RCLCPP_INFO(logger_->get_logger(), "odom_err_lin_lin: %.2f", odom_err_lin_lin_);
  
  parameter_->declare_parameter("odom_err_lin_ang", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter odom_err_lin_ang = parameter_->get_parameter("odom_err_lin_ang");
  odom_err_lin_ang_ = odom_err_lin_ang.as_double();
  RCLCPP_INFO(logger_->get_logger(), "odom_err_lin_ang: %.2f", odom_err_lin_ang_);

  parameter_->declare_parameter("odom_err_ang_lin", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter odom_err_ang_lin = parameter_->get_parameter("odom_err_ang_lin");
  odom_err_ang_lin_ = odom_err_ang_lin.as_double();
  RCLCPP_INFO(logger_->get_logger(), "odom_err_ang_lin: %.2f", odom_err_ang_lin_);

  parameter_->declare_parameter("odom_err_ang_ang", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter odom_err_ang_ang = parameter_->get_parameter("odom_err_ang_ang");
  odom_err_ang_ang_ = odom_err_ang_ang.as_double();
  RCLCPP_INFO(logger_->get_logger(), "odom_err_ang_ang: %.2f", odom_err_ang_ang_);

  parameter_->declare_parameter("odom_err_integ_lin_tc", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter odom_err_integ_lin_tc = parameter_->get_parameter("odom_err_integ_lin_tc");
  odom_err_integ_lin_tc_ = odom_err_integ_lin_tc.as_double();
  RCLCPP_INFO(logger_->get_logger(), "odom_err_integ_lin_tc: %.2f", odom_err_integ_lin_tc_);

  parameter_->declare_parameter("odom_err_integ_ang_tc", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter odom_err_integ_ang_tc = parameter_->get_parameter("odom_err_integ_ang_tc");
  odom_err_integ_ang_tc_ = odom_err_integ_ang_tc.as_double();
  RCLCPP_INFO(logger_->get_logger(), "odom_err_integ_ang_tc: %.2f", odom_err_integ_ang_tc_);
  
  parameter_->declare_parameter("lpf_step", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter lpf_step = parameter_->get_parameter("lpf_step");
  lpf_step_ = lpf_step.as_double();
  RCLCPP_INFO(logger_->get_logger(), "lpf_step: %.2f", lpf_step_);

  parameter_->declare_parameter("acc_lpf_step", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter acc_lpf_step = parameter_->get_parameter("acc_lpf_step");
  acc_lpf_step_ = acc_lpf_step.as_double();
  RCLCPP_INFO(logger_->get_logger(), "acc_lpf_step: %.2f", acc_lpf_step_);
  
  parameter_->declare_parameter("jump_dist", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter jump_dist = parameter_->get_parameter("jump_dist");
  jump_dist_ = jump_dist.as_double();
  RCLCPP_INFO(logger_->get_logger(), "jump_dist: %.2f", jump_dist_);

  parameter_->declare_parameter("jump_ang", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter jump_ang = parameter_->get_parameter("jump_ang");
  jump_ang_ = jump_ang.as_double();
  RCLCPP_INFO(logger_->get_logger(), "jump_ang: %.2f", jump_ang_);

  parameter_->declare_parameter("bias_var_dist", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter bias_var_dist = parameter_->get_parameter("bias_var_dist");
  bias_var_dist_ = bias_var_dist.as_double();
  RCLCPP_INFO(logger_->get_logger(), "bias_var_dist: %.2f", bias_var_dist_);

  parameter_->declare_parameter("bias_var_ang", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter bias_var_ang = parameter_->get_parameter("bias_var_ang");
  bias_var_ang_ = bias_var_ang.as_double();
  RCLCPP_INFO(logger_->get_logger(), "bias_var_ang: %.2f", bias_var_ang_);

  parameter_->declare_parameter("tf_tolerance", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter tf_tolerance = parameter_->get_parameter("tf_tolerance");
  tf_tolerance_ = tf2::durationFromSec(tf_tolerance.as_double());
  RCLCPP_INFO(logger_->get_logger(), "tf_tolerance: %.2f", tf_tolerance.as_double());

  parameter_->declare_parameter("publish_tf", rclcpp::ParameterValue(false));
  rclcpp::Parameter publish_tf = parameter_->get_parameter("publish_tf");
  publish_tf_ = publish_tf.as_bool();
  RCLCPP_INFO(logger_->get_logger(), "publish_tf: %d", publish_tf_);
  

  double x, y, z;
  double roll, pitch, yaw;
  double v_x, v_y, v_z;
  double v_roll, v_pitch, v_yaw;

  parameter_->declare_parameter("init_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_x = parameter_->get_parameter("init_x");
  x = init_x.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_x: %.2f", x);

  parameter_->declare_parameter("init_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_y = parameter_->get_parameter("init_y");
  y = init_y.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_y: %.2f", y);

  parameter_->declare_parameter("init_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_z = parameter_->get_parameter("init_z");
  z = init_z.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_z: %.2f", z);

  parameter_->declare_parameter("init_roll", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_roll = parameter_->get_parameter("init_roll");
  roll = init_roll.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_roll: %.2f", roll);

  parameter_->declare_parameter("init_pitch", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_pitch = parameter_->get_parameter("init_pitch");
  pitch = init_pitch.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_pitch: %.2f", pitch);

  parameter_->declare_parameter("init_yaw", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_yaw = parameter_->get_parameter("init_yaw");
  yaw = init_yaw.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_yaw: %.2f", yaw);

  parameter_->declare_parameter("init_var_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_var_x = parameter_->get_parameter("init_var_x");
  v_x = init_var_x.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_var_x: %.2f", v_x);

  parameter_->declare_parameter("init_var_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_var_y = parameter_->get_parameter("init_var_y");
  v_y = init_var_y.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_var_y: %.2f", v_y);

  parameter_->declare_parameter("init_var_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_var_z = parameter_->get_parameter("init_var_z");
  v_z = init_var_z.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_var_z: %.2f", v_z);

  parameter_->declare_parameter("init_var_roll", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_var_roll = parameter_->get_parameter("init_var_roll");
  v_roll = init_var_roll.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_var_roll: %.2f", v_roll);

  parameter_->declare_parameter("init_var_pitch", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_var_pitch = parameter_->get_parameter("init_var_pitch");
  v_pitch = init_var_pitch.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_var_pitch: %.2f", v_pitch);

  parameter_->declare_parameter("init_var_yaw", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter init_var_yaw = parameter_->get_parameter("init_var_yaw");
  v_yaw = init_var_yaw.as_double();
  RCLCPP_INFO(logger_->get_logger(), "init_var_yaw: %.2f", v_yaw);

  
  initial_pose_ = State6DOF(
      Vec3(x, y, z),
      Quat(Vec3(roll, pitch, yaw)));
  initial_pose_std_ = State6DOF(
      Vec3(v_x, v_y, v_z),
      Vec3(v_roll, v_pitch, v_yaw));
  

}
}  // namespace mcl_3dl
