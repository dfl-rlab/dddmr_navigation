/*
 * Copyright (c) 2016-2018, the mcl_3dl authors
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
#ifndef MCL_3DL_PARAMETERS_H
#define MCL_3DL_PARAMETERS_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <mcl_3dl/quat.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

namespace mcl_3dl
{
class Parameters
{

private:
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_;

public:
  Parameters(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
              const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter);
  double map_downsample_x_;
  double map_downsample_y_;
  double map_downsample_z_;
  double map_grid_min_;
  double map_grid_max_;
  double global_localization_grid_;
  int global_localization_div_yaw_;
  double resample_var_x_;
  double resample_var_y_;
  double resample_var_z_;
  double resample_var_roll_;
  double resample_var_pitch_;
  double resample_var_yaw_;
  double expansion_var_x_;
  double expansion_var_y_;
  double expansion_var_z_;
  double expansion_var_roll_;
  double expansion_var_pitch_;
  double expansion_var_yaw_;
  double match_ratio_thresh_;
  double jump_dist_;
  double jump_ang_;
  double odom_err_lin_lin_;
  double odom_err_lin_ang_;
  double odom_err_ang_lin_;
  double odom_err_ang_ang_;
  double euc_cluster_distance_;
  int euc_cluster_min_size_;

  std::chrono::seconds map_update_interval_;
  int num_particles_;
  double match_output_dist_;
  double bias_var_dist_;
  double bias_var_ang_;
  double odom_err_integ_lin_tc_;
  double odom_err_integ_ang_tc_;
  tf2::Duration tf_tolerance_;
  double lpf_step_;
  double acc_lpf_step_;
  bool publish_tf_;
  std::map<std::string, std::string> frame_ids_;
  State6DOF initial_pose_;
  State6DOF initial_pose_std_;

  int knn_num_of_ground_normals_;
  double update_min_d_, update_min_a_;

};
}  // namespace mcl_3dl

#endif  // MCL_3DL_PARAMETERS_H
