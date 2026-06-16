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
#ifndef DDDMR_SYS_CORE_ENUM_STATES_H
#define DDDMR_SYS_CORE_ENUM_STATES_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>

#include "rclcpp_action/rclcpp_action.hpp"
#include "dddmr_sys_core/action/p_to_p_move_base.hpp"

namespace dddmr_sys_core {

  typedef rclcpp_action::Server<dddmr_sys_core::action::PToPMoveBase> PToPMoveBaseActionServer;

  enum PlannerState {
    TF_FAIL,
    PRUNE_PLAN_FAIL,
    ALL_TRAJECTORIES_FAIL,
    PERCEPTION_MALFUNCTION,
    TRAJECTORY_FOUND,
    PATH_BLOCKED_WAIT,
    PATH_BLOCKED_REPLANNING,
    CONFIGURATION_ERROR
  };

  enum RecoveryState {
    RECOVERY_BEHAVIOR_NOT_FOUND,
    INTERRUPT_BY_CANCEL,
    INTERRUPT_BY_NEW_GOAL,
    RECOVERY_DONE,
    RECOVERY_FAIL
  };

  class dddmr_enum_states
  {
  private:
    /* data */
  public:
    dddmr_enum_states();
    ~dddmr_enum_states();
  };
  
};  // namespace dddmr_sys_core

#endif  // dddmr_sys_core_BASE_P2P_LOCAL_PLANNER_H
