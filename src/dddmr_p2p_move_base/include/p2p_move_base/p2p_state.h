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
#include "rclcpp/rclcpp.hpp"

/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <map>

#include <std_msgs/msg/string.hpp>

namespace p2p_move_base
{

class State{

  public:
    
    State(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
              const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter);

    bool setDecision(std::string m_decision);

    std::string getCurrentDecision(){return current_decision_;}
    std::string getLastDecision(){return last_decision_;}
    bool isCurrentDecision(std::string m_decision);

    void initialParams(geometry_msgs::msg::TransformStamped robot_curent_pose, rclcpp::Time current_time);
    
    rclcpp::Time last_valid_plan_;

    geometry_msgs::msg::PoseStamped current_goal_;
    double planner_patience_;
    double controller_frequency_;
    
    /*recovery params*/
    int no_plan_recovery_count_, no_plan_retry_num_;
    double oscillation_distance_, oscillation_angle_, oscillation_patience_, controller_patience_, waiting_patience_;
    rclcpp::Time last_oscillation_reset_;
    rclcpp::Time last_valid_control_;

    /*store global pose*/
    geometry_msgs::msg::TransformStamped global_pose_;
    geometry_msgs::msg::TransformStamped oscillation_pose_;

    rclcpp::Time last_goal_received_;

    rclcpp::Time waiting_time_;

    double getDistance(geometry_msgs::msg::TransformStamped& a, geometry_msgs::msg::TransformStamped& b);
    double getAngle(geometry_msgs::msg::TransformStamped& a, geometry_msgs::msg::TransformStamped& b);

    bool use_twist_stamped_;
    
    bool use_position_control_at_goal_;
    std::string main_trajectory_generator_;

  private:

    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_;
    std::string current_decision_;
    std::string last_decision_;

};


}//end of name space