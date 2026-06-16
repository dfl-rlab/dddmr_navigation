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
#include <p2p_move_base/p2p_global_plan_manager.h>
namespace p2p_move_base
{
P2PGlobalPlanManager::P2PGlobalPlanManager(std::string name) : Node(name), name_(name), got_first_goal_(false){
  clock_ = this->get_clock();
}

P2PGlobalPlanManager::~P2PGlobalPlanManager(){
  tf2Buffer_.reset();
  tfl_.reset();
}

void P2PGlobalPlanManager::initial(){

  this->declare_parameter("global_planner_action_name", rclcpp::ParameterValue("get_plan"));
  this->get_parameter("global_planner_action_name", global_planner_action_name_);
  RCLCPP_INFO(this->get_logger(), "P2P global plan manager uses \033[1;32m%s\033[0m service to query global plan.", global_planner_action_name_.c_str());

  this->declare_parameter("global_plan_query_frequency", rclcpp::ParameterValue(5.0));
  this->get_parameter("global_plan_query_frequency", global_plan_query_frequency_);
  RCLCPP_INFO(this->get_logger(), "global_plan_query_frequency: %.2f", global_plan_query_frequency_);

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);
  
  global_planner_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  global_planner_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::GetPlan>(
      this,
      global_planner_action_name_, global_planner_client_group_);
  
  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if(global_plan_query_frequency_>0.0){
    auto loop_time = std::chrono::milliseconds(int(1000/global_plan_query_frequency_));
    loop_timer_ = this->create_wall_timer(loop_time, std::bind(&P2PGlobalPlanManager::queryThread, this), timer_group_);
    stop();
  }
  else{
    auto loop_time = std::chrono::milliseconds(1000000000);
    loop_timer_ = this->create_wall_timer(loop_time, std::bind(&P2PGlobalPlanManager::queryThread, this), timer_group_);
    stop();
  }

}

void P2PGlobalPlanManager::resume(){
  global_path_.poses.clear();
  is_planning_ = false;
  loop_timer_->reset();
  RCLCPP_INFO(this->get_logger(), "Global plan manager is resumed");
}

void P2PGlobalPlanManager::stop(){
  loop_timer_->cancel();
  
  if(got_first_goal_){
    auto goal_msg = dddmr_sys_core::action::GetPlan::Goal();
    goal_msg.activate_threading = false;
    auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&P2PGlobalPlanManager::global_planner_client_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&P2PGlobalPlanManager::global_planner_client_result_callback, this, std::placeholders::_1);
    global_planner_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    got_first_goal_ = false;
  }

  RCLCPP_INFO(this->get_logger(), "Global plan manager is stopped");
}

void P2PGlobalPlanManager::queryThread(){

  std::unique_lock<std::mutex> lock(access_);
  
  if(is_planning_)
    return;
  
  if(!got_first_goal_)
    return;

  auto goal_msg = dddmr_sys_core::action::GetPlan::Goal();
  goal_msg.goal = goal_;
  goal_msg.activate_threading = true;

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::GetPlan>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&P2PGlobalPlanManager::global_planner_client_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&P2PGlobalPlanManager::global_planner_client_result_callback, this, std::placeholders::_1);
  
  is_planning_ = true;
  global_planner_client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void P2PGlobalPlanManager::global_planner_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::GetPlan>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    if(global_plan_query_frequency_>2)
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock_, 5000, "Goal was rejected by: %s", global_planner_action_name_.c_str());
    else
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by: %s", global_planner_action_name_.c_str());
  } else {
    if(global_plan_query_frequency_>2)
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Goal accepted by: %s, waiting for result", global_planner_action_name_.c_str());
    else
      RCLCPP_INFO(this->get_logger(), "Goal accepted by: %s, waiting for result", global_planner_action_name_.c_str()); 
  }
}

void P2PGlobalPlanManager::global_planner_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::GetPlan>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      //RCLCPP_INFO(this->get_logger(), "Global Planner ---> %s: Global plan is found", global_planner_action_name_.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Global Planner ---> %s: Goal was aborted", global_planner_action_name_.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Global Planner ---> %s: Goal was canceled", global_planner_action_name_.c_str());
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Global Planner ---> %s: Unknown result code", global_planner_action_name_.c_str());
      break;
  }
  global_path_ = result.result->path;
  is_planning_ = false;
}

void P2PGlobalPlanManager::setGoal(const geometry_msgs::msg::PoseStamped& goal){
  std::unique_lock<std::mutex> lock(access_);
  goal_ = goal;
  got_first_goal_ = true;
}

bool P2PGlobalPlanManager::hasPlan(){
  std::unique_lock<std::mutex> lock(access_);
  if(!is_planning_ && !global_path_.poses.empty())
    return true;
  return false;
}

void P2PGlobalPlanManager::copyPlan(std::vector<geometry_msgs::msg::PoseStamped>& plan){
  std::unique_lock<std::mutex> lock(access_);
  for(int i=0;i<global_path_.poses.size();i++){
    plan.push_back(global_path_.poses[i]);
  }
}

}