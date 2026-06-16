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
#include <global_planner/dynamic_window_aware_global_planner.h>

using namespace std::chrono_literals;

namespace global_planner
{

DWA_GlobalPlanner::DWA_GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse DWA_GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DWA_GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DWA_GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DWA_GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void DWA_GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d, 
                                    const std::shared_ptr<global_planner::GlobalPlanner>& global_planner){

  perception_3d_ros_ = perception_3d;
  global_planner_ = global_planner;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pcl_global_path_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  declare_parameter("look_ahead_distance", rclcpp::ParameterValue(5.0));
  this->get_parameter("look_ahead_distance", look_ahead_distance_);
  RCLCPP_INFO(this->get_logger(), "look_ahead_distance: %.2f", look_ahead_distance_);    
  declare_parameter("recompute_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("recompute_frequency", recompute_frequency_);
  RCLCPP_INFO(this->get_logger(), "recompute_frequency: %.2f", recompute_frequency_);     
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("awared_global_path", 1);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;

  auto loop_time = std::chrono::milliseconds(int(1000/recompute_frequency_));
  threading_timer_ = this->create_wall_timer(loop_time, std::bind(&DWA_GlobalPlanner::determineDWAPlan, this), action_server_group_);
  threading_timer_->cancel();
  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_dwa_plan",
    std::bind(&DWA_GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DWA_GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&DWA_GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

DWA_GlobalPlanner::~DWA_GlobalPlanner(){
  action_server_global_planner_.reset();
}

bool DWA_GlobalPlanner::isNewGoal(){
  if(new_goal_.pose.position.x==current_goal_.pose.position.x && 
      new_goal_.pose.position.y==current_goal_.pose.position.y && 
        new_goal_.pose.position.z==current_goal_.pose.position.z && 
          new_goal_.pose.orientation.x==current_goal_.pose.orientation.x && 
            new_goal_.pose.orientation.y==current_goal_.pose.orientation.y && 
              new_goal_.pose.orientation.z==current_goal_.pose.orientation.z && 
                new_goal_.pose.orientation.w==current_goal_.pose.orientation.w)
     {
      
      return false;
     }
  
  RCLCPP_INFO(this->get_logger(), "Received new goal at: %.2f, %.2f, %.2f", new_goal_.pose.position.x, new_goal_.pose.position.y, new_goal_.pose.position.z);
  return true;
}

void DWA_GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  new_goal_ = goal_handle->get_goal()->goal;

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    return;
  }

  if(!goal_handle->get_goal()->activate_threading){
    threading_timer_->cancel();
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->succeed(result);
    return;
  }
  
  if(isNewGoal()){
    
    geometry_msgs::msg::PoseStamped start;
    perception_3d_ros_->getGlobalPose(start);

    global_path_ = global_planner_->makeROSPlan(start, goal_handle->get_goal()->goal);
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    result->path = global_path_;

    if(global_path_.poses.empty()){
      goal_handle->abort(result);
    }
    else{
      current_goal_ = new_goal_;
      result->path = global_path_;
      goal_handle->succeed(result);
      
      //@move global plan to pcl
      pcl_global_path_->points.clear();
      for(auto it=global_path_.poses.begin(); it!=global_path_.poses.end(); it++){
        pcl::PointXYZ pt;
        pt.x = (*it).pose.position.x; pt.y = (*it).pose.position.y; pt.z = (*it).pose.position.z;
        pcl_global_path_->push_back(pt);
      }
      //@ generate kd-tree
      kdtree_global_path_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
      kdtree_global_path_->setInputCloud(pcl_global_path_); 

      threading_timer_->reset();
    }
    pub_path_->publish(global_path_);
    global_dwa_path_.poses.clear();
  }
  else{
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    result->path = global_dwa_path_;
    goal_handle->succeed(result);
    pub_path_->publish(global_dwa_path_);
  }
  
}

void DWA_GlobalPlanner::determineDWAPlan(){

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);
  pcl::PointXYZ start_pt;
  start_pt.x = start.pose.position.x; start_pt.y = start.pose.position.y; start_pt.z = start.pose.position.z;
  std::vector<float> resultant_distances;
  std::vector<int> indices;
  kdtree_global_path_->nearestKSearch(start_pt, 1, indices, resultant_distances);

  if(indices.empty()){
    RCLCPP_INFO(this->get_logger(), "k-NN search fail, something wrong.");
    return;
  }
  //@protect kdtree_ground
  std::unique_lock<std::recursive_mutex> lock(perception_3d_ros_->getSharedDataPtr()->ground_kdtree_cb_mutex_);
  //@check is dwa goal being blocked, if yes, shift ahead distance 1.0 m
  bool dwa_goal_clear = false;
  double look_ahead_step = 0.0;
  int dwa_pivot = 0;
  while(rclcpp::ok() && !dwa_goal_clear){

    double accumulative_distance = 0.0;
    int pivot = indices[0];
    pcl::PointXYZ last_point = pcl_global_path_->points[pivot];
    while(accumulative_distance<look_ahead_distance_+look_ahead_step && pivot<pcl_global_path_->points.size()-1){
      pcl::PointXYZ current_point = pcl_global_path_->points[pivot];
      double dx = current_point.x - last_point.x;
      double dy = current_point.y - last_point.y;
      double dz = current_point.z - last_point.z;
      accumulative_distance += sqrt(dx*dx + dy*dy + dz*dz);
      last_point = current_point;
      pivot++;
    }

    if(pivot>=pcl_global_path_->points.size()-1){
      dwa_pivot = pcl_global_path_->points.size()-1;
      if(recompute_frequency_>2.0)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "DWA goal reach the global path end at: %.2f, %.2f, %.2f", pcl_global_path_->points[dwa_pivot].x, pcl_global_path_->points[dwa_pivot].y, pcl_global_path_->points[dwa_pivot].z);
      else
        RCLCPP_INFO(this->get_logger(), "DWA goal reach the global path end at: %.2f, %.2f, %.2f", pcl_global_path_->points[dwa_pivot].x, pcl_global_path_->points[dwa_pivot].y, pcl_global_path_->points[dwa_pivot].z);
      break;
    }

    if(look_ahead_distance_+look_ahead_step>100.0){
      dwa_pivot = pcl_global_path_->points.size()-1;
      RCLCPP_INFO(this->get_logger(), "Look ahead distance reach unrealistic distance.");
      break;      
    }
    //check nothing is around dwa goal
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZI ipt;
    ipt.x = pcl_global_path_->points[pivot].x;
    ipt.y = pcl_global_path_->points[pivot].y;
    ipt.z = pcl_global_path_->points[pivot].z;
    perception_3d_ros_->getSharedDataPtr()->kdtree_ground_->radiusSearch(ipt, 0.25, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    
    double inscribed_radius = perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
    if(pointIdxRadiusSearch.empty()){
      look_ahead_step+=1.0;
      dwa_goal_clear = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000,  "No ground is found for DWA goal at: %.2f, %.2f, %.2f", pcl_global_path_->points[pivot].x, pcl_global_path_->points[pivot].y, pcl_global_path_->points[pivot].z);
    }
    else{
      for(auto it=pointIdxRadiusSearch.begin();it!=pointIdxRadiusSearch.end();it++){
        //@ dGraphValue is the distance to lethal
        double dGraphValue = perception_3d_ros_->get_min_dGraphValue((*it));
        /*This is for lethal*/
        if(dGraphValue<inscribed_radius){
          look_ahead_step+=1.0;
          dwa_goal_clear = false;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "DWA goal is blocked at: %.2f, %.2f, %.2f", pcl_global_path_->points[pivot].x, pcl_global_path_->points[pivot].y, pcl_global_path_->points[pivot].z);
          break;
        }
        else{
          dwa_goal_clear = true;
        }
      }
    }
    dwa_pivot = pivot;
  }

  geometry_msgs::msg::PoseStamped dwa_goal;
  dwa_goal.header.frame_id = global_frame_;
  dwa_goal.pose.position.x = pcl_global_path_->points[dwa_pivot].x;
  dwa_goal.pose.position.y = pcl_global_path_->points[dwa_pivot].y;
  dwa_goal.pose.position.z = pcl_global_path_->points[dwa_pivot].z;
  
  RCLCPP_DEBUG(this->get_logger(), "DWA pivot: %d is at %.2f, %.2f, %.2f with start: %.2f, %.2f, %.2f", 
        dwa_pivot, dwa_goal.pose.position.x, dwa_goal.pose.position.y, dwa_goal.pose.position.z,
        start.pose.position.x, start.pose.position.y, start.pose.position.z);

  nav_msgs::msg::Path dwa_path = global_planner_->makeROSPlan(start, dwa_goal);
  for(size_t i=dwa_pivot; i<pcl_global_path_->points.size(); i++){
    dwa_path.poses.push_back(global_path_.poses[i]);
  }
  dwa_path.poses.push_back(global_path_.poses.back());
  global_dwa_path_ = dwa_path;
}

}