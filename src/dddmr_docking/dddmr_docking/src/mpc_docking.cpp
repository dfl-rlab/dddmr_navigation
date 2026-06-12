#include "mpc_docking.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dddmr_docking {

MPCDocking::MPCDocking(const std::string &name) : Node(name) {

  clock_ = this->get_clock();

  //@Start to load cameras
  this->declare_parameter("cameras", rclcpp::PARAMETER_STRING_ARRAY);
  this->get_parameter("cameras", cameras_);
  for(auto i=cameras_.begin(); i!=cameras_.end(); i++){
    RCLCPP_INFO(this->get_logger(), "Use camera: %s", (*i).c_str());
    bool record_tags = false;
    apriltag_tracking_map_[(*i)] = std::make_shared<dddmr_docking::AprilTagTrtPoseDetector>(this, (*i), record_tags);
  }

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<dddmr_sys_core::action::TagDocking>(
    this,
    "tag_docking",
    std::bind(&MPCDocking::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MPCDocking::handle_cancel, this, std::placeholders::_1),
    std::bind(&MPCDocking::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_
  );

  RCLCPP_INFO(
      this->get_logger(),
      "MPCDocking action server initialized with Tag Tracking and Trajectory Generator.");
}

MPCDocking::~MPCDocking() {}

rclcpp_action::GoalResponse MPCDocking::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::TagDocking::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request for TagDocking");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MPCDocking::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MPCDocking::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle)
{
  if (current_handle_ != nullptr && current_handle_->is_active()) {
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::TagDocking::Result>();
    result->succeed = false;
    current_handle_->abort(result);
  }
  
  current_handle_ = goal_handle;

  std::thread{std::bind(&MPCDocking::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void MPCDocking::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle)
{
  rclcpp::Rate loop_rate(20);
  auto result = std::make_shared<dddmr_sys_core::action::TagDocking::Result>();

  rclcpp::Time tracking_start_time = clock_->now();
  rclcpp::Time success_start_time = clock_->now();
  
  //@ Activate Tag Detector
  for(auto i=apriltag_tracking_map_.begin();i!=apriltag_tracking_map_.end();i++){
    i->second->startDetection();
  }

  RCLCPP_INFO(this->get_logger(), "Executing goal");


  while(rclcpp::ok() && goal_handle->is_active()) {

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel_pub_->publish(cmd_vel);
      return;
    }

    loop_rate.sleep();
  }
}

} // namespace dddmr_docking
