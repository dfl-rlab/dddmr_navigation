#ifndef DDDMR_DOCKING__MPC_DOCKING_HPP_
#define DDDMR_DOCKING__MPC_DOCKING_HPP_

#include "apriltag_trt_pose_detector.hpp"
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "dddmr_sys_core/action/tag_docking.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace dddmr_docking {

class MPCDocking : public rclcpp::Node {
public:
  explicit MPCDocking(const std::string &name);
  ~MPCDocking();

private:
  
  rclcpp::Clock::SharedPtr clock_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const dddmr_sys_core::action::TagDocking::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle);

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle);

  void executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> goal_handle);

  rclcpp_action::Server<dddmr_sys_core::action::TagDocking>::SharedPtr action_server_;
  rclcpp::CallbackGroup::SharedPtr action_server_group_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::TagDocking>> current_handle_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::vector<std::string> cameras_;
  std::map<std::string, std::shared_ptr<dddmr_docking::AprilTagTrtPoseDetector>> apriltag_tracking_map_;
};

} // namespace dddmr_docking

#endif // DDDMR_DOCKING__MPC_DOCKING_HPP_
