#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class AckermannDriveToCmdVelNode : public rclcpp::Node {
 public:
  AckermannDriveToCmdVelNode() : Node("ackermann_drive_to_cmd_vel") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/ackermann_drive_cmd");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel");
    wheelbase_ = declare_parameter<double>("wheelbase", 0.2255);

    if (wheelbase_ <= 0.0) {
      throw std::runtime_error("wheelbase must be greater than zero");
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        input_topic_, 10,
        std::bind(&AckermannDriveToCmdVelNode::driveCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Converting %s to %s (wheelbase=%.4f m)",
                input_topic_.c_str(), output_topic_.c_str(), wheelbase_);
  }

 private:
  void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = msg->drive.speed;
    cmd_vel.angular.z = msg->drive.speed * std::tan(msg->drive.steering_angle) / wheelbase_;
    cmd_vel_pub_->publish(cmd_vel);
  }

  std::string input_topic_;
  std::string output_topic_;
  double wheelbase_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannDriveToCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
