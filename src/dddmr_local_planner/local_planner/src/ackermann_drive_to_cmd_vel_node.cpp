#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

class AckermannDriveToCmdVelNode : public rclcpp::Node {
 public:
  AckermannDriveToCmdVelNode() : Node("ackermann_drive_to_cmd_vel") {
    // --- Ackermann-to-Twist converter params ---
    input_topic_ = declare_parameter<std::string>("input_topic", "/ackermann_drive_cmd");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel");
    wheelbase_ = declare_parameter<double>("wheelbase", 0.2255);

    // --- Steering visualization params ---
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "/ackermann_feedback");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    track_width_ = declare_parameter<double>("track_width", 0.30);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.08);
    tire_length_ = declare_parameter<double>("tire_length", 0.18);
    tire_width_ = declare_parameter<double>("tire_width", 0.06);

    if (wheelbase_ <= 0.0) {
      throw std::runtime_error("wheelbase must be greater than zero");
    }

    // --- Ackermann drive → cmd_vel converter ---
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        input_topic_, 10,
        std::bind(&AckermannDriveToCmdVelNode::driveCallback, this, std::placeholders::_1));

    // --- Feedback → steering marker visualization ---
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "steering_markers", 1);
    feedback_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        feedback_topic_, 10,
        std::bind(&AckermannDriveToCmdVelNode::feedbackCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Converting %s → %s (wheelbase=%.4f m)",
                input_topic_.c_str(), output_topic_.c_str(), wheelbase_);
    RCLCPP_INFO(get_logger(), "Steering viz from %s (track=%.3f m, frame=%s)",
                feedback_topic_.c_str(), track_width_, base_frame_.c_str());
  }

 private:
  // Convert AckermannDriveStamped command to Twist.
  void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = msg->drive.speed;
    cmd_vel.angular.z = msg->drive.speed * std::tan(msg->drive.steering_angle) / wheelbase_;
    cmd_vel_pub_->publish(cmd_vel);
  }

  // Publish steering direction arrows from ackermann feedback.
  void feedbackCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    if (marker_pub_->get_subscription_count() == 0) {
      return;
    }

    const double steer = msg->drive.steering_angle;
    const rclcpp::Time stamp = msg->header.stamp.sec == 0
        ? this->get_clock()->now() : rclcpp::Time(msg->header.stamp);

    // Ackermann inner/outer steering angles (bicycle model approximation).
    // For visualization we use the same angle on both wheels for simplicity.
    const double half_track = track_width_ / 2.0;

    visualization_msgs::msg::MarkerArray markers;

    // --- Left front wheel ---
    markers.markers.push_back(
        makeWheelMarker(0, stamp, wheelbase_, half_track, steer));

    // --- Right front wheel ---
    markers.markers.push_back(
        makeWheelMarker(1, stamp, wheelbase_, -half_track, steer));

    marker_pub_->publish(markers);
  }

  visualization_msgs::msg::Marker makeWheelMarker(
      int id, const rclcpp::Time& stamp,
      double x_offset, double y_offset, double steer_angle) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.header.stamp = stamp;
    m.ns = "steering";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    // Cube center at wheel position.
    m.pose.position.x = x_offset;
    m.pose.position.y = y_offset;
    m.pose.position.z = wheel_radius_;

    // Rotate cube by steering angle around Z axis.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, steer_angle);
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    // Tire shape: configurable via params.
    m.scale.x = tire_length_;            // tire length (contact patch direction)
    m.scale.y = tire_width_;             // tire width (thickness)
    m.scale.z = wheel_radius_ * 2.0;    // tire height (diameter)

    // Front wheel color: green.
    m.color.r = 0.2f;
    m.color.g = 1.0f;
    m.color.b = 0.2f;
    m.color.a = 0.85f;

    m.lifetime = rclcpp::Duration::from_seconds(0.3);
    return m;
  }

  // Command converter members.
  std::string input_topic_;
  std::string output_topic_;
  double wheelbase_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;

  // Steering visualization members.
  std::string feedback_topic_;
  std::string base_frame_;
  double track_width_;
  double wheel_radius_;
  double tire_length_;
  double tire_width_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr feedback_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannDriveToCmdVelNode>());
  rclcpp::shutdown();
  return 0;
}
