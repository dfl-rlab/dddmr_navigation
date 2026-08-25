#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"

using namespace std::chrono_literals;

class LaserScan2PointCloud : public rclcpp::Node
{
public:
  explicit LaserScan2PointCloud(std::string name);
  void cbLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::CallbackGroup::SharedPtr cbs_group_;

  std::string topic_;
  double range_min_;
  double range_max_;

  laser_geometry::LaserProjection projector_;
};

LaserScan2PointCloud::LaserScan2PointCloud(std::string name) : Node(name)
{
  this->declare_parameter("topic", rclcpp::ParameterValue("/scan"));
  this->get_parameter("topic", topic_);
  RCLCPP_INFO(this->get_logger(), "topic: %s", topic_.c_str());

  this->declare_parameter("range_min", rclcpp::ParameterValue(-1.0));
  this->get_parameter("range_min", range_min_);
  RCLCPP_INFO(this->get_logger(), "range_min: %.2f", range_min_);

  this->declare_parameter("range_max", rclcpp::ParameterValue(-1.0));
  this->get_parameter("range_max", range_max_);
  RCLCPP_INFO(this->get_logger(), "range_max: %.2f", range_max_);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_from_scan", 2);

  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
    std::bind(&LaserScan2PointCloud::cbLaserScan, this, std::placeholders::_1), sub_options);
}

void LaserScan2PointCloud::cbLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  sensor_msgs::msg::LaserScan scan_msg = *msg;

  if (range_min_ > 0.0) {
    scan_msg.range_min = static_cast<float>(range_min_);
  }
  if (range_max_ > 0.0) {
    scan_msg.range_max = static_cast<float>(range_max_);
  }

  sensor_msgs::msg::PointCloud2 output;
  projector_.projectLaser(scan_msg, output);

  cloud_pub_->publish(output);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  LaserScan2PointCloud node("laserscan2pointcloud");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node.get_node_base_interface());
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
