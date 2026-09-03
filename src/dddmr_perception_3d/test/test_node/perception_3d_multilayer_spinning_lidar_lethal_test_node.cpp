#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rcpputils/asserts.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std::chrono_literals;

struct CheckPoint {
  std::string name;
  pcl::PointXYZ point;
};

class P3dMplLaserscan : public rclcpp::Node {

public:

  P3dMplLaserscan(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("perception_3d_multilayer_spinning_lidar_lethal_test_node",
           rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {
    
    clock_ = this->get_clock();

    loadCheckPoints();

    //@Initialize transform listener and broadcaster
    tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      tf_listener_group_);
    tf2Buffer_->setCreateTimerInterface(timer_interface);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

    sub_front_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "front_cloud", 2,
        std::bind(&P3dMplLaserscan::cbPC, this, std::placeholders::_1));

    sub_lethal_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "perception_3d_global/front_lidar/lethal", 2,
        std::bind(&P3dMplLaserscan::cbLethal, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        100ms, std::bind(&P3dMplLaserscan::testCb, this));

    RCLCPP_INFO(this->get_logger(), "P3dMplLaserscan: %s has been started.", this->get_name());



  }

  bool is_done_ = false;
  bool is_passed_ = false;

private:
  
  enum class State {
    WAIT_B2M_TF,
    WAIT_BAG_DONE,
    CHECK_RESULT,
    SUCCEED,
    FAIL
  };
  
  rclcpp::Clock::SharedPtr clock_;
  
  std::vector<CheckPoint> check_points_;
  double search_radius_ = 0.2;

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_lethal_;

  rclcpp::TimerBase::SharedPtr timer_;
  State current_state_ = State::WAIT_B2M_TF;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_front_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lethal_cloud_;

  rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
  
  std::chrono::system_clock::time_point latest_pc_time_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  void cbPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    latest_pc_time_ = std::chrono::system_clock::now();
  }

  void cbLethal(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_msg);
    kdtree_lethal_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_lethal_->setInputCloud(pcl_msg);
  }

  void loadCheckPoints() {
    check_points_.clear();

    if (!this->has_parameter("search_radius")) {
      this->declare_parameter("search_radius", rclcpp::ParameterValue(0.2));
    }
    this->get_parameter("search_radius", search_radius_);

    const std::string prefix = "check_points.";
    const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();

    for (const auto & [key, val] : overrides) {
      if (key.rfind(prefix, 0) == 0) {
        std::string point_name = key.substr(prefix.length());
        std::vector<double> coords;
        if (val.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
          coords = val.get<std::vector<double>>();
        } else if (val.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
          for (auto v : val.get<std::vector<int64_t>>()) {
            coords.push_back(static_cast<double>(v));
          }
        }

        if (coords.size() == 3) {
          CheckPoint cp;
          cp.name = point_name;
          cp.point.x = coords[0];
          cp.point.y = coords[1];
          cp.point.z = coords[2];
          check_points_.push_back(cp);
          RCLCPP_INFO(this->get_logger(), "Loaded check point '%s': [%.2f, %.2f, %.2f]",
                      cp.name.c_str(), cp.point.x, cp.point.y, cp.point.z);
        } else {
          RCLCPP_WARN(this->get_logger(), "Check point '%s' must have exactly 3 coordinates [x, y, z], got %zu",
                      point_name.c_str(), coords.size());
        }
      }
    }

    // Also check declared parameters if not already loaded from overrides
    if (check_points_.empty()) {
      auto param_names = this->list_parameters({"check_points"}, 2);
      for (const auto & full_name : param_names.names) {
        if (full_name.rfind(prefix, 0) == 0) {
          std::string point_name = full_name.substr(prefix.length());
          std::vector<double> coords;
          if (this->get_parameter(full_name, coords) && coords.size() == 3) {
            CheckPoint cp;
            cp.name = point_name;
            cp.point.x = coords[0];
            cp.point.y = coords[1];
            cp.point.z = coords[2];
            check_points_.push_back(cp);
            RCLCPP_INFO(this->get_logger(), "Loaded check point '%s': [%.2f, %.2f, %.2f]",
                        cp.name.c_str(), cp.point.x, cp.point.y, cp.point.z);
          }
        }
      }
    }

    // Fallback: support flat array format if check_points was specified as a flat list of doubles
    if (check_points_.empty() && this->has_parameter("check_points")) {
      std::vector<double> flat_points;
      this->get_parameter("check_points", flat_points);
      if (flat_points.size() % 3 == 0) {
        for (size_t i = 0; i + 2 < flat_points.size(); i += 3) {
          CheckPoint cp;
          cp.name = "p" + std::to_string(i / 3 + 1);
          cp.point.x = flat_points[i];
          cp.point.y = flat_points[i + 1];
          cp.point.z = flat_points[i + 2];
          check_points_.push_back(cp);
          RCLCPP_INFO(this->get_logger(), "Loaded check point '%s': [%.2f, %.2f, %.2f]",
                      cp.name.c_str(), cp.point.x, cp.point.y, cp.point.z);
        }
      }
    }

    // Default fallback if no check points found
    if (check_points_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No check_points found in config, fail the test.");
      is_done_ = true;
      is_passed_ = false;
    }
  }
  
  void testCb() {
    switch (current_state_) {

    case State::WAIT_B2M_TF: {
      std::string tf_error;
      if(tf2Buffer_->canTransform("map", "base_link", tf2::TimePointZero, &tf_error)){
        RCLCPP_INFO(this->get_logger(), "Got Map to Baselink TF.");
        current_state_ = State::WAIT_BAG_DONE;
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Wait for Map to Baselink TF.");
      }
      break;
    }
    case State::WAIT_BAG_DONE: {
      
      std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - latest_pc_time_;
      if(elapsed_time.count() > 3.0){
        RCLCPP_INFO(this->get_logger(), "Bag finished");
        current_state_ = State::CHECK_RESULT;
      }

      break;
    }
    case State::CHECK_RESULT: {
      //@ check points in lethal, they should not exist
      loadCheckPoints();

      if (!kdtree_lethal_) {
        RCLCPP_ERROR(this->get_logger(), "kdtree_lethal_ is null! No lethal cloud received.");
        current_state_ = State::FAIL;
        break;
      }

      bool failed = false;
      for (const auto & cp : check_points_) {
        std::vector<int> id;
        std::vector<float> sqdist;
        if (kdtree_lethal_->radiusSearch(cp.point, search_radius_, id, sqdist) > 0) {
          RCLCPP_ERROR(this->get_logger(), "Lethal point detected near %s [%.2f, %.2f, %.2f] within radius %.2f",
                       cp.name.c_str(), cp.point.x, cp.point.y, cp.point.z, search_radius_);
          failed = true;
          break;
        }
        RCLCPP_INFO(this->get_logger(), "No lethal at %s [%.2f, %.2f, %.2f]",
                    cp.name.c_str(), cp.point.x, cp.point.y, cp.point.z);
      }

      if (failed) {
        current_state_ = State::FAIL;
      } else {
        current_state_ = State::SUCCEED;
      }
      break;
    }
    case State::SUCCEED: {
      RCLCPP_INFO(this->get_logger(), "P3D MPL laser scan test SUCCEEDED!");
      is_done_ = true;
      is_passed_ = true;
      break;
    }
    case State::FAIL: {
      RCLCPP_ERROR(this->get_logger(), "P3D MPL laser scan test FAILED!");
      is_done_ = true;
      is_passed_ = false;
      break;
    }
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<P3dMplLaserscan>(options);
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    if (node->is_done_) {
      if (node->is_passed_) {
        std::cout << "DoneSuccess" << std::endl;
        break;
      } else {
        std::cout << "DoneFailed" << std::endl;
        break;
      }
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
