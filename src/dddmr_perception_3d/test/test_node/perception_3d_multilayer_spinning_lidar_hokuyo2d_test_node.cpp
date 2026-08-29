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

class P3dMplLaserscan : public rclcpp::Node {

public:

  P3dMplLaserscan() : Node("P3dMplLaserscan_test_node") {
    
    clock_ = this->get_clock();

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
      //@ check 3 point in lethal, they should not exist
      pcl::PointXYZ p1;
      p1.x = -1.78; p1.y = -1.48; p1.z = 0.0;
      std::vector<int> id;
      std::vector<float> sqdist;
      if(kdtree_lethal_->radiusSearch(p1, 0.2, id, sqdist)>0){
        current_state_ = State::FAIL;
        break;
      }   
      RCLCPP_INFO(this->get_logger(), "No lethal at %.2f, %.2f, %.2f", p1.x, p1.y, p1.z);  
      pcl::PointXYZ p2;
      p2.x = -1.67; p2.y = -2.16; p2.z = 0.0;
      if(kdtree_lethal_->radiusSearch(p2, 0.2, id, sqdist)>0){
        current_state_ = State::FAIL;
        break;
      } 
      RCLCPP_INFO(this->get_logger(), "No lethal at %.2f, %.2f, %.2f", p2.x, p2.y, p2.z);  
      pcl::PointXYZ p3;
      p3.x = -1.77; p3.y = -3.19; p3.z = 0.0;
      if(kdtree_lethal_->radiusSearch(p3, 0.2, id, sqdist)>0){
        current_state_ = State::FAIL;
        break;
      }     
      RCLCPP_INFO(this->get_logger(), "No lethal at %.2f, %.2f, %.2f", p3.x, p3.y, p3.z);  
      current_state_ = State::SUCCEED;
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
  auto node = std::make_shared<P3dMplLaserscan>();
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
