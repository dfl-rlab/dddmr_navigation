#include "rclcpp/rclcpp.hpp"
#include <rcpputils/asserts.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class MCL3DLTestNode : public rclcpp::Node {

public:

  MCL3DLTestNode() : Node("mcl_3dl_test_node") {
    
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

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 2,
        std::bind(&MCL3DLTestNode::cbOdom, this, std::placeholders::_1));

    initial_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initial_3d_pose", 2);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&MCL3DLTestNode::testCb, this));

    RCLCPP_INFO(this->get_logger(), "MCL3DLTestNode: %s has been started.", this->get_name());



  }

  bool is_done_ = false;
  bool is_passed_ = false;

private:

  enum class State {
    WAIT_B2M_TF,
    WAIT_N_SECONDS,
    SEND_INITIAL_POSE,
    WAIT_LOCALIZATION_DONE,
    CHECK_RESULT,
    SUCCEED,
    FAIL
  };
  
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::TimerBase::SharedPtr timer_;
  State current_state_ = State::WAIT_B2M_TF;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
  
  rclcpp::Time latest_odom_time_;
  int dummy_cnt_ = 0;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg){

    latest_odom_time_ = clock_->now();
  }

  void testCb() {
    switch (current_state_) {

    case State::WAIT_B2M_TF: {
      std::string tf_error;
      if(tf2Buffer_->canTransform("map", "base_link", tf2::TimePointZero, &tf_error)){
        RCLCPP_INFO(this->get_logger(), "Got Map to Baselink TF.");
        current_state_ = State::WAIT_N_SECONDS;
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Wait for Map to Baselink TF.");
      }
      break;
    }
    case State::WAIT_N_SECONDS: {
      dummy_cnt_++;
      if(dummy_cnt_>30){
        current_state_ = State::SEND_INITIAL_POSE;
      }
      break;
    }
    case State::SEND_INITIAL_POSE: {
      geometry_msgs::msg::PoseWithCovarianceStamped p;
      p.header.frame_id = "map";
      p.pose.pose.position.x = 1.1513099670410156;
      p.pose.pose.position.y = -2.90902042388916;
      p.pose.pose.position.z = -0.3209571838378906;
      p.pose.pose.orientation.x = 0.0;
      p.pose.pose.orientation.y = 0.0;
      p.pose.pose.orientation.z = 0.9986889872489132;
      p.pose.pose.orientation.w = 0.05118893188708095;
      RCLCPP_INFO(this->get_logger(), "Sending initial pose at %.2f ,%.2f ,%2.f", p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z);
      initial_pose_pub_->publish(p);
      current_state_ = State::WAIT_LOCALIZATION_DONE;
      latest_odom_time_ = clock_->now();
      break;
    }
    case State::WAIT_LOCALIZATION_DONE: {
      
      try
      {
        transform_stamped_ = tf2Buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      }
      catch (tf2::TransformException& e)
      {
        RCLCPP_INFO(this->get_logger(), "Failed to get transform: %s", e.what());
      }

      if(dummy_cnt_% 10 == 0 ){
        RCLCPP_INFO(this->get_logger(), "Location: %.2f, %.2f, %.2f", 
        transform_stamped_.transform.translation.x, 
        transform_stamped_.transform.translation.y, 
        transform_stamped_.transform.translation.z);
      }


      if((clock_->now() - latest_odom_time_).seconds()>3.0){
        RCLCPP_INFO(this->get_logger(), "Bag finished");
        current_state_ = State::CHECK_RESULT;
      }

      dummy_cnt_++;
      break;
    }
    case State::CHECK_RESULT: {
      /*
      At time 0.0
      - Translation: [50.449, -92.092, -1.738]
      - Rotation: in Quaternion (xyzw) [-0.003, 0.027, -0.377, 0.926]
      */
      double dx = transform_stamped_.transform.translation.x - 50.449;
      double dy = transform_stamped_.transform.translation.y + 92.092;
      double dz = transform_stamped_.transform.translation.z + 1.738;
      double distance_diff = sqrt(dx*dx + dy*dy + dz*dz);
      RCLCPP_INFO(this->get_logger(), "Distance deviation: %.3f m", distance_diff);
      if(distance_diff<0.3){
        current_state_ = State::SUCCEED;
      }
      else{
        current_state_ = State::FAIL;
      }
      break;
    }
    case State::SUCCEED: {
      RCLCPP_INFO(this->get_logger(), "MCL 3DL test SUCCEEDED!");
      is_done_ = true;
      is_passed_ = true;
      break;
    }
    case State::FAIL: {
      RCLCPP_ERROR(this->get_logger(), "MCL 3DL test FAILED!");
      is_done_ = true;
      is_passed_ = false;
      break;
    }
    }
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MCL3DLTestNode>();
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
