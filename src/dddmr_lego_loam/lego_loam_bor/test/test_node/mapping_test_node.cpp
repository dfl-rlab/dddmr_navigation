#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "opt_icp_gn/common.h"
#include "opt_icp_gn/optimized_ICP_GN.h"

#include <rcpputils/asserts.hpp>

using namespace std::chrono_literals;

class MappingTestNode : public rclcpp::Node {
public:
  MappingTestNode() : Node("mapping_test_node") {
    trigger_bag_mapping_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("lego_loam_bag_pause", 2);
    sub_playing_state_ = this->create_subscription<std_msgs::msg::Bool>(
        "is_bag_playing", 1,
        std::bind(&MappingTestNode::playingStateCb, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        100ms, std::bind(&MappingTestNode::testCb, this));
    RCLCPP_INFO(this->get_logger(), "MappingTestNode: %s has been started.", this->get_name());
  }

  bool is_done_ = false;
  bool is_passed_ = false;

private:
  OptimizedICPGN icp_opti_;
  bool is_playing_ = false;
  enum class State {
    TRIGGER_MAPPING,
    WAIT_MAPPING_DONE,
    WAIT_FOR_FILE_COMPLETE,
    ANALYZE_MAPPING_RESULT,
    SUCCEED,
    FAIL
  };

  void testCb() {
    switch (current_state_) {
    case State::TRIGGER_MAPPING: {
      std_msgs::msg::Bool pause_mapping;
      pause_mapping.data = false;
      trigger_bag_mapping_pub_->publish(pause_mapping);
      if(!is_playing_){
        break;
      }
      else{
        std::filesystem::remove_all("/tmp/testing_pg");
        RCLCPP_INFO(this->get_logger(), "Changing state to WAIT_MAPPING_DONE");
        current_state_ = State::WAIT_MAPPING_DONE;
        break;
      }

    }
    case State::WAIT_MAPPING_DONE:
      // Implement dir scan mechanism
      if (std::filesystem::exists("/tmp/testing_pg") &&
          std::filesystem::is_directory("/tmp/testing_pg")) {
        RCLCPP_INFO(this->get_logger(),
                    "Changing state to WAIT_FOR_FILE_COMPLETE");
        current_state_ = State::WAIT_FOR_FILE_COMPLETE;
      }
      break;
    case State::WAIT_FOR_FILE_COMPLETE: {
      static int wait_ticks = 0;
      wait_ticks++;
      if (wait_ticks >= 30) { // 3 seconds at 10 Hz
        RCLCPP_INFO(this->get_logger(),
                    "Changing state to ANALYZE_MAPPING_RESULT");
        current_state_ = State::ANALYZE_MAPPING_RESULT;
        wait_ticks = 0;
      }
      break;
    }
    case State::ANALYZE_MAPPING_RESULT: {
      pcl::PointCloud<pcl::PointXYZI>::Ptr golden_map(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr golden_ground(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr golden_all(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr result_map(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr result_ground(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr result_all(
          new pcl::PointCloud<pcl::PointXYZI>);

      std::string dir_path = "/root/dddmr_bags/cicdtest/" + std::string(this->get_name()) + "/pg/";
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(dir_path + "map.pcd",
                                               *golden_map) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read file golden map.pcd");
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(dir_path + "ground.pcd",
                                               *golden_ground) == -1) {
        RCLCPP_ERROR(this->get_logger(),
                     "Couldn't read file golden ground.pcd");
      }

      *golden_all = *golden_map + *golden_ground;

      std::string result_dir_path = "/tmp/testing_pg/";
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(result_dir_path + "map.pcd",
                                               *result_map) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read file result map.pcd");
        std::cout << "Done\n";
        assert(false);
        timer_->cancel();
        rclcpp::shutdown();
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(result_dir_path + "ground.pcd",
                                               *result_ground) == -1) {
        RCLCPP_ERROR(this->get_logger(),
                     "Couldn't read file result ground.pcd");
        std::cout << "Done\n";
        assert(false);
        timer_->cancel();
        rclcpp::shutdown();
      }
      *result_all = *result_map + *result_ground;

      icp_opti_.SetTargetCloud(golden_all);
      icp_opti_.SetTransformationEpsilon(1e-2);
      icp_opti_.SetMaxIterations(50);
      icp_opti_.SetMaxCorrespondDistance(100.0);

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_opti_transformed_ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
      Eigen::Matrix4f T_predict = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f T_final;

      icp_opti_.Match(result_all, T_predict, cloud_source_opti_transformed_ptr,
                      T_final);
      RCLCPP_INFO(this->get_logger(), "ICP Score: %.2f",
                  icp_opti_.GetFitnessScore());

      if (icp_opti_.GetFitnessScore() < 0.1) {
        RCLCPP_INFO(this->get_logger(), "Changing state to SUCCEED");
        current_state_ = State::SUCCEED;
      } else {
        RCLCPP_INFO(this->get_logger(), "Changing state to FAIL");
        current_state_ = State::FAIL;
      }
      break;
    }
    case State::SUCCEED: {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Mapping test SUCCEEDED! ICP score: %.2f",
                           icp_opti_.GetFitnessScore());
      is_done_ = true;
      is_passed_ = true;
      break;
    }
    case State::FAIL: {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Mapping test FAILED! ICP score: %.2f",
                            icp_opti_.GetFitnessScore());
      is_done_ = true;
      is_passed_ = false;
      break;
    }
    }
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_bag_mapping_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_playing_state_;
  void playingStateCb(const std_msgs::msg::Bool::SharedPtr msg){
    is_playing_ = msg->data;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  State current_state_ = State::TRIGGER_MAPPING;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingTestNode>();
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
