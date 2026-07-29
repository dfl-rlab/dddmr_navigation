#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <bits/stdc++.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class OptICPGNPlaygroundKey2Target : public rclcpp::Node
{
  public:

    OptICPGNPlaygroundKey2Target();

  private:

    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_frame_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_frame_pre_integrated_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_frame_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icped_frame_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr key_frame_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr key_frame_pre_integrated_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_frame_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr icped_frame_;

    Eigen::Affine3f key_2_second_af3_;
    Eigen::Affine3d key_2_second_af3d_;


};

OptICPGNPlaygroundKey2Target::OptICPGNPlaygroundKey2Target():Node("optimized_icp_gn_playground"){
  
  clock_ = this->get_clock();

  key_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("key_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  key_frame_pre_integrated_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("key_frame_pre_integrated", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  target_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("target_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  icped_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icped_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());


  std::string package_share_directory = ament_index_cpp::get_package_share_directory("lego_loam_bor");
  RCLCPP_DEBUG(this->get_logger(), "%s", package_share_directory.c_str());

  key_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (package_share_directory + "/pcd/feature.pcd", *key_frame_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read PCD file fail: %s", package_share_directory.c_str());
  }

  target_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (package_share_directory + "/pcd/map.pcd", *target_frame_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read PCD file fail: %s", package_share_directory.c_str());
  }

  sensor_msgs::msg::PointCloud2 feature_cloudMsgTemp;
  pcl::toROSMsg(*key_frame_, feature_cloudMsgTemp);
  feature_cloudMsgTemp.header.stamp = clock_->now();
  feature_cloudMsgTemp.header.frame_id = "map";
  key_frame_pub_->publish(feature_cloudMsgTemp);

  sensor_msgs::msg::PointCloud2 target_cloudMsgTemp;
  pcl::toROSMsg(*target_frame_, target_cloudMsgTemp);
  target_cloudMsgTemp.header.stamp = clock_->now();
  target_cloudMsgTemp.header.frame_id = "map";
  target_frame_pub_->publish(target_cloudMsgTemp);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_opti_transformed_ptr;
  cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  key_frame_pre_integrated_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  Eigen::Matrix4f T_pre_intergration, T_final;

  T_pre_intergration << 1.0, -0.0, 0.0, -15.0,
                0.0, -1.0, 0.0, -0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

  
  pcl::transformPointCloud(*key_frame_, *key_frame_pre_integrated_, T_pre_intergration);

  sensor_msgs::msg::PointCloud2 pre_cloudMsgTemp;
  pcl::toROSMsg(*key_frame_pre_integrated_, pre_cloudMsgTemp);
  pre_cloudMsgTemp.header.stamp = clock_->now();
  pre_cloudMsgTemp.header.frame_id = "map";
  key_frame_pre_integrated_pub_->publish(pre_cloudMsgTemp);
  
  rclcpp::Time start = clock_->now();

  OptimizedICPGN icp_opti;
  icp_opti.SetTargetCloud(target_frame_);
  icp_opti.SetTransformationEpsilon(1e-4);
  icp_opti.SetMaxIterations(100);
  icp_opti.SetMaxCorrespondDistance(1.0);
  icp_opti.Match(key_frame_, T_pre_intergration, cloud_source_opti_transformed_ptr, T_final);
  float icp_score = icp_opti.GetFitnessScore();
  double t_taken = (clock_->now()-start).seconds();
  RCLCPP_INFO(this->get_logger(), "Time taken: %.8f, ICP score: %.2f", t_taken, icp_score);
  
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f relative_rt;
  relative_rt = T_final;
  pcl::getTranslationAndEulerAngles(relative_rt, x, y, z, roll, pitch, yaw);
  RCLCPP_INFO(this->get_logger(), "RT ---> XYZ: %.5f, %.5f, %.5f, RPY: %.5f, %.5f, %.5f", x, y, z, roll, pitch, yaw);

  key_2_second_af3_ = T_final;//icp.getFinalTransformation();
  key_2_second_af3d_ = key_2_second_af3_.cast<double>();
  icped_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*key_frame_, *icped_frame_, key_2_second_af3d_);
  
  sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
  pcl::toROSMsg(*icped_frame_, icpedcloud2MsgTemp);
  icpedcloud2MsgTemp.header.stamp = clock_->now();
  icpedcloud2MsgTemp.header.frame_id = "map";
  icped_frame_pub_->publish(icpedcloud2MsgTemp);
}


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto OICPGNP = std::make_shared<OptICPGNPlaygroundKey2Target>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(OICPGNP);
  executor.spin();

  return 0;
}


