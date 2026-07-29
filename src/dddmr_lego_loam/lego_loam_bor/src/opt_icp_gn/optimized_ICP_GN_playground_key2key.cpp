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

class OptICPGNPlayground : public rclcpp::Node
{
  public:

    OptICPGNPlayground();

  private:

    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr key_frame_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr second_frame_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr second_frame_answer_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icped_frame_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr key_frame_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr second_frame_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr second_answer_frame_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr icped_frame_;

    Eigen::Affine3f key_2_second_af3_;
    Eigen::Affine3d key_2_second_af3d_;


};

OptICPGNPlayground::OptICPGNPlayground():Node("optimized_icp_gn_playground"){
  
  clock_ = this->get_clock();

  key_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("key_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  second_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("second_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  second_frame_answer_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("second_answer_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  icped_frame_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icped_frame", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());


  std::string package_share_directory = ament_index_cpp::get_package_share_directory("lego_loam_bor");
  RCLCPP_DEBUG(this->get_logger(), "%s", package_share_directory.c_str());

  key_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (package_share_directory + "/pcd/feature.pcd", *key_frame_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read PCD file fail: %s", package_share_directory.c_str());
  }

  sensor_msgs::msg::PointCloud2 feature_cloudMsgTemp;
  pcl::toROSMsg(*key_frame_, feature_cloudMsgTemp);
  feature_cloudMsgTemp.header.stamp = clock_->now();
  feature_cloudMsgTemp.header.frame_id = "map";
  key_frame_pub_->publish(feature_cloudMsgTemp);
  
  // translate and rotate key frame to second frame
  second_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  transform_1.translation() << 30.0, 30.0, 0.0;
  pcl::transformPointCloud (*key_frame_, *second_frame_, transform_1);

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.rotate (Eigen::AngleAxisf (3.1415926/4.0, Eigen::Vector3f::UnitZ()));  
  pcl::transformPointCloud (*second_frame_, *second_frame_, transform_2);

  Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
  transform_3.translation() << -20.0, -20.0, 0.0;
  pcl::transformPointCloud (*second_frame_, *second_frame_, transform_3);

  Eigen::Affine3f transform_4 = Eigen::Affine3f::Identity();
  transform_4.rotate (Eigen::AngleAxisf (3.1415926/4.0, Eigen::Vector3f::UnitZ()));  
  pcl::transformPointCloud (*second_frame_, *second_frame_, transform_4);

  Eigen::Affine3f af3_answer = transform_4*transform_3*transform_2*transform_1;
  
  second_answer_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*key_frame_, *second_answer_frame_, af3_answer);

  sensor_msgs::msg::PointCloud2 answercloud2MsgTemp;
  pcl::toROSMsg(*second_answer_frame_, answercloud2MsgTemp);
  answercloud2MsgTemp.header.stamp = clock_->now();
  answercloud2MsgTemp.header.frame_id = "map";
  second_frame_answer_pub_->publish(answercloud2MsgTemp);

  sensor_msgs::msg::PointCloud2 feature2_cloudMsgTemp;
  pcl::toROSMsg(*second_frame_, feature2_cloudMsgTemp);
  feature2_cloudMsgTemp.header.stamp = clock_->now();
  feature2_cloudMsgTemp.header.frame_id = "map";
  second_frame_pub_->publish(feature2_cloudMsgTemp);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_opti_transformed_ptr;
  cloud_source_opti_transformed_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  Eigen::Matrix4f T_predict, T_final;
  T_predict.setIdentity();
  
  /*
  af3_answer.matrix()
  */
  RCLCPP_INFO_STREAM(this->get_logger(), "Relative: \n" << af3_answer.matrix());  
  RCLCPP_INFO_STREAM(this->get_logger(), "Relative: \n" << af3_answer.matrix().inverse());  
  /*
  T_predict << 0.707107, 0.707107, 0.0, -30.0,
                -0.707107, 0.707107, 0.0, -30.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
  */
  
  T_predict = af3_answer.matrix().inverse();
  
  // RCLCPP_INFO_STREAM(this->get_logger(), "Relative: \n" << T_predict2);  
  rclcpp::Time start = clock_->now();

  OptimizedICPGN icp_opti;
  icp_opti.SetTargetCloud(key_frame_);
  icp_opti.SetTransformationEpsilon(1e-4);
  icp_opti.SetMaxIterations(100);
  icp_opti.SetMaxCorrespondDistance(10);
  icp_opti.Match(second_frame_, T_predict, cloud_source_opti_transformed_ptr, T_final);
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
  pcl::transformPointCloud(*second_frame_, *icped_frame_, key_2_second_af3d_);

  sensor_msgs::msg::PointCloud2 icpedcloud2MsgTemp;
  pcl::toROSMsg(*icped_frame_, icpedcloud2MsgTemp);
  icpedcloud2MsgTemp.header.stamp = clock_->now();
  icpedcloud2MsgTemp.header.frame_id = "map";
  icped_frame_pub_->publish(icpedcloud2MsgTemp);
}


int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto OICPGNP = std::make_shared<OptICPGNPlayground>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(OICPGNP);
  executor.spin();

  return 0;
}


