/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "odom_3d.h"

using std::placeholders::_1;

void GenericDriveOdom::init(){
  
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group_;
  clock_ = this->get_clock();

  declare_parameter("output_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("output_frequency", output_frequency_);
  RCLCPP_INFO(this->get_logger(), "output_frequency: %.2f", output_frequency_);

  declare_parameter("odom_topic", "/odom_2d");
  auto odom_topic = this->get_parameter("odom_topic").as_string();

  declare_parameter("imu_topic", "/imu/data");
  auto imu_topic = this->get_parameter("imu_topic").as_string();

  //quaternion initialized
  latest_imu_.orientation.x = 0.0;
  latest_imu_.orientation.y = 0.0;
  latest_imu_.orientation.z = 0.0;
  latest_imu_.orientation.w = 0.0;

  //euler angles initialized
  imu_roll_ = 0.0;
  imu_pitch_ = 0.0;
  imu_yaw_ =  0.0;

  //based on the pub_timer
  dt_ = 1.0/output_frequency_; 
  
  odom_3d_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable());

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&GenericDriveOdom::cbOdom2D, this, std::placeholders::_1), options);  

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
  imu_topic, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(),
      std::bind(&GenericDriveOdom::cbImu, this, std::placeholders::_1), options);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  auto loop_time = std::chrono::milliseconds(int(1000/output_frequency_));
  pub_timer_ = this->create_wall_timer(loop_time, std::bind(&GenericDriveOdom::pubOdom3D, this), cb_group_);
  
}

void GenericDriveOdom::pubOdom3D(){

  if(latest_imu_.orientation.x == 0.0 &&
    latest_imu_.orientation.y == 0.0 &&
    latest_imu_.orientation.z == 0.0 &&
    latest_imu_.orientation.w == 0.0){
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1000, "Looks like the IMU wasn't connected at the beginning");
    return;
  }

  current_time_ = this->get_clock()->now();
  tf2::Quaternion tmp_quaternion(latest_imu_.orientation.x, latest_imu_.orientation.y, latest_imu_.orientation.z, latest_imu_.orientation.w);
  tf2::Matrix3x3 tmp_matrix_trans(tmp_quaternion);
  tmp_matrix_trans.getRPY(imu_roll_, imu_pitch_, imu_yaw_);

  odom_3d_.header.stamp = current_time_;
  odom_3d_.header.frame_id = "odom";
  odom_3d_.child_frame_id = "base_link";
  odom_3d_.pose.pose.orientation.x = latest_imu_.orientation.x;
  odom_3d_.pose.pose.orientation.y = latest_imu_.orientation.y;
  odom_3d_.pose.pose.orientation.z = latest_imu_.orientation.z;
  odom_3d_.pose.pose.orientation.w = latest_imu_.orientation.w;

  odom_3d_.pose.pose.position.x += (((latest_odom_2d_.twist.twist.linear.x * cos(imu_pitch_)) * cos(imu_yaw_)) * dt_ + 
                                    ((latest_odom_2d_.twist.twist.linear.y * cos(imu_roll_)) * cos(1.570796327 + imu_yaw_)) * dt_);

  odom_3d_.pose.pose.position.y += (((latest_odom_2d_.twist.twist.linear.x * cos(imu_pitch_)) * sin(imu_yaw_)) * dt_ + 
                                    ((latest_odom_2d_.twist.twist.linear.y * cos(imu_roll_)) * sin(1.570796327 + imu_yaw_)) * dt_);

  odom_3d_.pose.pose.position.z += ((latest_odom_2d_.twist.twist.linear.x * sin(-imu_pitch_)) * dt_ + 
                                    (latest_odom_2d_.twist.twist.linear.y * sin(imu_roll_)) * dt_);

  odom_3d_.twist.twist.linear.x = latest_odom_2d_.twist.twist.linear.x;
  odom_3d_.twist.twist.angular.z = latest_odom_2d_.twist.twist.angular.z;
  odom_3d_pub_->publish(odom_3d_);

  // publish tf transformation
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time_;
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = odom_3d_.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_3d_.pose.pose.position.y;
  tf_msg.transform.translation.z = odom_3d_.pose.pose.position.z;
  tf_msg.transform.rotation = odom_3d_.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf_msg);
}

void GenericDriveOdom::cbImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_imu_ = *msg;
}

void GenericDriveOdom::cbOdom2D(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_2d_ = *msg;
  if(fabs(latest_odom_2d_.twist.twist.linear.x)<0.1)
    latest_odom_2d_.twist.twist.linear.x = 0.0;
  if(fabs(latest_odom_2d_.twist.twist.linear.y)<0.1)
    latest_odom_2d_.twist.twist.linear.y = 0.0; 
}

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  auto DDO = std::make_shared<GenericDriveOdom>();
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(); 
  executor->add_node(DDO); 
  executor->spin();
  rclcpp::shutdown();  
  return 0;
}
