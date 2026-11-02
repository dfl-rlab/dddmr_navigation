#include "imageProjectionTool.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

// ROS bag
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include "rclcpp/serialization.hpp"

//interactive
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "interactive_pose_graph_editor.h"

using namespace std::chrono_literals;

class BagReader : public rclcpp::Node
{
  public:
    BagReader();
    std::string getBagFilePath(){return bag_file_dir_;}
    std::string getPointCloudTopic(){return point_cloud_topic_;}
    std::string getOdometryTopic(){return odometry_topic_;}
    void writeLog(std::string input_str){RCLCPP_INFO(this->get_logger(), "%s", input_str.c_str());}
    rclcpp::Time first_odom_stamp_, current_odom_stamp_;
    bool pause_mapping_;
    std::string operation_command_;
    bool next_frame_{false};
    bool export_current_selection_{false};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_point_cloud_pub_;
    std::string bag_format_;
    pcl::PointCloud<PointType>::Ptr selected_cloud_;

  private:

    std::string bag_file_dir_;
    std::string point_cloud_topic_;
    std::string odometry_topic_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
    void bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_operation_command_;
    void operationCommandCb(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_selected_points_;
    void selectedPointsCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

BagReader::BagReader():Node("bag_reader"), pause_mapping_(true){
  
  declare_parameter("bag_file_dir", rclcpp::ParameterValue(""));
  this->get_parameter("bag_file_dir", bag_file_dir_);
  RCLCPP_INFO(this->get_logger(), "bag_file_dir: %s", bag_file_dir_.c_str());

  declare_parameter("bag_format", rclcpp::ParameterValue("sqlite3"));
  this->get_parameter("bag_format", bag_format_);
  RCLCPP_INFO(this->get_logger(), "bag_format: %s", bag_format_.c_str());

  declare_parameter("point_cloud_topic", rclcpp::ParameterValue(""));
  this->get_parameter("point_cloud_topic", point_cloud_topic_);
  RCLCPP_INFO(this->get_logger(), "point_cloud_topic: %s", point_cloud_topic_.c_str());

  declare_parameter("odometry_topic", rclcpp::ParameterValue(""));
  this->get_parameter("odometry_topic", odometry_topic_);
  RCLCPP_INFO(this->get_logger(), "odometry_topic: %s", odometry_topic_.c_str());
  raw_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_point_cloud", 1);  
  sub_pause_ = this->create_subscription<std_msgs::msg::Bool>(
        "lego_loam_bag_pause", 1,
        std::bind(&BagReader::bagPauseCb, this, std::placeholders::_1));
  sub_operation_command_ = this->create_subscription<std_msgs::msg::String>(
        "operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&BagReader::operationCommandCb, this, std::placeholders::_1));
  sub_selected_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud_selection/selected_points", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&BagReader::selectedPointsCb, this, std::placeholders::_1));
  selected_cloud_.reset(new pcl::PointCloud<PointType>());
}

void BagReader::bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg){
  pause_mapping_ = msg->data;
}

void BagReader::operationCommandCb(const std_msgs::msg::String::SharedPtr msg){
  operation_command_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received operation command: %s", msg->data.c_str());
  if(msg->data == "next_frame"){
    next_frame_ = true;
  }
  else if(msg->data == "export_current_selection"){
    export_current_selection_ = true;
  }
}

void BagReader::selectedPointsCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  selected_cloud_->clear();
  pcl::fromROSMsg(*msg, *selected_cloud_);
  RCLCPP_INFO(this->get_logger(), "Received selected points: %lu", selected_cloud_->points.size());
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjectionTool>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  //auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  //auto MO = std::make_shared<MapOptimization>("lego_loam_mo", association_out_channel);
  auto BR = std::make_shared<BagReader>();
  //auto IPGE = std::make_shared<InteractivePoseGraphEditor>("interactive_pose_graph_editor", MO);
  
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = BR->getBagFilePath();
  storage_options.storage_id = BR->bag_format_;

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  //@calculate clock
  double last_wall_time = 0.0;
  double last_global_map_pub_time = 0.0;
  double last_frame_number_print_time = 0.0;
  bool got_first_odom = false;
  struct timeval start, inloop, end;
  gettimeofday(&start, NULL);
  int cycle_cnt = 0;
  last_global_map_pub_time = start.tv_sec + double(start.tv_usec) / 1e6;
  
  //@ create depth image folder
  std::string mapping_dir_string;
  auto env_p = std::getenv("DDDMR_MAPPING_DIR");
  if ( env_p == NULL ) {
    mapping_dir_string = std::string("/tmp/") + currentDateTime();
    std::filesystem::create_directory(mapping_dir_string);
    BR->writeLog("Create dir: " + mapping_dir_string);
  } else {
    mapping_dir_string = std::string( env_p ) + currentDateTime();
    std::filesystem::create_directory(mapping_dir_string);
    BR->writeLog("Create dir: " + mapping_dir_string);
  }
  IP->mapping_dir_string_ = mapping_dir_string;
  *IP->selected_cloud_ = *BR->selected_cloud_;

  while (rclcpp::ok() && reader.has_next())
  {
    
    if(BR->pause_mapping_){
      rclcpp::spin_some(BR);
      *IP->selected_cloud_ = *BR->selected_cloud_;
      IP->projectImageBySelection();
      if(BR->export_current_selection_){
        IP->exportProjectedImages();
        BR->export_current_selection_ = false;
      }
      //rclcpp::spin_some(IPGE);
      if(BR->next_frame_){
        BR->pause_mapping_ = false;
      }
      continue;
    }

    // serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    auto topic = serialized_message->topic_name;

    if (topic == BR->getOdometryTopic())
    {
      nav_msgs::msg::Odometry msg;
      auto serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
      serializer.deserialize_message(&extracted_serialized_msg, &msg);
      nav_msgs::msg::Odometry::SharedPtr odom;
      odom = std::make_shared<nav_msgs::msg::Odometry>();
      *odom = msg;
      //FA->odomHandler(odom);
      if(!got_first_odom){
        BR->first_odom_stamp_ = msg.header.stamp;
        BR->current_odom_stamp_ = msg.header.stamp;
        got_first_odom = true;
      }
      else{
        BR->current_odom_stamp_ = msg.header.stamp;
      }
    }

    if (topic == BR->getPointCloudTopic())
    {
      cycle_cnt++;
      sensor_msgs::msg::PointCloud2 msg;
      auto zero_time_stamp = msg.header.stamp;
      auto serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
      serializer.deserialize_message(&extracted_serialized_msg, &msg);
      msg.header.frame_id = "lidar";
      sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg;
      laserCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      *laserCloudMsg = msg;
      msg.header.stamp = zero_time_stamp;
      BR->raw_point_cloud_pub_->publish(msg);
      IP->cloudHandler(laserCloudMsg);
    }
    //@ calculate bag time and wall ime
    double bag_time_diff = (BR->current_odom_stamp_ - BR->first_odom_stamp_).seconds();
    gettimeofday(&end, NULL);
    double start_t, end_t;
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    double wall_time_diff = end_t - start_t;
    double processing_speed = bag_time_diff/wall_time_diff;
    if(wall_time_diff - last_wall_time > 5.0){
      BR->writeLog("Processing Speed: " + std::to_string(processing_speed));
      last_wall_time = wall_time_diff;
    }
    if(wall_time_diff - last_frame_number_print_time > 1.0){
      BR->writeLog("Processing Frame Number: " + std::to_string(cycle_cnt));
      last_frame_number_print_time = wall_time_diff;
    }
    rclcpp::spin_some(BR);  
    if(BR->next_frame_){
      BR->pause_mapping_ = true;
      BR->next_frame_ = false;
    }
  }
  rclcpp::shutdown();

  return 0;
}


