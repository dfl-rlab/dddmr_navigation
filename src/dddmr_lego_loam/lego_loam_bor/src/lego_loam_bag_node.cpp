#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include "lego_loam_map_visualization.hpp"
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
    int skip_frame_;
    bool pause_mapping_;
    float icp_score_;
    float history_keyframe_search_radius_;
    bool save_current_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_point_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr play_state_pub_;
    std::string bag_format_;

  private:

    std::string bag_file_dir_;
    std::string point_cloud_topic_;
    std::string odometry_topic_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_pause_;
    void bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_skip_frame_;
    void bagSkipFrameCb(const std_msgs::msg::Int32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_icp_score_;
    void bagICPScoreCb(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_history_keyframe_search_radius_;
    void bagHistoryKeyframeSearchRadiusCb(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_save_current_map_;
    void saveCurrentMapCb(const std_msgs::msg::Bool::SharedPtr msg);
};

BagReader::BagReader():Node("bag_reader"), pause_mapping_(true), save_current_map_(false){
  
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

  declare_parameter("skip_frame", rclcpp::ParameterValue(2));
  this->get_parameter("skip_frame", skip_frame_);
  RCLCPP_INFO(this->get_logger(), "skip_frame: %d", skip_frame_);  

  raw_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 1);  
  raw_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 1);  
  play_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_bag_playing", 1); 

  sub_pause_ = this->create_subscription<std_msgs::msg::Bool>(
        "lego_loam_bag_pause", 1,
        std::bind(&BagReader::bagPauseCb, this, std::placeholders::_1));
  sub_skip_frame_ = this->create_subscription<std_msgs::msg::Int32>(
        "lego_loam_bag_skip_frame", 1,
        std::bind(&BagReader::bagSkipFrameCb, this, std::placeholders::_1));
  sub_icp_score_ = this->create_subscription<std_msgs::msg::Float32>(
        "lego_loam_bag_icp_score", 1,
        std::bind(&BagReader::bagICPScoreCb, this, std::placeholders::_1));
  sub_history_keyframe_search_radius_ = this->create_subscription<std_msgs::msg::Float32>(
        "lego_loam_bag_history_keyframe_search_radius", 1,
        std::bind(&BagReader::bagHistoryKeyframeSearchRadiusCb, this, std::placeholders::_1));
  sub_save_current_map_ = this->create_subscription<std_msgs::msg::Bool>(
        "lego_loam_bag_save_current_map", 1,
        std::bind(&BagReader::saveCurrentMapCb, this, std::placeholders::_1));
}

void BagReader::bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg){
  pause_mapping_ = msg->data;
}
void BagReader::bagSkipFrameCb(const std_msgs::msg::Int32::SharedPtr msg){
  skip_frame_ = msg->data;
}
void BagReader::bagICPScoreCb(const std_msgs::msg::Float32::SharedPtr msg){
  icp_score_ = msg->data;
}
void BagReader::bagHistoryKeyframeSearchRadiusCb(const std_msgs::msg::Float32::SharedPtr msg){
  history_keyframe_search_radius_ = msg->data;
}
void BagReader::saveCurrentMapCb(const std_msgs::msg::Bool::SharedPtr msg){
  save_current_map_ = msg->data;
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  auto MO = std::make_shared<MapOptimization>("lego_loam_mo", association_out_channel);
  auto BR = std::make_shared<BagReader>();
  auto IPGE = std::make_shared<InteractivePoseGraphEditor>("interactive_pose_graph_editor", MO);
  auto LLV = std::make_shared<LegoLoamVisualization>("lego_loam_map_visualization");
  
  IP->tfInitial();
  FA->tfInitial();
  
  BR->icp_score_ = MO->_history_keyframe_fitness_score;
  BR->history_keyframe_search_radius_ = MO->_history_keyframe_search_radius;

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
  bool go_first_odom = false;
  struct timeval start, inloop, end;
  gettimeofday(&start, NULL);
  int cycle_cnt = 0;
  last_global_map_pub_time = start.tv_sec + double(start.tv_usec) / 1e6;
  while (rclcpp::ok() && reader.has_next())
  {
    std_msgs::msg::Bool playing;
    if(BR->pause_mapping_){
      rclcpp::spin_some(BR);
      rclcpp::spin_some(IPGE);
      rclcpp::spin_some(LLV);
      if(BR->save_current_map_){
        std::shared_ptr<std_srvs::srv::Empty::Request> request;
        std::shared_ptr<std_srvs::srv::Empty::Response> response;
        MO->pcdSaver(request, response);
        BR->save_current_map_ = false;
      }
      playing.data = false;
      BR->play_state_pub_->publish(playing);
      continue;
    }

    playing.data = true;
    BR->play_state_pub_->publish(playing);    
    //assign interactive values
    MO->_history_keyframe_fitness_score = BR->icp_score_;
    MO->_history_keyframe_search_radius = BR->history_keyframe_search_radius_;

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
      BR->raw_odom_pub_->publish(msg);
      FA->odomHandler(odom);
      if(!go_first_odom){
        BR->first_odom_stamp_ = msg.header.stamp;
        BR->current_odom_stamp_ = msg.header.stamp;
        go_first_odom = true;
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
      sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg;
      laserCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      *laserCloudMsg = msg;
      msg.header.stamp = zero_time_stamp;
      BR->raw_point_cloud_pub_->publish(msg);
      IP->cloudHandler(laserCloudMsg);
      if(!IP->pc_valid_){
        continue;
      }
      
      bool FA_ready = FA->systemInitedLM; //This line should come before FA->runFeatureAssociation()
      FA->runFeatureAssociation();
      nav_msgs::msg::Odometry::SharedPtr mapping_odom;
      mapping_odom = std::make_shared<nav_msgs::msg::Odometry>();
      *mapping_odom = FA->mappingOdometry;

      std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Request> request_key_frame(new dddmr_sys_core::srv::GetKeyFrameCloud::Request());
      request_key_frame->key_frame_number = LLV->corner_key_frame_clouds_.size();
      std::shared_ptr<dddmr_sys_core::srv::GetKeyFrameCloud::Response> response_key_frame(new dddmr_sys_core::srv::GetKeyFrameCloud::Response());
      MO->getKeyFrameCloud(request_key_frame, response_key_frame);
      LLV->processKeyFrameCloudResult(response_key_frame);
      if(FA_ready && cycle_cnt%BR->skip_frame_==0){

        MO->run();
        LLV->trans_m2ci_af3_ = MO->trans_m2ci_af3_;
        LLV->has_m2ci_ = true;
        *(LLV->cloudKeyPoses3D) = *(MO->cloudKeyPoses3D);
        *(LLV->cloudKeyPoses6D) = *(MO->cloudKeyPoses6D);

        gettimeofday(&inloop, NULL);
        double inloop_t = inloop.tv_sec + double(inloop.tv_usec) / 1e6;
        if(inloop_t - last_global_map_pub_time > 1.0){
          LLV->pubMapThread();
          last_global_map_pub_time = inloop_t;
        }
        
        MO->loopClosureThread();
        //MO->groundEdgeDetectionThread();
      }
      else if(FA_ready){
        MO->runWoLO();
      }
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
    if(BR->save_current_map_){
      std::shared_ptr<std_srvs::srv::Empty::Request> request;
      std::shared_ptr<std_srvs::srv::Empty::Response> response;
      MO->pcdSaver(request, response);
      BR->save_current_map_ = false;
    }
    
    rclcpp::spin_some(BR); 
  }
  std::shared_ptr<std_srvs::srv::Empty::Request> request;
  std::shared_ptr<std_srvs::srv::Empty::Response> response;

  MO->pcdSaver(request, response);
  rclcpp::shutdown();

  return 0;
}


