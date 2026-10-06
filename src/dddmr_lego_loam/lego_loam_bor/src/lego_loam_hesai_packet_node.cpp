#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include "lego_loam_map_visualization.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "rclcpp/rclcpp.hpp"

// ROS bag
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include "rclcpp/serialization.hpp"

//interactive
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interactive_pose_graph_editor.h"
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/srv/udp_frame_to_point_cloud.hpp>

using namespace std::chrono_literals;

class HesaiBagReader : public rclcpp::Node
{
  public:
    HesaiBagReader();
    std::string getBagFilePath(){return bag_file_dir_;}
    std::string getPacketTopic(){return hesai_packet_topic_;}
    std::string getOdometryTopic(){return odometry_topic_;}
    std::string getLidarServiceName(){return lidar_service_name_;}
    void writeLog(std::string input_str){RCLCPP_INFO(this->get_logger(), "%s", input_str.c_str());}
    sensor_msgs::msg::PointCloud2::SharedPtr convertPacketToPointCloud(const hesai_ros_driver::msg::UdpFrame& frame_msg);
    rclcpp::Time first_odom_stamp_, current_odom_stamp_;
    int skip_frame_;
    bool pause_mapping_;
    float icp_score_;
    float history_keyframe_search_radius_;
    bool save_current_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_point_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr play_state_pub_;
    rclcpp::Client<hesai_ros_driver::srv::UdpFrameToPointCloud>::SharedPtr udp_to_pc_client_;
    std::string bag_format_;

  private:

    std::string bag_file_dir_;
    std::string hesai_packet_topic_;
    std::string odometry_topic_;
    std::string lidar_service_name_;
    
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

HesaiBagReader::HesaiBagReader():Node("bag_reader"), pause_mapping_(true), save_current_map_(false){
  
  declare_parameter("bag_file_dir", rclcpp::ParameterValue(""));
  this->get_parameter("bag_file_dir", bag_file_dir_);
  RCLCPP_INFO(this->get_logger(), "bag_file_dir: %s", bag_file_dir_.c_str());

  declare_parameter("bag_format", rclcpp::ParameterValue("sqlite3"));
  this->get_parameter("bag_format", bag_format_);
  RCLCPP_INFO(this->get_logger(), "bag_format: %s", bag_format_.c_str());

  declare_parameter("hesai_packet_topic", rclcpp::ParameterValue(""));
  this->get_parameter("hesai_packet_topic", hesai_packet_topic_);
  if (hesai_packet_topic_.empty()) {
    declare_parameter("point_cloud_topic", rclcpp::ParameterValue(""));
    this->get_parameter("point_cloud_topic", hesai_packet_topic_);
  }
  RCLCPP_INFO(this->get_logger(), "hesai_packet_topic: %s", hesai_packet_topic_.c_str());

  declare_parameter("odometry_topic", rclcpp::ParameterValue(""));
  this->get_parameter("odometry_topic", odometry_topic_);
  RCLCPP_INFO(this->get_logger(), "odometry_topic: %s", odometry_topic_.c_str());

  declare_parameter("skip_frame", rclcpp::ParameterValue(2));
  this->get_parameter("skip_frame", skip_frame_);
  RCLCPP_INFO(this->get_logger(), "skip_frame: %d", skip_frame_);  

  declare_parameter("lidar_service_name", rclcpp::ParameterValue("/lidar_packet_to_point_cloud"));
  this->get_parameter("lidar_service_name", lidar_service_name_);
  RCLCPP_INFO(this->get_logger(), "lidar_service_name: %s", lidar_service_name_.c_str());

  raw_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_point_cloud", 1);  
  raw_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 1);  
  play_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_bag_playing", 1); 
  udp_to_pc_client_ = this->create_client<hesai_ros_driver::srv::UdpFrameToPointCloud>(lidar_service_name_);

  sub_pause_ = this->create_subscription<std_msgs::msg::Bool>(
        "lego_loam_bag_pause", 1,
        std::bind(&HesaiBagReader::bagPauseCb, this, std::placeholders::_1));
  sub_skip_frame_ = this->create_subscription<std_msgs::msg::Int32>(
        "lego_loam_bag_skip_frame", 1,
        std::bind(&HesaiBagReader::bagSkipFrameCb, this, std::placeholders::_1));
  sub_icp_score_ = this->create_subscription<std_msgs::msg::Float32>(
        "lego_loam_bag_icp_score", 1,
        std::bind(&HesaiBagReader::bagICPScoreCb, this, std::placeholders::_1));
  sub_history_keyframe_search_radius_ = this->create_subscription<std_msgs::msg::Float32>(
        "lego_loam_bag_history_keyframe_search_radius", 1,
        std::bind(&HesaiBagReader::bagHistoryKeyframeSearchRadiusCb, this, std::placeholders::_1));
  sub_save_current_map_ = this->create_subscription<std_msgs::msg::Bool>(
        "lego_loam_bag_save_current_map", 1,
        std::bind(&HesaiBagReader::saveCurrentMapCb, this, std::placeholders::_1));
}

sensor_msgs::msg::PointCloud2::SharedPtr HesaiBagReader::convertPacketToPointCloud(const hesai_ros_driver::msg::UdpFrame& frame_msg)
{
  if (!udp_to_pc_client_->service_is_ready()) {
    if (!udp_to_pc_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service '%s' is not ready!", udp_to_pc_client_->get_service_name());
      return nullptr;
    }
  }

  auto request = std::make_shared<hesai_ros_driver::srv::UdpFrameToPointCloud::Request>();
  request->frame = frame_msg;

  auto future = udp_to_pc_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    return std::make_shared<sensor_msgs::msg::PointCloud2>(response->point_cloud);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service '%s' or request timed out", udp_to_pc_client_->get_service_name());
    return nullptr;
  }
}

void HesaiBagReader::bagPauseCb(const std_msgs::msg::Bool::SharedPtr msg){
  pause_mapping_ = msg->data;
}
void HesaiBagReader::bagSkipFrameCb(const std_msgs::msg::Int32::SharedPtr msg){
  skip_frame_ = msg->data;
}
void HesaiBagReader::bagICPScoreCb(const std_msgs::msg::Float32::SharedPtr msg){
  icp_score_ = msg->data;
}
void HesaiBagReader::bagHistoryKeyframeSearchRadiusCb(const std_msgs::msg::Float32::SharedPtr msg){
  history_keyframe_search_radius_ = msg->data;
}
void HesaiBagReader::saveCurrentMapCb(const std_msgs::msg::Bool::SharedPtr msg){
  save_current_map_ = msg->data;
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  Channel<ProjectionOut> projection_out_channel(true);
  auto IP = std::make_shared<ImageProjection>("lego_loam_ip", projection_out_channel);
  Channel<AssociationOut> association_out_channel(false);
  auto FA = std::make_shared<FeatureAssociation>("lego_loam_fa", projection_out_channel, association_out_channel);
  auto MO = std::make_shared<MapOptimization>("lego_loam_mo", association_out_channel);
  auto BR = std::make_shared<HesaiBagReader>();
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

  auto match_topic = [](const std::string& a, const std::string& b) {
    if (a == b) return true;
    if (!a.empty() && a[0] == '/' && a.substr(1) == b) return true;
    if (!b.empty() && b[0] == '/' && b.substr(1) == a) return true;
    return false;
  };

  std::string packet_topic_type = "";
  for (const auto& tm : reader.get_all_topics_and_types()) {
    if (match_topic(tm.name, BR->getPacketTopic())) {
      packet_topic_type = tm.type;
      break;
    }
  }
  RCLCPP_INFO(BR->get_logger(), "Packet topic '%s' detected type in bag: '%s'",
              BR->getPacketTopic().c_str(), packet_topic_type.c_str());

  if (packet_topic_type != "sensor_msgs/msg/PointCloud2") {
    std::string srv_name = BR->getLidarServiceName();
    if (!srv_name.empty() && srv_name[0] != '/') {
      srv_name = "/" + srv_name;
    }
    std::vector<std::string> candidate_services = {
      srv_name,
      "/lidar_packet_to_point_cloud",
      "/hesai_ros_driver/lidar_packet_to_point_cloud"
    };

    RCLCPP_INFO(BR->get_logger(), "Waiting for lidar_packet_to_point_cloud service to become available...");
    bool service_found = false;
    while (rclcpp::ok() && !service_found) {
      for (const auto& cand : candidate_services) {
        if (cand != BR->udp_to_pc_client_->get_service_name()) {
          BR->udp_to_pc_client_ = BR->create_client<hesai_ros_driver::srv::UdpFrameToPointCloud>(cand);
        }
        if (BR->udp_to_pc_client_->wait_for_service(std::chrono::milliseconds(250))) {
          service_found = true;
          RCLCPP_INFO(BR->get_logger(), "Service '%s' is ready!", cand.c_str());
          break;
        }
      }
      if (!service_found) {
        RCLCPP_INFO_THROTTLE(BR->get_logger(), *BR->get_clock(), 2000,
                             "Waiting for lidar_packet_to_point_cloud service (tried %s, /hesai_ros_driver/lidar_packet_to_point_cloud)...",
                             srv_name.c_str());
      }
    }
  }

  //@calculate clock
  double last_wall_time = 0.0;
  double last_global_map_pub_time = 0.0;
  double last_frame_number_print_time = 0.0;
  bool go_first_odom = false;
  struct timeval start, inloop, end;
  gettimeofday(&start, NULL);
  int cycle_cnt = 0;
  last_global_map_pub_time = start.tv_sec + double(start.tv_usec) / 1e6;
  builtin_interfaces::msg::Time zero_time_stamp;
  zero_time_stamp.sec = 0;
  zero_time_stamp.nanosec = 0;

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

    if (match_topic(topic, BR->getOdometryTopic()))
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

    if (match_topic(topic, BR->getPacketTopic()))
    {
      cycle_cnt++;
      sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg;

      if (packet_topic_type == "hesai_ros_driver/msg/UdpFrame" ||
          packet_topic_type.find("UdpFrame") != std::string::npos ||
          (packet_topic_type != "sensor_msgs/msg/PointCloud2" && !packet_topic_type.empty()))
      {
        hesai_ros_driver::msg::UdpFrame udp_frame;
        auto serializer = rclcpp::Serialization<hesai_ros_driver::msg::UdpFrame>();
        serializer.deserialize_message(&extracted_serialized_msg, &udp_frame);
        laserCloudMsg = BR->convertPacketToPointCloud(udp_frame);
      }
      else
      {
        sensor_msgs::msg::PointCloud2 msg;
        auto serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
        serializer.deserialize_message(&extracted_serialized_msg, &msg);
        laserCloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);
      }

      if (!laserCloudMsg) {
        continue;
      }

      sensor_msgs::msg::PointCloud2 pub_msg = *laserCloudMsg;
      pub_msg.header.stamp = zero_time_stamp;
      BR->raw_point_cloud_pub_->publish(pub_msg);
      IP->cloudHandler(laserCloudMsg);
      if(!IP->pc_valid_){
        continue;
      }
      
      bool FA_ready = FA->systemInitedLM; //This line should come before FA->runFeatureAssociation()
      FA->runFeatureAssociation();
      if(FA->odom_type_ == "wheel_odometry" && !FA->first_odom_prepared_){
        continue;
      }
      
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
        
        auto shared_6d_pose = std::make_shared<sensor_msgs::msg::PointCloud2>(MO->cloud_msg_pose_6d_);
        LLV->cloudKeyPoses6D_callback(shared_6d_pose);

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
