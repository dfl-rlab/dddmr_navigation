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
#include <perception_3d/perception_3d_ros.h>

namespace perception_3d
{

Perception3D_ROS::Perception3D_ROS(std::string name):
  Node(name),
  name_(name),
  stacked_perception_(NULL),
  plugin_loader_("perception_3d", "perception_3d::Sensor")
{ 
  //constructer is not able to accept pass node ptr to other class, because itself is node and is not yet constructed. 
}


void Perception3D_ROS::initial(){

  //@ get global and robot base frame names
  declare_parameter("global_frame", rclcpp::ParameterValue(""));
  this->get_parameter("global_frame", global_frame_);
  RCLCPP_INFO(this->get_logger(), "global_frame: %s", global_frame_.c_str());//map

  declare_parameter("robot_base_frame", rclcpp::ParameterValue(""));
  this->get_parameter("robot_base_frame", robot_base_frame_);
  RCLCPP_INFO(this->get_logger(), "robot_base_frame: %s", robot_base_frame_.c_str());//base_link

  declare_parameter("max_obstacle_distance", rclcpp::ParameterValue(0.0));
  this->get_parameter("max_obstacle_distance", max_obstacle_distance_);
  RCLCPP_INFO(this->get_logger(), "max_obstacle_distance: %.1f", max_obstacle_distance_);//9999.0

  declare_parameter("inscribed_radius", rclcpp::ParameterValue(0.0));
  this->get_parameter("inscribed_radius", inscribed_radius_);
  RCLCPP_INFO(this->get_logger(), "inscribed_radius: %.2f", inscribed_radius_);//0.5

  declare_parameter("inflation_descending_rate", rclcpp::ParameterValue(0.0));
  this->get_parameter("inflation_descending_rate", inflation_descending_rate_);
  RCLCPP_INFO(this->get_logger(), "inflation_descending_rate: %.2f", inflation_descending_rate_);//2.0

  declare_parameter("inflation_radius", rclcpp::ParameterValue(0.0));
  this->get_parameter("inflation_radius", inflation_radius_);
  RCLCPP_INFO(this->get_logger(), "inflation_radius: %.2f", inflation_radius_);//1.5

  declare_parameter("sensors_collected_frequency", rclcpp::ParameterValue(0.0));
  this->get_parameter("sensors_collected_frequency", sensors_collected_frequency_);
  RCLCPP_INFO(this->get_logger(), "sensors_collected_frequency: %.2f", sensors_collected_frequency_);//5.0

  declare_parameter("dgraph_publish_frequency", rclcpp::ParameterValue(0.0));
  this->get_parameter("dgraph_publish_frequency", dgraph_publish_frequency_);
  RCLCPP_INFO(this->get_logger(), "dgraph_publish_frequency: %.2f", dgraph_publish_frequency_);//5.0

  
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

  
  //@
  //Create gbl utils
  //This is convenient to pass tf2_buffer, global_frame, robot_frame to plugins.
  //Anything want to pass down to plugins and be implemented by extending this class
  //
  gbl_utils_ = std::make_shared<GlobalUtils>(global_frame_, robot_base_frame_,
                                            max_obstacle_distance_, inscribed_radius_, 
                                            inflation_descending_rate_, inflation_radius_,
                                            tf2Buffer_);

  
  //@We need to make sure that the transform between the robot base frame and the global frame is available
  std::string tf_error;
  rclcpp::Rate r(2);
  while (rclcpp::ok() &&
    !tf2Buffer_->canTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }
  
  stacked_perception_ = new StackedPerception(this->get_node_logging_interface());

  //@Start to load plugins
  this->declare_parameter("plugins", rclcpp::PARAMETER_STRING_ARRAY);
  rclcpp::Parameter plugins = this->get_parameter("plugins");
  plugins_ = plugins.as_string_array();
  for(auto i=plugins_.begin(); i!=plugins_.end(); i++){

    //@ get plugin type
    std::string get_plugin_type_str = (*i)+".plugin"; //plugins.plugin, ex: map.plugin
    this->declare_parameter(get_plugin_type_str, rclcpp::ParameterValue(""));
    rclcpp::Parameter plugin_type_param = this->get_parameter(get_plugin_type_str);
    std::string plugin_type_str = plugin_type_param.as_string();

    RCLCPP_INFO(this->get_logger(), "Use plugin name: %s ---> %s", (*i).c_str(), plugin_type_str.c_str());  

    std::shared_ptr<Sensor> plugin = plugin_loader_.createSharedInstance(plugin_type_str);
    stacked_perception_->addPlugin(plugin);
    //Pass shared data for Sensor to share customized data betrween plugins
    plugin->initialize((*i), 
                      shared_from_this(),
                      gbl_utils_); //ex: send "map" to plugin for param reader
  }
  
  //@ if perception is under clearing, timer should stop update.
  //@ leverage same group to save mutex lock
  clear_perception_and_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto loop_time = std::chrono::milliseconds(int(1000/sensors_collected_frequency_));
  RCLCPP_WARN(this->get_logger(), "Loop time: %ld ms", loop_time.count());
  mark_and_clear_start_time_ = clock_->now();
  sensors_update_loop_timer_ = this->create_wall_timer(loop_time, std::bind(&Perception3D_ROS::sensorsUpdateLoop, this), clear_perception_and_timer_group_);
  
  //@ create dGraph publisher
  if(dgraph_publish_frequency_>0.0){
    auto publish_time = std::chrono::milliseconds(int(1000/dgraph_publish_frequency_));
    RCLCPP_WARN(this->get_logger(), "dGraph publish time: %ld ms", publish_time.count());
    pub_dGraph_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("perception_3d_ros/dGraph", 2);
    dgraph_publish_timer_ = this->create_wall_timer(publish_time, std::bind(&Perception3D_ROS::dGraphPublishLoop, this), clear_perception_and_timer_group_);
  }

  clear_perceptions_srv_ = 
      this->create_service<std_srvs::srv::Empty>
          ("~/clear_perception_marking", 
          std::bind(&Perception3D_ROS::clearPerceptionsSrv, this, 
          std::placeholders::_1, std::placeholders::_2), 
          rmw_qos_profile_services_default,
          clear_perception_and_timer_group_);
}


Perception3D_ROS::~Perception3D_ROS()
{
  delete stacked_perception_;
}

std::shared_ptr<perception_3d::SharedData> Perception3D_ROS::getSharedDataPtr(){
  return stacked_perception_->getSharedDataPtr();
}


void Perception3D_ROS::getGlobalPose(geometry_msgs::msg::TransformStamped& gbl_pose){

  try
  {
    gbl_pose = tf2Buffer_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to get global pose: %s", e.what());
    return;
  }

}

void Perception3D_ROS::getGlobalPose(geometry_msgs::msg::PoseStamped& gbl_pose){

  try
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero);
    gbl_pose.header.frame_id = global_frame_;
    gbl_pose.header.stamp = clock_->now();
    gbl_pose.pose.position.x = transformStamped.transform.translation.x;
    gbl_pose.pose.position.y = transformStamped.transform.translation.y;
    gbl_pose.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to get global pose: %s", e.what());
    return;
  }

}

void Perception3D_ROS::sensorsUpdateLoop()
{

  mark_and_clear_start_time_ = clock_->now();
  #ifdef HAVE_SYS_TIME_H
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  #endif


  stacked_perception_->doClear_then_Mark();


  #ifdef HAVE_SYS_TIME_H
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  RCLCPP_INFO(this->get_logger(), "Map update time: %.9f", t_diff);
  #endif

  // make sure to sleep for the remainder of our cycle time
  auto time_diff = (clock_->now() - mark_and_clear_start_time_).seconds();
  if(time_diff > 1./sensors_collected_frequency_+0.01){
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 1.0, "Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", sensors_collected_frequency_,
              time_diff);  
  }
  
}

void Perception3D_ROS::dGraphPublishLoop(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg2 (new pcl::PointCloud<pcl::PointXYZI>);
  for(size_t index=0;index<stacked_perception_->getSharedDataPtr()->static_ground_size_;index++){
    pcl::PointXYZI ipt;
    ipt.x = stacked_perception_->getSharedDataPtr()->pcl_ground_->points[index].x;
    ipt.y = stacked_perception_->getSharedDataPtr()->pcl_ground_->points[index].y;
    ipt.z = stacked_perception_->getSharedDataPtr()->pcl_ground_->points[index].z; 
    ipt.intensity = get_min_dGraphValue(index);
    pcl_msg2->push_back(ipt);
  }
  sensor_msgs::msg::PointCloud2 ros_pc2_msg2;
  pcl_msg2->header.frame_id = gbl_utils_->getGblFrame();
  pcl::toROSMsg(*pcl_msg2, ros_pc2_msg2);
  pub_dGraph_->publish(ros_pc2_msg2); 
}

double Perception3D_ROS::get_min_dGraphValue(const unsigned int index){
  return stacked_perception_->get_min_dGraphValue(index);
}

unsigned long Perception3D_ROS::getdGraphSize(){
  // return minimum size of dGraph for a start algorithm to rule out dgrpah value, this feature is for dynamic ground size
  return stacked_perception_->getdGraphSize();
}

void Perception3D_ROS::clearPerceptionsSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                        std::shared_ptr<std_srvs::srv::Empty::Response> resp)
{
  RCLCPP_INFO(this->get_logger(), "Reset graph in perception 3d ros.");
  stacked_perception_->resetdGraph();
  return;
}

}//end of name space
