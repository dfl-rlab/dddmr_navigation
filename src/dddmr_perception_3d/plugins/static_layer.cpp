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
#include <perception_3d/static_layer.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::StaticLayer, perception_3d::Sensor)

namespace perception_3d
{

StaticLayer::StaticLayer(){
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

StaticLayer::~StaticLayer(){

}

void StaticLayer::onInitialize()
{ 
  
  ptrInitial();

  cbs_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;

  node_->declare_parameter(name_ + ".radius_of_ground_connection", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".radius_of_ground_connection", radius_of_ground_connection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "radius_of_ground_connection: %.2f", radius_of_ground_connection_);

  node_->declare_parameter(name_ + ".use_adaptive_connection", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".use_adaptive_connection", use_adaptive_connection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "use_adaptive_connection: %d", use_adaptive_connection_);  
  
  node_->declare_parameter(name_ + ".adaptive_connection_number", rclcpp::ParameterValue(20));
  node_->get_parameter(name_ + ".adaptive_connection_number", adaptive_connection_number_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "adaptive_connection_number: %d", adaptive_connection_number_);  
  
  node_->declare_parameter(name_ + ".turning_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".turning_weight", turning_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "turning_weight: %.2f", turning_weight_);    

  node_->declare_parameter(name_ + ".intensity_search_radius", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".intensity_search_radius", intensity_search_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "intensity_search_radius: %.2f", intensity_search_radius_);    

  node_->declare_parameter(name_ + ".intensity_search_punish_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".intensity_search_punish_weight", intensity_search_punish_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "intensity_search_punish_weight: %.2f", intensity_search_punish_weight_);    

  node_->declare_parameter(name_ + ".static_imposing_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".static_imposing_radius", static_imposing_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "static_imposing_radius: %.2f", static_imposing_radius_);    

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);      

  node_->declare_parameter(name_ + ".map_topic", rclcpp::ParameterValue("mapcloud"));
  node_->get_parameter(name_ + ".map_topic", map_topic_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "map_topic: %s", map_topic_.c_str());     

  node_->declare_parameter(name_ + ".ground_topic", rclcpp::ParameterValue("mapground"));
  node_->get_parameter(name_ + ".ground_topic", ground_topic_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "ground_topic: %s", ground_topic_.c_str());     

  node_->declare_parameter(name_ + ".support.mapping_mode", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".support.mapping_mode", mapping_mode_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "mapping_mode: %d", mapping_mode_);     
  
  node_->declare_parameter(name_ + ".support.enable_edge_detection", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".support.enable_edge_detection", enable_edge_detection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "enable_edge_detection: %d", enable_edge_detection_);     

  node_->declare_parameter(name_ + ".support.generate_static_graph", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".support.generate_static_graph", generate_static_graph_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "generate_static_graph: %d", generate_static_graph_);     

  
  shared_data_->mapping_mode_ = mapping_mode_;
  
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(name_ + "/dGraph", 2);
  
  if(mapping_mode_){
    pcl_map_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), 
      std::bind(&StaticLayer::cbMap, this, std::placeholders::_1), sub_options);

    pcl_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), 
      std::bind(&StaticLayer::cbGround, this, std::placeholders::_1), sub_options);
  }
  else{
    pcl_map_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&StaticLayer::cbMap, this, std::placeholders::_1), sub_options);

    pcl_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&StaticLayer::cbGround, this, std::placeholders::_1), sub_options);
  }



}

void StaticLayer::ptrInitial(){
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->sGraph_ptr_ = std::make_shared<perception_3d::StaticGraph>();
  new_map_ = new_ground_ = is_local_planner_ = false;
  is_ground_and_map_being_initialized_once_ = false;
}

void StaticLayer::cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  /*transform to point cloud library format first so we can leverage PCL*/
  pcl::fromROSMsg(*msg, *pcl_map_);
  // remove NaN
  std::vector<int> indices;
  pcl_map_->is_dense = false;
  pcl::removeNaNFromPointCloud(*pcl_map_, *pcl_map_, indices);

  if(shared_data_->static_map_size_!=pcl_map_->points.size()){
    new_map_ = true;
    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s receive new \033[1;32mMap\033[0m with size: %lu", name_.c_str(), pcl_map_->points.size());
  }
  shared_data_->static_map_size_ = pcl_map_->points.size();
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_map_->setInputCloud(pcl_map_);
  shared_data_->pcl_map_ = pcl_map_;
}

void StaticLayer::cbGround(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  pcl::fromROSMsg(*msg, *pcl_ground_);  
  // remove NaN
  std::vector<int> indices;
  pcl_ground_->is_dense = false;
  pcl::removeNaNFromPointCloud(*pcl_ground_, *pcl_ground_, indices);

  if(shared_data_->static_ground_size_!=pcl_ground_->points.size()){
    new_ground_ = true;
    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s receive new \033[1;32mGround\033[0m with size: %lu", name_.c_str(), pcl_ground_->points.size());
  }  
  shared_data_->static_ground_size_ = pcl_ground_->points.size();
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_ground_->setInputCloud(pcl_ground_);
  shared_data_->pcl_ground_ = pcl_ground_;
 
}

void StaticLayer::updateLethalPointCloud(){
}

void StaticLayer::selfMark(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if(pub_dGraph_->get_subscription_count()>0){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg2 (new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t index=0;index<shared_data_->static_ground_size_;index++){
      pcl::PointXYZI ipt;
      ipt.x = shared_data_->pcl_ground_->points[index].x;
      ipt.y = shared_data_->pcl_ground_->points[index].y;
      ipt.z = shared_data_->pcl_ground_->points[index].z;   
      ipt.intensity = get_dGraphValue(index);
      pcl_msg2->push_back(ipt);
    }
    sensor_msgs::msg::PointCloud2 ros_pc2_msg2;
    pcl_msg2->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pcl_msg2, ros_pc2_msg2);
    pub_dGraph_->publish(ros_pc2_msg2);
  }
}

void StaticLayer::selfClear(){

  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if(new_ground_ && new_map_){

    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s has already received two msg.", name_.c_str());
    shared_data_->requestAllLayersToResetDGraph();

    shared_data_->dgraph_update_request_[name_] = false;
    new_ground_ = false;
    new_map_ = false;

    //@ radius search connection to generate sGraph
    resetdGraph();
    if(!is_local_planner_){
      //@ it is static layer of a global planner
      shared_data_->sGraph_ptr_->allocateGraph(pcl_ground_->points.size());
      if(!mapping_mode_ && enable_edge_detection_){
        radiusSearchConnection();
        RCLCPP_INFO(node_->get_logger().get_child(name_), "Computation of Edge from ground point cloud is done.");
      }
      if(generate_static_graph_){
        generateStaticGraph();
        RCLCPP_INFO(node_->get_logger().get_child(name_), "Static graph is generated with graph size: %lu", shared_data_->sGraph_ptr_->getSize());
      }
    }

    shared_data_->is_static_layer_ready_ = true;
    is_ground_and_map_being_initialized_once_ = true;
  }
  else{ //@ disable ready flag when state is different
    if(!is_ground_and_map_being_initialized_once_)
      shared_data_->is_static_layer_ready_ = false;
  }

}

void StaticLayer::generateStaticGraph(){
  
  #pragma omp parallel for
  for(unsigned int index_cnt = 0; index_cnt<pcl_ground_->points.size(); index_cnt++){

    pcl::PointXYZI pcl_node = pcl_ground_->points[index_cnt];

    //@Kd-tree to find nn point for planar equation
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!use_adaptive_connection_){
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, radius_of_ground_connection_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }
    else{

      int hard_interrupt_cnt = 100;
      float search_r = 0.5;
      int search_cnt = 1;
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      while(pointIdxRadiusSearch.size()<adaptive_connection_number_ && hard_interrupt_cnt>0){
        search_cnt++;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);    
        hard_interrupt_cnt--;   
      }
    }
    
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      //chekc relative z value for the edge, because we need to eliminate stair and wheel chair passage issue
      edge_t a_edge;
      auto node = index_cnt;
      a_edge.first = (*it);
      //@Create an edge
      double distance_between_pair = sqrt(pcl::geometry::squaredDistance(pcl_ground_->points[node], pcl_ground_->points[a_edge.first]));
      a_edge.second = distance_between_pair;
      shared_data_->sGraph_ptr_->insertEdgeInNode(node, a_edge);

    }

    //@ use map to impose weight on each node
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    if(shared_data_->kdtree_map_->nearestKSearch(pcl_node, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0){
      double distance_to_obstacle = sqrt(pointRadiusSquaredDistance[0]);
      if(distance_to_obstacle < gbl_utils_->getInscribedRadius())
        dGraph_.setValue(index_cnt, distance_to_obstacle);
    }
  }
}

void StaticLayer::radiusSearchConnection(){
  
  // 1. Determine total loop size
  const unsigned int total_points = pcl_ground_->points.size();

  // 2. OpenMP Parallel For Loop
  // We specify that 'index_cnt' is the loop variable (implicitly private)
  #pragma omp parallel for
  for(unsigned int index_cnt = 0; index_cnt < total_points; index_cnt++){
    
    pcl::PointXYZI pcl_node = pcl_ground_->points[index_cnt];

    //@Kd-tree to find nn point for planar equation
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!use_adaptive_connection_){
      // kdtree radiusSearch is thread-safe for concurrent reads
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, radius_of_ground_connection_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }
    else{
      int hard_interrupt_cnt = 100;
      float search_r = 0.5;
      int search_cnt = 1;
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      while(pointIdxRadiusSearch.size()<adaptive_connection_number_ && hard_interrupt_cnt>0){
        search_cnt++;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);    
        hard_interrupt_cnt--;   
      }
    }
    
    // Allocating objects inside the loop ensures they are thread-local (private)
    pcl::PointCloud<pcl::PointXYZI>::Ptr nn_pc (new pcl::PointCloud<pcl::PointXYZI>);
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      //@Push back the points for plane equation later
      nn_pc->push_back(pcl_ground_->points[(*it)]);
    }

    float weight = 1.0;
    float intensity_penality = 0.0;
    float max_radius = intensity_search_radius_; 
    int reject_threshold = 0;
    //@ consider this scenario to be boundary of ground
    if(nn_pc->points.size()<5){
      weight = 1000;
    }
    else{
      //@Use RANSAC to get normal
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      
      // Creating the segmentation object locally per thread
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.05); 

      seg.setInputCloud (nn_pc);
      seg.segment (*inliers, *coefficients);
      
      for(float ring_radius=max_radius; ring_radius>0; ring_radius-=0.25){
        for(float d_theta=-3.1415926; d_theta<=3.1415926; d_theta+=0.174){ //per 10 deg
          pcl::PointXYZI pcl_ring;
          pcl_ring.x = pcl_node.x + ring_radius*sin(d_theta);
          pcl_ring.y = pcl_node.y + ring_radius*cos(d_theta);
          pcl_ring.z = (-coefficients->values[3]-coefficients->values[0]*pcl_ring.x-coefficients->values[1]*pcl_ring.y)/coefficients->values[2];
          if(isinf(pcl_ring.z)){
            pcl_ring.z = 0.0;
          }
          if(std::isnan(pcl_ring.z))
            continue;
          
          std::vector<int> pointIdxRadiusSearch_ring;
          std::vector<float> pointRadiusSquaredDistance_ring;
          if(shared_data_->kdtree_ground_->radiusSearch (pcl_ring, 0.3, pointIdxRadiusSearch_ring, pointRadiusSquaredDistance_ring)<1) 
            reject_threshold++;
        }
      }
      intensity_penality += reject_threshold*intensity_search_punish_weight_;
      
      //@ use map to impose weight on each node
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_map_->radiusSearch (pcl_node, static_imposing_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_z_axes(new pcl::PointCloud<pcl::PointXYZI>);
      for(auto i_pcl_z_axes=pointIdxRadiusSearch.begin();i_pcl_z_axes!=pointIdxRadiusSearch.end();i_pcl_z_axes++){
        pcl_z_axes->push_back(pcl_map_->points[*i_pcl_z_axes]);
      }

      // Local Filter instantiation
      pcl::PassThrough<pcl::PointXYZI> pass;
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (pcl_node.z+0.1, pcl_node.z+1.0);
      pass.filter (*pcl_z_axes);
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (pcl_node.x-0.5, pcl_node.x+0.5);
      pass.filter (*pcl_z_axes);
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (pcl_node.y-0.5, pcl_node.y+0.5);
      pass.filter (*pcl_z_axes);
      
      if(pcl_z_axes->points.size()>10){
        dGraph_.setValue(index_cnt, 0.25);
      }  
    }
    shared_data_->sGraph_ptr_->setPenality(index_cnt, intensity_penality);
  }
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Static graph has been generated.");
}

void StaticLayer::resetdGraph(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
}

double StaticLayer::get_dGraphValue(const unsigned int index){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if (dGraph_.graph_.find(index) == dGraph_.graph_.end())
    return 0.0;
  return dGraph_.getValue(index);
}

bool StaticLayer::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StaticLayer::getObservation(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr StaticLayer::getLethal(){
  return current_lethal_;
}
}//end of name space
