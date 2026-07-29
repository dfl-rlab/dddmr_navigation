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
#include <global_planner/a_star_on_pc.h>

AstarList::AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_ground_->setInputCloud(pc_original_z_up_);
}

void AstarList::Initial(){
  as_list_.clear(); 
  for(unsigned int it=0; it!=pc_original_z_up_->points.size();it++){
    Node_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false};
    as_list_[it] = new_node;
  }
  f_priority_set_.clear();
}

Node_t AstarList::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

Node_t AstarList::getNode_wi_MinimumF(){
  auto first_it = f_priority_set_.begin();
  Node_t m_node = as_list_[(*first_it).second];
  if(!m_node.is_closed){
    f_priority_set_.erase(first_it);
    return m_node;
  }
  
  //Because we updateNode node even when new g value is smaller than that in openlist
  //We will have duplicate f value in the f_priority_set_
  int concern_cnt = 0;
  while(m_node.is_closed && !f_priority_set_.empty()){
    concern_cnt++;
    f_priority_set_.erase(first_it);
    first_it = f_priority_set_.begin();
    m_node = as_list_[(*first_it).second];
  }
  return m_node;
}

bool AstarList::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  return f_priority_set_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_Graph::A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                                  double a_star_expanding_radius){
  
  perception_ros_ = perception_ros;
  pc_original_z_up_ = pc_original_z_up;
  a_star_expanding_radius_ = a_star_expanding_radius;
  ASLS_ = new AstarList(pc_original_z_up_);
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up){
  ASLS_->pc_original_z_up_ = pc_original_z_up;
  ASLS_->kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  ASLS_->kdtree_ground_->setInputCloud(pc_original_z_up_);
}

double A_Star_on_Graph::getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1, s1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  s1 = sqrt(vx1*vx1 + vy1*vy1);
  //@ calculate vector: current -> expanding
  float vx2, vy2, s2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  s2 = sqrt(vx2*vx2 + vy2*vy2);

  float pitch = fabs(m_pcl_current_parent.z - m_pcl_expanding.z)/(s1+s2);

  return pitch;
}

double A_Star_on_Graph::getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  //@ calculate vector: current -> expanding
  float vx2, vy2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  float cos_theta = (vx1*vx2 + vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2));
  if(fabs(cos_theta)>1)
    cos_theta = 1.0;
  double theta_of_vector = acos(cos_theta);
  if(vx1==0 && vy1==0)
    theta_of_vector = 0;
  else if(vx2==0 && vy2==0)
    theta_of_vector = 0;
  else if(fabs(fabs(vx1)-fabs(vx2))<=0.0001)
    theta_of_vector = 0;
  
  if(fabs(theta_of_vector)<=0.345)//cap
    theta_of_vector = 0.0;

  return theta_of_vector;
}

bool A_Star_on_Graph::isLineOfSightClear(pcl::PointXYZI& pcl_current, pcl::PointXYZI& pcl_expanding, double inscribed_radius){

  //@ generate line equation
  float dX =
      pcl_expanding.x - pcl_current.x;
  float dY =
      pcl_expanding.y - pcl_current.y;
  float dZ =
      pcl_expanding.z - pcl_current.z;
  
  float distance = sqrt(dX*dX + dY*dY + dZ*dZ);
  distance = distance/inscribed_radius; //sample by every inscribed radius
  float dt = 1/distance;
  for(float t=0; t<=1.0+dt; t+=dt){
    float r = t;
    if(t>=1.0) //@ make sure we examine t=1.0
      r = 1.0;
    pcl::PointXYZI a_pt;
    a_pt.intensity = 0.0;
    a_pt.x = pcl_current.x + dX*r;
    a_pt.y = pcl_current.y + dY*r;
    a_pt.z = pcl_current.z + dZ*r;
    std::vector<int> pidx;
    std::vector<float> prsd;
    kdtree_lethal_->radiusSearch(a_pt, 2*inscribed_radius, pidx, prsd);
    if(pidx.size()>1){
      return false;
    }
  }
  return true;
}

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path){

  //RCLCPP_INFO(rclcpp::get_logger("astar"),"Start: %u, Goal: %u", start, goal);

  /*
  Create the first node which is start and add into frontier
  */

  pcl::PointXYZI pcl_goal = pc_original_z_up_->points[goal];
  pcl::PointXYZI pcl_start = pc_original_z_up_->points[start];
  float f = sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal));
  Node_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

  ASLS_->Initial();
  ASLS_->updateNode(current_node);
  
  double inscribed_radius = perception_ros_->getGlobalUtils()->getInscribedRadius();
  double inflation_descending_rate = perception_ros_->getGlobalUtils()->getInflationDescendingRate();
  double max_obstacle_distance = perception_ros_->getGlobalUtils()->getMaxObstacleDistance();
  
  perception_ros_->getStackedPerception()->aggregateLethal();
  //@ generate kd-tree and handle no point cloud edge case
  kdtree_lethal_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  
  if(perception_ros_->getSharedDataPtr()->aggregate_lethal_->points.size()>0){
    kdtree_lethal_->setInputCloud(perception_ros_->getSharedDataPtr()->aggregate_lethal_);
  }
    

  while(!ASLS_->isFrontierEmpty()){ 
    /*Pop minimum F, we leverage prior queue, so we dont need to loop frontier everytime*/
    current_node = ASLS_->getNode_wi_MinimumF();

    //RCLCPP_INFO(rclcpp::get_logger("astar"), "Expand node: %u", current_node.self_index);
    /*Get successors*/
    pcl::PointXYZI pcl_now = pc_original_z_up_->points[current_node.self_index];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    ASLS_->kdtree_ground_->radiusSearch(pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    //@dealing with orphan node
    if(pointIdxRadiusSearch.size()<8){
      std::vector<int> pointIdxRadiusX2Search;
      std::vector<float> pointRadiusSquaredDistanceX2;
      //ASLS_->kdtree_ground_->nearestKSearch(pcl_now, 8, pointIdxRadiusX2Search, pointRadiusSquaredDistanceX2);
      ASLS_->kdtree_ground_->radiusSearch(pcl_now, 2*a_star_expanding_radius_, pointIdxRadiusX2Search, pointRadiusSquaredDistanceX2);
      pointIdxRadiusSearch = pointIdxRadiusX2Search;
    }

    //@ calculated average intensity, because we have sparse low cost orphan, and it is unlikely to have a low cost node surrounded by high cost nodes
    float avg_intensity = 0.0;
    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      avg_intensity += pc_original_z_up_->points[pointIdxRadiusSearch[it]].intensity;
    }
    avg_intensity = avg_intensity/pointIdxRadiusSearch.size();

    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      
      int current_expanding_index = pointIdxRadiusSearch[it];
      float current_expanding_g = sqrt(pointRadiusSquaredDistance[it]);

      //@ dGraphValue is the distance to lethal
      double dGraphValue = perception_ros_->get_min_dGraphValue(current_expanding_index);

      /*This is for lethal*/
      if(dGraphValue<inscribed_radius){
        //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "%.2f,%.2f,%.2f, v: %.2f",pc_original_z_up_->points[(*it).first].x,pc_original_z_up_->points[(*it).first].y,pc_original_z_up_->points[(*it).first].z, dGraphValue);
        continue;
      }

      pcl::PointXYZI pcl_current = pc_original_z_up_->points[current_node.self_index];
      pcl::PointXYZI pcl_current_parent = pc_original_z_up_->points[current_node.parent_index];
      pcl::PointXYZI pcl_expanding = pc_original_z_up_->points[current_expanding_index];

      //@ check line-of-sight when distance is 2 times larger than inscribed_radius
      if(current_expanding_g>=2*inscribed_radius){
        if(!isLineOfSightClear(pcl_current, pcl_expanding, inscribed_radius))
          continue;
      }
      
      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      //@ get current_parent, current, expanding to compute theta od expanding
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);
      
      //if(getPitchFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding)>0.2)
      //  continue;
      
      float ground_edge_weight = avg_intensity;
      float node_weight = perception_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeight(current_expanding_index);
      float new_g = current_node.g + current_expanding_g + factor * 1.0 + node_weight + theta*turning_weight_ + ground_edge_weight;
      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal));
      float new_f = new_g + new_h;

      Node_t new_node = {.self_index=(current_expanding_index), .g=new_g, .h=new_h, .f=new_f, .parent_index=current_node.self_index, .is_closed=false, .is_opened=true};

      /*Check is in closed list*/
      if(ASLS_->isClosed(current_expanding_index))
        continue;
      /*Check is in opened list*/
      else if(ASLS_->isOpened(current_expanding_index)){
        if(ASLS_->getGVal(new_node)>new_g){
          ASLS_->updateNode(new_node);          
        }
      }
      /*addNode*/
      else{
        ASLS_->updateNode(new_node);
      }
        
      
    }

    /*Close this node*/
    ASLS_->closeNode(current_node);

    /*If goal is in closed list, we are done*/
    if(ASLS_->isClosed(goal)){
      Node_t trace_back = ASLS_->getNode(goal);
      while(trace_back.self_index!=trace_back.parent_index){
        path.push_back(trace_back.self_index);
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      path.push_back(trace_back.self_index);//Push start point
      std::reverse(path.begin(),path.end()); 
      break;
    }

    /*Check if*/
  }

}
