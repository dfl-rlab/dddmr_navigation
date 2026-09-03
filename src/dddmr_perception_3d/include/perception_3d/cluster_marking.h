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
#ifndef PERCEPTION_3D_CLUSTER_MARKING_H_
#define PERCEPTION_3D_CLUSTER_MARKING_H_

#include <perception_3d/sensor.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
/*Point cloud library*/
#include <pcl/point_types.h>

/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

/*voxel*/
#include <pcl/filters/voxel_grid.h>

/*For normal/casting markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*For map*/
#include <map>
#include <set>

/*For sqrt*/
#include <math.h> 

/*kd tree for casting*/
#include <pcl/kdtree/kdtree_flann.h>

/*Open MP*/
#include <omp.h>

/*Project to ground*/
#include <pcl/filters/project_inliers.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

/*tf2 to ros msg/vice versa*/
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
/*For shortest angle*/
#include <angles/angles.h>

namespace perception_3d
{

struct marking_voxel{
  int x;
  int y;
  int z;
};

class per_marking{
  public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_;
    pcl::ModelCoefficients::Ptr mc_;
    std::unordered_map<int, float> nodes_of_min_distance_;
};

class Marking{

  /*
  This class is created to support marking in this plugin and made it sync with dynamic graph
  Originally, the marking_ is created but I need to sync it with dynamic graph,
  therefore, the class with do it to handle two std::map
  */
  typedef std::map<int, std::map<int, std::map<int, perception_3d::per_marking>>> marking_t;

  public:

    //@ Voxel structure
    marking_t marking_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr marking_pc_;

    Marking(std::string m_name, DynamicGraph* dg, double inscribed_radius, double inflation_radius, const std::shared_ptr<perception_3d::SharedData>& shared_data, double xy_resolution, double height_resolution):
      name_(m_name), dGraph_(dg), inscribed_radius_(inscribed_radius), inflation_radius_(inflation_radius), shared_data_(shared_data), xy_resolution_(xy_resolution), height_resolution_(height_resolution){
        marking_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      };
    
    ~Marking();

    void addPCPtr(const double x, const double y, const double z, 
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
      const pcl::ModelCoefficients::Ptr& pcplaneptr);

    void computeMinDistanceFromObstacle2GroundNodes(  
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
      const pcl::ModelCoefficients::Ptr& pcplaneptr,
      std::unordered_map<int, float>& nodes_of_min_distance);

    void removePCPtr(perception_3d::per_marking& per_marking);

    void updateCleared(const std::vector<marking_voxel>& current_observation_ptr);

    marking_t::iterator getXIter(const int x_bound){return marking_.lower_bound(x_bound);};
    marking_t::iterator getBegin(){return marking_.begin();};
    marking_t::iterator getEnd(){return marking_.end();};

    double get_dGraphValue(const unsigned int index){
      if (dGraph_->graph_.find(index) == dGraph_->graph_.end()){
        RCLCPP_INFO(rclcpp::get_logger(name_), "dGraph is queried without initialization, make sure your sensor topic is published and TF setup is correct.");
        return 0.0;
      }
      return dGraph_->getValue(index);
    };

    //@ groud pcl index, distance to lehal. Everything within this map will be used for line-of-sight check
    std::map<int, double> lethal_map_;
    
  private:
    
    std::string name_;

    //@ For dynamic_graph, because kdd find int index, we use int
    std::map<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::unordered_map<int, float>> marking2node_;  

    DynamicGraph* dGraph_;  
    
    double xy_resolution_, height_resolution_;
    double inflation_radius_;
    double inscribed_radius_;
    
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_;

    rclcpp::Time last_observation_time_;

    std::shared_ptr<perception_3d::SharedData> shared_data_;

};


}//end of name space
#endif