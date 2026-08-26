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
#ifndef PERCEPTION_3D_CLUSTER_KDTREE_MARKING_H_
#define PERCEPTION_3D_CLUSTER_KDTREE_MARKING_H_

#include <perception_3d/sensor.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstdint>
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
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp> // Include if using pcl::search::KdTree

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

/* Customized point cloud type with fourth element as std::uint64_t */
struct PointXYZU64
{
  PCL_ADD_POINT4D;
  std::uint32_t hbyte;
  std::uint32_t lbyte;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZU64,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (std::uint32_t, hbyte, hbyte)
                                   (std::uint32_t, lbyte, lbyte)
)

namespace perception_3d
{

using PointXYZU64 = ::PointXYZU64;

class per_marking{
  public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_;
    pcl::ModelCoefficients::Ptr mc_;
    std::unordered_map<int, float> nodes_of_min_distance_;

    per_marking() = default;

    per_marking(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc,
                const pcl::ModelCoefficients::Ptr& mc,
                const std::unordered_map<int, float>& nodes_of_min_distance):
      pc_(pc), mc_(mc), nodes_of_min_distance_(nodes_of_min_distance) {}
};

class KDTreeMarking{

  public:
    
    std::unordered_map<std::uint64_t, perception_3d::per_marking> marking_map_;
    pcl::PointCloud<PointXYZU64>::Ptr marking_pc_;
    pcl::KdTreeFLANN<PointXYZU64>::Ptr kdtree_marking_;
    
    KDTreeMarking(std::string m_name, DynamicGraph* dg, double inscribed_radius, double inflation_radius, const std::shared_ptr<perception_3d::SharedData>& shared_data, double xy_resolution, double height_resolution):
      name_(m_name), dGraph_(dg), inscribed_radius_(inscribed_radius), inflation_radius_(inflation_radius), shared_data_(shared_data), xy_resolution_(xy_resolution), height_resolution_(height_resolution), marking_id_(0){
        marking_pc_.reset(new pcl::PointCloud<PointXYZU64>);
        kdtree_marking_.reset(new pcl::KdTreeFLANN<PointXYZU64>);
      };
    
    ~KDTreeMarking();

    void addPCPtr(PointXYZU64 centroid, 
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
      const pcl::ModelCoefficients::Ptr& pcplaneptr);

    void computeMinDistanceFromObstacle2GroundNodes(  
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
      const pcl::ModelCoefficients::Ptr& pcplaneptr,
      std::unordered_map<int, float>& nodes_of_min_distance);

    void removePCPtr(const PointXYZU64& centroid);

    void updateKDTree();

    void splitMarkingId(std::uint32_t& hbyte, std::uint32_t& lbyte);
    void splitUint64(std::uint64_t val, std::uint32_t& hbyte, std::uint32_t& lbyte);
    std::uint64_t mergeUint32ToUint64(std::uint32_t hbyte, std::uint32_t lbyte);

    std::uint64_t pointToVoxelKey(float x, float y, float z) const {
      std::int64_t ix = static_cast<std::int64_t>(std::floor(x / xy_resolution_));
      std::int64_t iy = static_cast<std::int64_t>(std::floor(y / xy_resolution_));
      std::int64_t iz = static_cast<std::int64_t>(std::floor(z / height_resolution_));
      return voxelKeyFromIndices(ix, iy, iz);
    }

    static std::uint64_t voxelKeyFromIndices(std::int64_t ix, std::int64_t iy, std::int64_t iz) {
      return (static_cast<std::uint64_t>(ix & 0x1FFFFF) << 42) |
             (static_cast<std::uint64_t>(iy & 0x1FFFFF) << 21) |
             (static_cast<std::uint64_t>(iz & 0x1FFFFF));
    }

    double get_dGraphValue(const unsigned int index){
      if (dGraph_->graph_.find(index) == dGraph_->graph_.end()){
        RCLCPP_INFO(rclcpp::get_logger(name_), "dGraph is queried without initialization, make sure your sensor topic is published and TF setup is correct.");
        return 0.0;
      }
      return dGraph_->getValue(index);
    };

    //@ groud pcl index, distance to lehal. Everything within this map will be used for line-of-sight check
    std::map<int, double> lethal_map_;
    std::unordered_map<std::uint64_t, PointXYZU64> spatial_hash_grid_;
    
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

    std::uint64_t marking_id_{0};

};


}//end of name space
#endif