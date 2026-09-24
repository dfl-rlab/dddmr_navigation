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
#ifndef DEPTH_CAMERA_OBSERVATION_H_
#define DEPTH_CAMERA_OBSERVATION_H_

#include <geometry_msgs/msg/point.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace perception_3d
{

class PointCloudCluster{

  public:
    pcl::PointCloud<pcl::PointXYZI> point_cloud_;
    pcl::PointXYZ centroid_;
};

class DepthCameraObservation
{
public:

  DepthCameraObservation(const sensor_msgs::msg::PointCloud2& cloud);
  //DepthCameraObservation(const DepthCameraObservation& obs);

  virtual ~DepthCameraObservation();

  pcl::PointXYZ getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  pcl::PointXYZ getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  void getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);
  void findFrustumVertex();
  void findFrustumNormal();
  void findFrustumPlane();



  geometry_msgs::msg::Point origin_;
  //@ cloud_ is in global frame after min/max height check and distance check
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  //@ raw_cloud_ is in robot frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_i_;
  //@ everything related to frustum is in global frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr frustum_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr frustum_normal_;
  std::vector<Eigen::Vector4f> frustum_plane_equation_;
  //@ clusters storage
  std::vector<perception_3d::PointCloudCluster> clusters_;

  double FOV_V_;
  double FOV_W_;
  double min_detect_distance_;
  double max_detect_distance_;

  pcl::PointXYZ BRNear_;
  pcl::PointXYZ TLFar_;
  

};
}
#endif  // DEPTH_CAMERA_OBSERVATION_H_
