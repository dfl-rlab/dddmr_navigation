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
#include <rclcpp/rclcpp.hpp>
#include <perception_3d/depth_camera/depth_camera_observation.h>

namespace perception_3d
{
  DepthCameraObservation::DepthCameraObservation(const sensor_msgs::msg::PointCloud2& cloud) 
  : raw_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
  , raw_cloud_i_(new pcl::PointCloud<pcl::PointXYZI>())
  , cloud_(new pcl::PointCloud<pcl::PointXYZI>())
  , frustum_(new pcl::PointCloud<pcl::PointXYZ>())
  , frustum_normal_(new pcl::PointCloud<pcl::PointXYZ>())
  , frustum_plane_equation_()
  , FOV_V_(1.0)
  , FOV_W_(1.5)
  , min_detect_distance_(0.01)
  , max_detect_distance_(2.5)
  {
    //copyPointCloud(cloud_xyz, cloud_xyzrgb);
    pcl::fromROSMsg(cloud, *raw_cloud_);
  }
  
  /*
  DepthCameraObservation::DepthCameraObservation(const DepthCameraObservation& obs) 
  : origin_(obs.origin_)
  , cloud_(new pcl::PointCloud<pcl::PointXYZI>(*(obs.cloud_)))
  , frustum_(new pcl::PointCloud<pcl::PointXYZ>(*(obs.frustum_)))
  , frustum_normal_(new pcl::PointCloud<pcl::PointXYZ>(*(obs.frustum_normal_)))
  , frustum_plane_equation_(obs.frustum_plane_equation_)
  , FOV_V_(obs.FOV_V_)
  , FOV_W_(obs.FOV_W_)
  , min_detect_distance_(obs.min_detect_distance_)
  , max_detect_distance_(obs.max_detect_distance_)
  , BRNear_(obs.BRNear_)
  , TLFar_(obs.TLFar_)
  {

  }
  */
  DepthCameraObservation::~DepthCameraObservation()
  {
    raw_cloud_.reset();
    cloud_.reset();
    raw_cloud_i_.reset();
    frustum_.reset();
    frustum_normal_.reset();
  }

  pcl::PointXYZ DepthCameraObservation::getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2)
  {
    pcl::PointXYZ vec;
    vec.x = vec2.x - vec1.x;
    vec.y = vec2.y - vec1.y;
    vec.z = vec2.z - vec1.z;
    return vec;
  }

  pcl::PointXYZ DepthCameraObservation::getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2)
  {
    pcl::PointXYZ crsp;
    crsp.x = vec1.y * vec2.z - vec1.z * vec2.y;
    crsp.y = (vec1.x * vec2.z - vec1.z * vec2.x)*-1.0;
    crsp.z = vec1.x * vec2.y - vec1.y * vec2.x;
    //cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]; 
    //cross_P[1] = vect_A[0] * vect_B[2] - vect_A[2] * vect_B[0]; 
    //cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]; 

    return crsp;
  }

  /// Function to find equation of plane. 
  void DepthCameraObservation::getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) 
  { 
    plane_equation = Eigen::Vector4f::Zero();
    float a1 = p2.x - p1.x; 
    float b1 = p2.y - p1.y; 
    float c1 = p2.z - p1.z; 
    float a2 = p3.x - p1.x; 
    float b2 = p3.y - p1.y; 
    float c2 = p3.z - p1.z; 
    plane_equation[0] = b1 * c2 - b2 * c1; 
    plane_equation[1] = a2 * c1 - a1 * c2; 
    plane_equation[2] = a1 * b2 - b1 * a2; 
    plane_equation[3] = (-plane_equation[0] * p1.x - plane_equation[1] * p1.y - plane_equation[2] * p1.z); 
  }

  void DepthCameraObservation::findFrustumVertex()
  {

    frustum_->clear();
    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, min_detect_distance_*tan(FOV_W_/2.0), min_detect_distance_*tan(FOV_V_/2.0)));
    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, -min_detect_distance_*tan(FOV_W_/2.0), min_detect_distance_*tan(FOV_V_/2.0)));
    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, min_detect_distance_*tan(FOV_W_/2.0), -min_detect_distance_*tan(FOV_V_/2.0)));
    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, -min_detect_distance_*tan(FOV_W_/2.0), -min_detect_distance_*tan(FOV_V_/2.0)));
    //-------------------------------------------
    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, max_detect_distance_*tan(FOV_W_/2.0), max_detect_distance_*tan(FOV_V_/2.0)));
    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, -max_detect_distance_*tan(FOV_W_/2.0), max_detect_distance_*tan(FOV_V_/2.0)));
    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, max_detect_distance_*tan(FOV_W_/2.0), -max_detect_distance_*tan(FOV_V_/2.0)));
    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, -max_detect_distance_*tan(FOV_W_/2.0), -max_detect_distance_*tan(FOV_V_/2.0)));
  }
 

  void DepthCameraObservation::findFrustumNormal()
  {

    BRNear_ = frustum_->points[3];
    TLFar_ = frustum_->points[4];

    frustum_normal_->clear();

    pcl::PointXYZ TLNear,TRNear, BLNear, BRNear;

    TLNear = frustum_->points[0];

    TRNear = frustum_->points[1];

    BLNear = frustum_->points[2];

    BRNear = frustum_->points[3];


    pcl::PointXYZ TLFar,TRFar, BLFar, BRFar;

    TLFar = frustum_->points[4];

    TRFar = frustum_->points[5];

    BLFar = frustum_->points[6];

    BRFar = frustum_->points[7];

    pcl::PointXYZ TLN2TRN, TRN2BRN; //note it is vector, we manipulate point as vector
    pcl::PointXYZ TRN2TRF, TRF2BRF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ BRN2BRF, BRF2BLF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ BLN2BLF, BLF2TLF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ BRF2TRF, TRF2TLF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ TLF2TRF, TRF2TRN; //note it is vector, we manipulate point as vector

    pcl::PointXYZ pt, pb, pl, pr, pn, pf;

    ////
    TLN2TRN = getVec(TLNear,TRNear);
    TRN2BRN = getVec(TRNear,BRNear);
    pn = getCrossProduct(TLN2TRN, TRN2BRN);
    frustum_normal_->push_back(pn);
    ////
    TRN2TRF = getVec(TRNear,TRFar);
    TRF2BRF = getVec(TRFar,BRFar);
    pr = getCrossProduct(TRN2TRF, TRF2BRF);
    frustum_normal_->push_back(pr);
    ////
    BRN2BRF = getVec(BRNear,BRFar);
    BRF2BLF = getVec(BRFar,BLFar);
    pb = getCrossProduct(BRN2BRF,BRF2BLF);
    frustum_normal_->push_back(pb);
    ////

    BLN2BLF = getVec(BLNear,BLFar);
    BLF2TLF = getVec(BLFar,TLFar);
    pl = getCrossProduct(BLN2BLF, BLF2TLF);
    frustum_normal_->push_back(pl);
    ////
    BRF2TRF = getVec(BRFar,TRFar);
    TRF2TLF = getVec(TRFar,TLFar);
    pf = getCrossProduct(BRF2TRF, TRF2TLF);
    frustum_normal_->push_back(pf);
    ////
    TLF2TRF = getVec(TLFar,TRFar);
    TRF2TRN = getVec(TRFar,TRNear);
    pt = getCrossProduct(TLF2TRF, TRF2TRN);
    frustum_normal_->push_back(pt);
      
  }

  void DepthCameraObservation::findFrustumPlane()
  {
    //frustum_plane_equation_.clear();

    pcl::PointXYZ TLNear,TRNear, BLNear, BRNear;
    TLNear = frustum_->points[0];
    TRNear = frustum_->points[1];
    BLNear = frustum_->points[2];
    BRNear = frustum_->points[3];


    pcl::PointXYZ TLFar,TRFar, BLFar, BRFar;
    TLFar = frustum_->points[4];
    TRFar = frustum_->points[5];
    BLFar = frustum_->points[6];
    BRFar = frustum_->points[7];

    //we manipulate orientation x,y,z,w as a,b,c,d in plane equation
    Eigen::Vector4f plane1;
    Eigen::Vector4f plane2;
    Eigen::Vector4f plane3;
    Eigen::Vector4f plane4;
    Eigen::Vector4f plane5;
    Eigen::Vector4f plane6;
      
    getPlaneN(plane1,TLNear,TLFar,BLNear);
    getPlaneN(plane2,BLNear,BRNear,BLFar);
    getPlaneN(plane3,TRNear,BRNear,BRFar);
    getPlaneN(plane4,TLNear,TRNear,TLFar);
    getPlaneN(plane5,TLNear,BLNear,BRNear);
    getPlaneN(plane6,TLFar,TRFar,BRFar);
    frustum_plane_equation_.push_back(plane1);
    frustum_plane_equation_.push_back(plane2);
    frustum_plane_equation_.push_back(plane3);
    frustum_plane_equation_.push_back(plane4);
    frustum_plane_equation_.push_back(plane5);
    frustum_plane_equation_.push_back(plane6);
  }

}
