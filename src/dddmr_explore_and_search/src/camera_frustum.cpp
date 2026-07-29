#include "dddmr_explore_and_search/camera_frustum.hpp"
#include <cmath>

CameraFrustum::CameraFrustum(){

}

CameraFrustum::CameraFrustum(double v_fov, double h_fov, double z_near, double z_far, geometry_msgs::msg::TransformStamped t)
: FOV_V_(v_fov), FOV_W_(h_fov), min_detect_distance_(z_near), max_detect_distance_(z_far), t_(t)
{
  findFrustumVertex();
  tf2_baseframe2camera_.setOrigin(tf2::Vector3(t_.transform.translation.x, t_.transform.translation.y, t_.transform.translation.z));
  tf2_baseframe2camera_.setRotation(tf2::Quaternion(t_.transform.rotation.x, t_.transform.rotation.y, t_.transform.rotation.z, t_.transform.rotation.w));
}

void CameraFrustum::convertFrustum2GlobalFrame(const geometry_msgs::msg::Pose& pose){
  tf2_global_pose_.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf2_global_pose_.setRotation(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
  tf2_global2camera_.mult(tf2_global_pose_,tf2_baseframe2camera_);
  geometry_msgs::msg::TransformStamped trans_global2camera;
  trans_global2camera.transform.translation.x = tf2_global2camera_.getOrigin().x();
  trans_global2camera.transform.translation.y = tf2_global2camera_.getOrigin().y();
  trans_global2camera.transform.translation.z = tf2_global2camera_.getOrigin().z();
  trans_global2camera.transform.rotation.x = tf2_global2camera_.getRotation().x();
  trans_global2camera.transform.rotation.y = tf2_global2camera_.getRotation().y();
  trans_global2camera.transform.rotation.z = tf2_global2camera_.getRotation().z();
  trans_global2camera.transform.rotation.w = tf2_global2camera_.getRotation().w();
  Eigen::Affine3d trans_m2c_af3 = tf2::transformToEigen(trans_global2camera);
  pcl::transformPointCloud(frustum_, frustum_, trans_m2c_af3);
  findFrustumNormal();
  findFrustumPlane();
}

void CameraFrustum::findFrustumVertex()
{
  frustum_.clear();
  frustum_.push_back(pcl::PointXYZ(min_detect_distance_, min_detect_distance_*tan(FOV_W_/2.0), min_detect_distance_*tan(FOV_V_/2.0)));
  frustum_.push_back(pcl::PointXYZ(min_detect_distance_, -min_detect_distance_*tan(FOV_W_/2.0), min_detect_distance_*tan(FOV_V_/2.0)));
  frustum_.push_back(pcl::PointXYZ(min_detect_distance_, min_detect_distance_*tan(FOV_W_/2.0), -min_detect_distance_*tan(FOV_V_/2.0)));
  frustum_.push_back(pcl::PointXYZ(min_detect_distance_, -min_detect_distance_*tan(FOV_W_/2.0), -min_detect_distance_*tan(FOV_V_/2.0)));
  //-------------------------------------------
  frustum_.push_back(pcl::PointXYZ(max_detect_distance_, max_detect_distance_*tan(FOV_W_/2.0), max_detect_distance_*tan(FOV_V_/2.0)));
  frustum_.push_back(pcl::PointXYZ(max_detect_distance_, -max_detect_distance_*tan(FOV_W_/2.0), max_detect_distance_*tan(FOV_V_/2.0)));
  frustum_.push_back(pcl::PointXYZ(max_detect_distance_, max_detect_distance_*tan(FOV_W_/2.0), -max_detect_distance_*tan(FOV_V_/2.0)));
  frustum_.push_back(pcl::PointXYZ(max_detect_distance_, -max_detect_distance_*tan(FOV_W_/2.0), -max_detect_distance_*tan(FOV_V_/2.0)));
}

pcl::PointXYZ CameraFrustum::getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2)
{
  pcl::PointXYZ vec;
  vec.x = vec2.x - vec1.x;
  vec.y = vec2.y - vec1.y;
  vec.z = vec2.z - vec1.z;
  return vec;
}

pcl::PointXYZ CameraFrustum::getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2)
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
void CameraFrustum::getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) 
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

void CameraFrustum::findFrustumNormal()
{

  BRNear_ = frustum_.points[3];
  TLFar_ = frustum_.points[4];

  frustum_normal_.clear();

  pcl::PointXYZ TLNear,TRNear, BLNear, BRNear;

  TLNear = frustum_.points[0];

  TRNear = frustum_.points[1];

  BLNear = frustum_.points[2];

  BRNear = frustum_.points[3];


  pcl::PointXYZ TLFar,TRFar, BLFar, BRFar;

  TLFar = frustum_.points[4];

  TRFar = frustum_.points[5];

  BLFar = frustum_.points[6];

  BRFar = frustum_.points[7];

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
  frustum_normal_.push_back(pn);
  ////
  TRN2TRF = getVec(TRNear,TRFar);
  TRF2BRF = getVec(TRFar,BRFar);
  pr = getCrossProduct(TRN2TRF, TRF2BRF);
  frustum_normal_.push_back(pr);
  ////
  BRN2BRF = getVec(BRNear,BRFar);
  BRF2BLF = getVec(BRFar,BLFar);
  pb = getCrossProduct(BRN2BRF,BRF2BLF);
  frustum_normal_.push_back(pb);
  ////

  BLN2BLF = getVec(BLNear,BLFar);
  BLF2TLF = getVec(BLFar,TLFar);
  pl = getCrossProduct(BLN2BLF, BLF2TLF);
  frustum_normal_.push_back(pl);
  ////
  BRF2TRF = getVec(BRFar,TRFar);
  TRF2TLF = getVec(TRFar,TLFar);
  pf = getCrossProduct(BRF2TRF, TRF2TLF);
  frustum_normal_.push_back(pf);
  ////
  TLF2TRF = getVec(TLFar,TRFar);
  TRF2TRN = getVec(TRFar,TRNear);
  pt = getCrossProduct(TLF2TRF, TRF2TRN);
  frustum_normal_.push_back(pt);
    
}

void CameraFrustum::findFrustumPlane()
{
  //frustum_plane_equation_.clear();

  pcl::PointXYZ TLNear,TRNear, BLNear, BRNear;
  TLNear = frustum_.points[0];
  TRNear = frustum_.points[1];
  BLNear = frustum_.points[2];
  BRNear = frustum_.points[3];


  pcl::PointXYZ TLFar,TRFar, BLFar, BRFar;
  TLFar = frustum_.points[4];
  TRFar = frustum_.points[5];
  BLFar = frustum_.points[6];
  BRFar = frustum_.points[7];

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

bool CameraFrustum::isPointInFrustum(pcl::PointXYZ& testPoint){

  bool one_frustum_test = true; //if we test the point in one of the frustum, then we can exit
  pcl::PointXYZ vec_form_pc2corner;
  double test;
  for(int i=0;i<6;i++)
  {
    test = 0.0;
    if(i<3)
    {
      vec_form_pc2corner = getVec(BRNear_, testPoint);
      
      test = vec_form_pc2corner.x*frustum_normal_.points[i].x + vec_form_pc2corner.y*frustum_normal_.points[i].y +vec_form_pc2corner.z*frustum_normal_.points[i].z;
      
      if(test<0)
      {
        one_frustum_test = false;
        break;
      }
    }
    else
    {
      vec_form_pc2corner = getVec(TLFar_, testPoint);
      test = vec_form_pc2corner.x*frustum_normal_.points[i].x + vec_form_pc2corner.y*frustum_normal_.points[i].y +vec_form_pc2corner.z*frustum_normal_.points[i].z;
      if(test<0)
      {
        one_frustum_test = false;
        break;
      }
    }
  }
  if(one_frustum_test)
  {
    return true;
  }
  return false;
}