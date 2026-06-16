#ifndef DDDMR_EXPLORE_AND_SEARCH_CAMERA_FRUSTUM_HPP_
#define DDDMR_EXPLORE_AND_SEARCH_CAMERA_FRUSTUM_HPP_

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

class CameraFrustum
{
public:
  CameraFrustum();
  CameraFrustum(double v_fov, double h_fov, double z_near, double z_far, geometry_msgs::msg::TransformStamped t);

  void convertFrustum2GlobalFrame(const geometry_msgs::msg::Pose& t);
  bool isPointInFrustum(pcl::PointXYZ& testPoint);
  
  pcl::PointCloud<pcl::PointXYZ> frustum_;
private:

  pcl::PointXYZ getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  pcl::PointXYZ getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  void getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);
  void findFrustumVertex();
  void findFrustumNormal();
  void findFrustumPlane();

  double FOV_V_;
  double FOV_W_;
  double min_detect_distance_;
  double max_detect_distance_;
  geometry_msgs::msg::TransformStamped t_;
  geometry_msgs::msg::TransformStamped global_pose_;
  pcl::PointCloud<pcl::PointXYZ> frustum_normal_;
  std::vector<Eigen::Vector4f> frustum_plane_equation_;

  pcl::PointXYZ BRNear_;
  pcl::PointXYZ TLFar_;

  tf2::Transform tf2_global_pose_, tf2_baseframe2camera_, tf2_global2camera_;

};

#endif  // DDDMR_EXPLORE_AND_SEARCH_CAMERA_FRUSTUM_HPP_
