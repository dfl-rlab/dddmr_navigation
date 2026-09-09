#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cloud_msgs/msg/cloud_info.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_eigen/tf2_eigen.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

typedef pcl::PointXYZI  PointType;

typedef Eigen::Vector3f Vector3;

const double DEG_TO_RAD = M_PI / 180.0;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct ProjectionOut
{
  pcl::PointCloud<PointType>::Ptr segmented_cloud;
  pcl::PointCloud<PointType>::Ptr outlier_cloud;
  pcl::PointCloud<PointType>::Ptr patched_ground;
  pcl::PointCloud<PointType>::Ptr patched_ground_edge;
  std::string odom_type;
  geometry_msgs::msg::TransformStamped trans_c2s;
  geometry_msgs::msg::TransformStamped trans_b2s;
  geometry_msgs::msg::TransformStamped trans_m2ci;
  cloud_msgs::msg::CloudInfo seg_msg;
  int vertical_scans;
  int horizontal_scans;
  double scan_period;
  tf2::Stamped<tf2::Transform> ideal_sensor_orientation2sensor;
};


struct AssociationOut
{
  pcl::PointCloud<PointType>::Ptr cloud_outlier_last;
  pcl::PointCloud<PointType>::Ptr cloud_corner_last;
  pcl::PointCloud<PointType>::Ptr cloud_surf_last;
  pcl::PointCloud<PointType>::Ptr cloud_patched_ground_last;
  pcl::PointCloud<PointType>::Ptr cloud_patched_ground_edge_last;
  geometry_msgs::msg::TransformStamped trans_c2s;
  geometry_msgs::msg::TransformStamped trans_b2s;
  geometry_msgs::msg::TransformStamped trans_m2ci;
  nav_msgs::msg::Odometry decisive_odometry; //in the lego_loam definition frame
  nav_msgs::msg::Odometry external_odometry; //external odom, odom->baselink frame
  bool broadcast_odom_tf;
  tf2::Stamped<tf2::Transform> ideal_sensor_orientation2sensor;
};

inline void OdometryToTransform(const nav_msgs::msg::Odometry& odometry,
                                float* transform) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odometry.pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  transform[0] = -pitch;
  transform[1] = -yaw;
  transform[2] = roll;

  transform[3] = odometry.pose.pose.position.x;
  transform[4] = odometry.pose.pose.position.y;
  transform[5] = odometry.pose.pose.position.z;
}

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
