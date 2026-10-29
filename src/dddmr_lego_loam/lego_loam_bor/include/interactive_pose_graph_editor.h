#ifndef INTERACTIVEPOSEGRAPHEDITOR_H
#define INTERACTIVEPOSEGRAPHEDITOR_H

#include "utility.h"
#include <pcl/kdtree/kdtree_flann.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
//optimized icp gaussian newton
#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"

//#include MO
#include "mapOptimization.h"

using namespace std::placeholders;

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

using namespace gtsam;

class InteractivePoseGraphEditor : public rclcpp::Node 
{

inline Eigen::Affine3f pclPointToAffine3fCameraToLidar(
    PointTypePose thisPoint) {
  // camera frame to lidar frame
  return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y,
                                thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}

inline gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
  // camera frame to lidar frame
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll),
                          double(thisPoint.pitch)),
      gtsam::Point3(double(thisPoint.z), double(thisPoint.x),
                    double(thisPoint.y)));
}


public:

  InteractivePoseGraphEditor(std::string name, std::shared_ptr<MapOptimization> mo);
  ~InteractivePoseGraphEditor();
  std::string getName(){return name_;}  
  std::shared_ptr<MapOptimization> mo_;

private:
  
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::pair<int, int> current_operation_nodes_;
  Eigen::Affine3f node1_2_node2_af3_;
  Eigen::Affine3d node1_2_node2_af3d_;
  float icp_score_;
  double trans_step_ = 0.5;
  double rot_step_ = 0.1;
  geometry_msgs::msg::TransformStamped trans_node1_2_node2_;

  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames_global_; 
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundCloudKeyFrames_global_; 

  pcl::PointCloud<PointType>::Ptr node_1_pointcloud_;
  pcl::PointCloud<PointType>::Ptr node_2_pointcloud_;
  pcl::PointCloud<PointType>::Ptr node_2_pointcloud_icp_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr node_1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr node_2_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node_selection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_history_keyframe_search_radius_;

  void historyKeyframeSearchRadiusCb(const std_msgs::msg::Float32::SharedPtr msg);
  void nodeSelectionCB(const std_msgs::msg::String::SharedPtr msg);
  void operationCommandCB(const std_msgs::msg::String::SharedPtr msg);
  
  void convert2Global();

};

#endif // INTERACTIVEPOSEGRAPHEDITOR_H
