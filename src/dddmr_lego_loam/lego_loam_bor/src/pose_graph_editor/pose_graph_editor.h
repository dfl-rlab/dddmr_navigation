#ifndef POSEGRAPHEDITOR_H
#define POSEGRAPHEDITOR_H

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

//@allows us to use pcl::transformPointCloud function
#include <pcl/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

//@ for mkdir
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

//@ for kd tree, used to enhance loop closure robust, we use line of sight test
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "tf2_ros/static_transform_broadcaster.h"

//@string split
#include <sstream>
//@Create folder
#include <filesystem>

//optimized icp gaussian newton
#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"

using namespace std::placeholders;

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

using namespace gtsam;

inline std::string currentDateTime() {
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);
 
    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", now);
    return buffer;
}

class PoseGraphEditor : public rclcpp::Node 
{

typedef pcl::PointCloud<PointTypePose> point_cloud_t;
typedef std::set<std::pair<int, int>> edge_t;
public:
  PoseGraphEditor(std::string name, std::string pose_graph_dir);
    
  ~PoseGraphEditor();

  //@all about merge
  std::string getName(){return name_;}  

  //@ for merge to easily access, we made following as public
  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames_; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames_baselink_; //original frame from files are base_link
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames; //this frame is converted to global using poses_
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames_baselink_; //original frame from files are base_link
  point_cloud_t poses_; //Operating poses, we maitain this poses all the time
  edge_t edges_; //Operating poses, we maitain this edges all the time

private:

  std::string name_, pose_graph_dir_;
  rclcpp::Clock::SharedPtr clock_;

  gtsam::ISAM2 *isam_;
  ISAM2Params parameters_;

  gtsam::NonlinearFactorGraph gtSAMgraph_;
  gtsam::Values initialEstimate;
  gtsam::Values optimizedEstimate;
  gtsam::Values isamCurrentEstimate;

  gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
  gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;

  pcl::PointCloud<PointTypePose>::Ptr pcd_poses_; //original poses from files
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_edges_; //original edges from files
  pcl::PointCloud<PointType>::Ptr node_1_pointcloud_;
  pcl::PointCloud<PointType>::Ptr node_2_pointcloud_;
  pcl::PointCloud<PointType>::Ptr node_2_pointcloud_icp_;
  edge_t removed_edges_;
  edge_t selected_edges_;
  std::vector<std::pair<point_cloud_t, edge_t>> history_operation_;
  std::pair<int, int> current_operation_nodes_;
  Eigen::Affine3d node1_2_node2_af3d_;
  float icp_score_;


  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr node_1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr node_2_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_pub_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr edge_selection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node_selection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_focus_pg_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_command_sub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  void readPoseGraph();
  void pubTF();
  void pubGraph();

  //@ generateISAM generate ISAM by original poses and edges, in case we want to reset everything, so we isolate this function
  void generateISAM();
  
  //@ generate ISAM by edited pose_graph_ and update poses after isam_->calculateEstimate()
  void generateISAMwiPosesEdges();
  
  void edgeSelectionCB(const std_msgs::msg::String::SharedPtr msg);
  void nodeSelectionCB(const std_msgs::msg::String::SharedPtr msg);
  void operationCommandCB(const std_msgs::msg::String::SharedPtr msg);
  void currentFocusCB(const std_msgs::msg::String::SharedPtr msg);

  gtsam::Vector getOdometryNoise(double m_value);
  void add2History();
  void popHistory();

  pcl::PointCloud<PointType>::Ptr map_;
  pcl::PointCloud<PointType>::Ptr ground_;
  std::string current_focus_pg_;
  
  Eigen::Affine3f first_frame_2_second_frame_af3_;
  double trans_step_ = 0.5;
  double rot_step_ = 0.1;
  
};

#endif // PoseGraphEditor_H
