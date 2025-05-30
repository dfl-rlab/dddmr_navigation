#ifndef MAPOPTIMIZATION_H
#define MAPOPTIMIZATION_H

#include "utility.h"
#include "channel.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "std_srvs/srv/empty.hpp"

//@allows us to use pcl::transformPointCloud function
#include <pcl/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

//optimized icp gaussian newton
#include "opt_icp_gn/optimized_ICP_GN.h"
#include "opt_icp_gn/common.h"

//optimized pcl transform
#include "transforms.hpp"

using namespace std::placeholders;

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

inline gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
  // camera frame to lidar frame
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll),
                          double(thisPoint.pitch)),
      gtsam::Point3(double(thisPoint.z), double(thisPoint.x),
                    double(thisPoint.y)));
}

inline Eigen::Affine3f pclPointToAffine3fCameraToLidar(
    PointTypePose thisPoint) {
  // camera frame to lidar frame
  return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y,
                                thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}


inline std::string currentDateTime() {
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);
 
    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", now);
    return buffer;
}
 

class MapOptimization : public rclcpp::Node 
{

 public:
  MapOptimization(std::string name, Channel<AssociationOut> &input_channel);
  
  ~MapOptimization();

  void pcdSaver(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
          
  void run();
  void publishGlobalMapThread();
  void loopClosureThread();

  nav_msgs::msg::Odometry odomAftMapped;
  float _history_keyframe_fitness_score;
  float _history_keyframe_search_radius;
  
  // move following to public for interactive pose graph editor
  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; 
  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFramesVisualization; 
  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFramesVisualization_Copy;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundKeyFrames_Copy;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundKeyFramesVisualization;

  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_Copy;
  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  pcl::PointCloud<PointType>::Ptr transformPointCloudInverse(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  void correctPoses();
  void addEdgeFromPose(int pose_1, int pose_2, gtsam::Pose3 poseFrom, gtsam::Pose3 poseTo, float icp_score);
  void publishKeyPosesAndFrames();
  void groundEdgeDetectionThread();
  void copyPosesAndFrames();

 private:
   
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_key_pose_arr_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph_;
  //@ relative pose between current pose and nearest key frame for initial guess to opt_icp_gn
  tf2::Transform tf2_current2closestKeyFrame_;
  //@ affine for final transformation to map when outputing pose graph
  Eigen::Affine3d trans_m2ci_af3_, trans_c2s_af3_, trans_s2c_af3_;
  tf2::Stamped<tf2::Transform> tf2_trans_m2ci_;
  tf2::Stamped<tf2::Transform> tf2_trans_c2s_; //camera2sensorlink
  std::set<std::pair<int, int>> pose_graph_;

  gtsam::NonlinearFactorGraph gtSAMgraph;
  gtsam::Values initialEstimate;
  gtsam::Values optimizedEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isamCurrentEstimate;

  gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
  gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;

  bool _loop_closure_enabled;

  int   _surrounding_keyframe_search_num;
  int   _history_keyframe_search_num;
  float _global_map_visualization_search_radius;

  Channel<AssociationOut>& _input_channel;
  
  rclcpp::CallbackGroup::SharedPtr timer_run_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_pub_gbl_map_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_loop_closure_cb_group_;

  rclcpp::TimerBase::SharedPtr timer_run_;
  rclcpp::TimerBase::SharedPtr timer_pub_gbl_map_;
  rclcpp::TimerBase::SharedPtr timer_loop_closure_;
  rclcpp::TimerBase::SharedPtr timer_ground_edge_detection_;

  rclcpp::Clock::SharedPtr clock_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGround;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGroundEdge;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpTargetKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentCornerKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentSurfKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLastOptimizedCornerKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLastOptimizedSurfKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSelectedCloudForLMOptimization;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvSavePCD;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped aftMappedTrans;
  
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundEdgeKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundEdgeProcessedKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> patchedGroundEdgeProcessedKeyFrames_Copy;

  std::deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID;

  std::vector<int> surroundingExistingKeyPosesID;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

  PointTypePose previousRobotPos_;
  PointTypePose currentRobotPos_;
  PointType currentRobotPosPoint_;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;

  pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
  pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudPatchedGroundLast;  // for ground pcd stitching
  pcl::PointCloud<PointType>::Ptr
      laserCloudPatchedGroundEdgeLast;  // for ground pcd stitching
  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLastDS;  // downsampled corner featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLastDS;  // downsampled surf featuer set from
      // odoOptimization

  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLastDS;  // corner feature set from odoOptimization

  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLastDS;  // downsampled corner featuer set from
      // odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

  pcl::KdTreeFLANN<PointType> kdtreeCornerFromMap;
  pcl::KdTreeFLANN<PointType> kdtreeSurfFromMap;

  pcl::KdTreeFLANN<PointType> kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType> kdtreeHistoryKeyPoses;

  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;
  pcl::PointCloud<PointType>::Ptr globalGroundKeyFrames;
  pcl::PointCloud<PointType>::Ptr completeGlobalStitch; //for final stitch
  pcl::PointCloud<PointType>::Ptr keyFrameCorner; //for exporting key frame: cornerCloudKeyFrames
  pcl::PointCloud<PointType>::Ptr keyFrameGround; //for exporting key frame: surfFlatCloudKeyFrames
  pcl::PointCloud<PointType>::Ptr keyFrameSurface; //for exporting key frame: surfCloudKeyFrames

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType>
      downSizeFilterHistoryKeyFrames;  // for histor key frames of loop closure
      // scan-to-map optimization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyPoses;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyFrames;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalGroundKeyFrames_Copy;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalGroundKeyFrames;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterFinalStitch;  // DS for final stitch
  
  std_msgs::msg::Header timeLaserOdometry_header_;
  double timeLaserOdometry;
  double timeLastGloalMapPublish;
  double distance_between_key_frame_, angle_between_key_frame_;

  float transformLast[6];
  float transformSum[6];
  float transformIncre[6];
  float transformTobeMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std::mutex mtx;

  double timeLastProcessing;

  PointType pointOri, pointSel, pointProj, coeff;

  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;

  Eigen::Matrix3f matA1;
  Eigen::Matrix<float, 1, 3> matD1;
  Eigen::Matrix3f matV1;

  Eigen::Matrix<float, 6, 6> matP;

  bool isDegenerate;

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;

  bool potentialLoopFlag;
  double timeSaveFirstCurrentScanForLoopClosure;
  int closestHistoryFrameID;
  int latestFrameIDLoopCloure;

  bool aLoopIsClosed;

  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;

 private:

  void allocateMemory();
  void transformAssociateToMap();
  void transformUpdate();
  void updatePointAssociateToMapSinCos();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);

  void publishTF();
  void publishGlobalMap();

  bool detectLoopClosure();
  void performLoopClosure();

  void extractSurroundingKeyFrames();

  void downsampleCurrentScan();
  void cornerOptimization(int iterCount);
  void surfOptimization(int iterCount);

  bool LMOptimization(int iterCount);
  void scan2MapOptimization();

  void saveKeyFramesAndFactor();

  void clearCloud();
  
  double ground_voxel_size_;
  int ground_edge_threshold_num_;

  std::vector<bool> ground_edge_processed_;

};

#endif // MAPOPTIMIZATION_H
