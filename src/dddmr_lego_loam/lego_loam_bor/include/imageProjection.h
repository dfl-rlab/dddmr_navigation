#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "utility.h"
#include "channel.h"
#include <Eigen/QR>

// for tilted lidar
#include <tf2_eigen/tf2_eigen.hpp>

// get robot frame to sensor frame tf
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"

#include "tf2_ros/static_transform_broadcaster.h"

// ros
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>

// omp voxel
#include "dddmr_pcl/voxel_omp/voxel_grid_omp.h"

#ifdef TRT_ENABLED
#include "dddmr_trt/yolov8.h"
#include <opencv2/cudaimgproc.hpp>
#endif

class ImageProjection : public rclcpp::Node 
{
  public:

    ImageProjection(std::string name, Channel<ProjectionOut>& output_channel);

    ~ImageProjection() = default;
    
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void tfInitial();
    
    bool to_fa_;
    bool pc_valid_;
    std::string mapping_dir_string_;

  private:

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds

    void findStartEndAngle();
    void resetParameters();
    void projectPointCloud();
    void zPitchRollFeatureRemoval();
    void cloudSegmentation();
    void labelComponents(int row, int col);
    void publishClouds();
    bool allEssentialTFReady(std::string sensor_frame);
    void getNoPitchPoint(PointType& pt_in, PointType& pt_out);
    
    pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

    pcl::PointCloud<PointType>::Ptr _full_cloud;
    pcl::PointCloud<PointType>::Ptr _full_info_cloud;

    pcl::PointCloud<PointType>::Ptr _z_pitch_roll_decisive_feature_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
    pcl::PointCloud<PointType>::Ptr _outlier_cloud;
    pcl::PointCloud<PointType>::Ptr patched_ground_;
    pcl::PointCloud<PointType>::Ptr patched_ground_edge_;
    pcl::PointCloud<PointType>::Ptr yolo_labelled_point_cloud_;

    pcl::VoxelGrid<PointType> dsf_patched_ground_;
    pcl::VoxelGridOMP dsf_patched_ground_omp_;
    pcl::VoxelGridOMP dsf_patched_ground_edge_omp_;

    int _vertical_scans;
    int _horizontal_scans;
    double _scan_period;
    float _ang_bottom;
    float _ang_resolution_X;
    float _ang_resolution_Y;
    float _segment_theta;
    int _segment_valid_point_num;
    int _segment_valid_line_num;
    std::string odom_type_;
    std::string base_ground_frame_, sensor_frame_;

    Channel<ProjectionOut>& _output_channel;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_ground_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud_pure;
    rclcpp::Publisher<cloud_msgs::msg::CloudInfo>::SharedPtr _pub_segmented_cloud_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloud;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_projected_image;
    
    cloud_msgs::msg::CloudInfo _seg_msg;

    int _label_count;
    
    cv::Mat range_mat_removing_moving_object_;
    Eigen::MatrixXf _range_mat;   // range matrix for range image
    Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
    Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking

    float _maximum_detection_range;
    float _minimum_detection_range;
    double distance_for_patch_between_rings_;
    int first_frame_processed_;
    bool got_baselink2sensor_tf_;
    geometry_msgs::msg::TransformStamped trans_b2s_;
    tf2::Transform tf2_trans_b2s_, tf2_trans_c2s_;
    geometry_msgs::msg::TransformStamped trans_c2s_;
    geometry_msgs::msg::TransformStamped trans_c2b_;
    geometry_msgs::msg::TransformStamped trans_m2ci_;
    
    //@ list of pointcloud sticher for non-repetitive scan lidar
    std::list<pcl::PointCloud<PointType>> pcl_stitcher_;    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    double last_save_depth_img_time_;
    double time_step_between_depth_image_;
    
    int stitcher_num_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_img_;
    
    bool is_trt_engine_exist_;
    std::string trt_model_path_;
    int projected_image_stack_size_;
    std::deque<cv::Mat> projected_image_queue_;
    
    double sensor_install_pitch_;
    double ground_slope_tolerance_, ground_dz_tolerance_;
    
    bool patch_first_ring_to_baselink_;

    double ground_fov_bottom_;
    double ground_fov_top_;
    double ground_positive_start_;
    double ground_positive_stop_;
    double ground_negative_start_;
    double ground_negative_stop_;

    double ignore_fov_bottom_;
    double ignore_fov_top_;
    double ignore_positive_start_;
    double ignore_positive_stop_;
    double ignore_negative_start_;
    double ignore_negative_stop_;

    bool use_sensor_height_to_filter_out_ground_;
    
#ifdef TRT_ENABLED
    std::shared_ptr<YoloV8> yolov8_;
#endif
};

#endif  // IMAGEPROJECTION_H
