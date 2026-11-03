#ifndef IMAGEPROJECTIONTOOL_H
#define IMAGEPROJECTIONTOOL_H

#include "utility.h"
#include "channel.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/QR>

// for tilted lidar
#include <tf2_eigen/tf2_eigen.hpp>

// get robot frame to sensor frame tf
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"

#include "tf2_ros/static_transform_broadcaster.h"

//auto detect cv bridge version to prepare for jazzy
#ifdef ROS_JAZZY
    // ROS 2 Iron, Jazzy, and newer use the .hpp extension
    #include <cv_bridge/cv_bridge.hpp>
#else
    // ROS 2 Humble and older use the .h extension
    #include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <filesystem>

// omp voxel
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>


class ImageProjectionTool : public rclcpp::Node 
{
  public:

    ImageProjectionTool(std::string name, Channel<ProjectionOut>& output_channel);

    ~ImageProjectionTool() = default;
    
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    pcl::PointCloud<PointType>::Ptr filterPointCloud(const pcl::PointCloud<PointType>& cloud_in);
    void publishProjectedPointClouds();
    void projectImageBySelection();
    void exportProjectedImages();

    
    bool to_fa_;
    bool pc_valid_;
    std::string mapping_dir_string_;
    pcl::PointCloud<PointType>::Ptr selected_cloud_;
    cv::Mat projected_stack_result_;
    cv::Mat projected_selected_stack_result_;
    float xmax_{50.0f}, xmin_{-50.0f}, ymax_{50.0f}, ymin_{-50.0f}, zmax_{50.0f}, zmin_{-50.0f};
    std::string base_ground_frame_, sensor_frame_;
    bool got_baselink2sensor_tf_;
    geometry_msgs::msg::TransformStamped trans_b2s_;
    tf2::Stamped<tf2::Transform> tf2_trans_b2s_, tf2_trans_c2s_;

  private:

    rclcpp::Clock::SharedPtr clock_;

    void resetParameters();
    void projectPointCloud();
    
    pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

    pcl::PointCloud<PointType>::Ptr _full_cloud;
    pcl::PointCloud<PointType>::Ptr _full_info_cloud;

    pcl::PointCloud<PointType>::Ptr _z_pitch_roll_decisive_feature_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
    pcl::PointCloud<PointType>::Ptr _outlier_cloud;
    pcl::PointCloud<PointType>::Ptr patched_ground_;
    pcl::PointCloud<PointType>::Ptr patched_ground_edge_;
    pcl::PointCloud<PointType>::Ptr ds_patched_ground_;
    pcl::PointCloud<PointType>::Ptr ds_patched_ground_edge_;
    pcl::PointCloud<PointType>::Ptr yolo_labelled_point_cloud_;

    pcl::VoxelGrid<PointType> dsf_patched_ground_;

    size_t cloud_size_;
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

    Channel<ProjectionOut>& _output_channel;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_projected_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_projected_selected_image;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> point_cloud_publishers_vector_;

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
    std::deque<pcl::PointCloud<PointType>> projected_pc_queue_;
    
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
    
    double yaw_correction_;
    double pitch_correction_;
    tf2::Stamped<tf2::Transform> ideal_sensor_orientation2sensor_;


};

#endif  // IMAGEPROJECTIONTOOL_H
