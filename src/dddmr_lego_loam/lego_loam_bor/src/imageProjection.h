#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "utility.h"
#include "channel.h"
#include <Eigen/QR>

// for tilted lidar
#include <tf2_eigen/tf2_eigen.hpp>

// pub camera to sensor frame
#include "tf2_ros/static_transform_broadcaster.h"

class ImageProjection : public rclcpp::Node 
{
  public:

    ImageProjection(std::string name, Channel<ProjectionOut>& output_channel);

    ~ImageProjection() = default;
    
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  private:

    rclcpp::Clock::SharedPtr clock_;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    void pubCamera2Sensor(std::string frame_id);
    void findStartEndAngle();
    void resetParameters();
    void projectPointCloud();
    void groundRemoval();
    void cloudSegmentation();
    void labelComponents(int row, int col);
    void publishClouds();

    pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

    pcl::PointCloud<PointType>::Ptr _full_cloud;
    pcl::PointCloud<PointType>::Ptr _full_info_cloud;

    pcl::PointCloud<PointType>::Ptr _ground_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud;
    pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
    pcl::PointCloud<PointType>::Ptr _outlier_cloud;
    pcl::PointCloud<PointType>::Ptr patched_ground_;
    pcl::PointCloud<PointType>::Ptr patched_ground_edge_;

    pcl::VoxelGrid<PointType> dsf_patched_ground_;

    int _vertical_scans;
    int _horizontal_scans;
    float _ang_bottom;
    float _ang_resolution_X;
    float _ang_resolution_Y;
    float _segment_alpha_X;
    float _segment_alpha_Y;
    float _segment_theta;
    int _segment_valid_point_num;
    int _segment_valid_line_num;
    int _ground_scan_index;
    float _sensor_mount_angle;

    Channel<ProjectionOut>& _output_channel;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_ground_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud_pure;
    rclcpp::Publisher<cloud_msgs::msg::CloudInfo>::SharedPtr _pub_segmented_cloud_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloud;
    
    cloud_msgs::msg::CloudInfo _seg_msg;

    int _label_count;

    Eigen::MatrixXf _range_mat;   // range matrix for range image
    Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
    Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking

    float _maximum_detection_range;
    float _minimum_detection_range;
    double distance_for_patch_between_rings_;
};



#endif  // IMAGEPROJECTION_H
