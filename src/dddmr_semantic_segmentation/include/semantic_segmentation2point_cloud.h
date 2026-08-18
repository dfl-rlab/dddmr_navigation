#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

//auto detect cv bridge version to prepare for jazzy
#ifdef ROS_JAZZY
    // ROS 2 Iron, Jazzy, and newer use the .hpp extension
    #include <cv_bridge/cv_bridge.hpp>
    #include <image_geometry/pinhole_camera_model.hpp>
#else
    // ROS 2 Humble and older use the .h extension
    #include <cv_bridge/cv_bridge.h>
    #include <image_geometry/pinhole_camera_model.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// sync mask and depth
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
 
class SemanticSegmentation2PointCloud : public rclcpp::Node
{

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MaskDepthSyncPolicy;

  public:

    SemanticSegmentation2PointCloud(std::string name);
    
  private:
    //@Prepare all subscribers from LegoLoam
    message_filters::Subscriber<sensor_msgs::msg::Image> mask_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_img_sub_;
    std::shared_ptr<message_filters::Synchronizer<MaskDepthSyncPolicy>> syncApproximate_;
    
    void cbCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void cbMaskDepthImg(const sensor_msgs::msg::Image::SharedPtr mask,
                    const sensor_msgs::msg::Image::SharedPtr depth);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;
    rclcpp::CallbackGroup::SharedPtr cbs_group_, cbs_group2_;
    bool has_info_;
    sensor_msgs::msg::CameraInfo camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    cv_bridge::CvImagePtr cv_mask_;
    cv_bridge::CvImagePtr cv_image_;
    rclcpp::Time last_pub_time_;
    double max_distance_;
    double frequency_;
    double leaf_size_;
    int sample_step_;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter_intensity_;
    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter_rgb_;
    int width_start_;
    int width_end_;
    int height_start_;
    int height_end_;
    std::map<char, pcl::PointCloud<pcl::PointXYZI>> xyzi_map_;
    std::map<unsigned int, pcl::PointCloud<pcl::PointXYZRGB>> xyzrgb_map_;
    std::vector<long int> exclude_class_vector_;
};
