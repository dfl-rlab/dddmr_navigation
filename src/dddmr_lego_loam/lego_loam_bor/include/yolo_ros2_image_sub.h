#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


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

#include "dddmr_trt/yolov8.h"
#include <opencv2/cudaimgproc.hpp>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
 
class YoloRos2ImageSub : public rclcpp::Node
{


  public:

    YoloRos2ImageSub(std::string name);
    
  private:
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_img_;

    void cbImg(const sensor_msgs::msg::Image::SharedPtr img);
    
    cv_bridge::CvImagePtr cv_image_;
    
    std::string trt_model_path_;
    std::shared_ptr<YoloV8> yolov8_;
    
};