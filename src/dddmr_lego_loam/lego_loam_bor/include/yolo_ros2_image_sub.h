#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


// Auto-detect cv_bridge version based on file existence
#if __has_include(<cv_bridge/cv_bridge.hpp>)
    // ROS 2 Iron, Jazzy, and newer
    #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
    // ROS 2 Humble and older
    #include <cv_bridge/cv_bridge.h>
#else
    #error "Could not find cv_bridge headers!"
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