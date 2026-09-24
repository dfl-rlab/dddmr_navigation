/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef PERCEPTION_3D_DEPTH_CAMERA_LAYER_H_
#define PERCEPTION_3D_DEPTH_CAMERA_LAYER_H_

#include <perception_3d/sensor.h>
#include <perception_3d/cluster_kdtree_marking.h>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>
#include <perception_3d/depth_camera/frustum_utils.h>

namespace perception_3d
{

class DepthCameraLayer: public Sensor{

  public:

    DepthCameraLayer();
    ~DepthCameraLayer();

    void onInitialize();
    void selfClear();
    void selfMark();
    void updateLethalPointCloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getObservationRobotFrame();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal();
    void resetdGraph();
    double get_dGraphValue(const unsigned int index);
    bool isCurrent();

  private:
    
    std::shared_ptr<perception_3d::KDTreeMarking> pct_marking_;
    std::shared_ptr<perception_3d::FrustumUtils> frustum_utils_;

    rclcpp::Clock::SharedPtr clock_;
    
    rclcpp::CallbackGroup::SharedPtr marking_pub_cb_group_;

    rclcpp::TimerBase::SharedPtr marking_pub_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_observation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_segmentation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_projected_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_window_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cleared_window_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_marked_voxel_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_casting_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gbl_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lethal_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frustum_;

    void cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                    const std::shared_ptr<perception_3d::DepthCameraObservationBuffer>& buffer);
    void aggregatePointCloudFromObservations(const pcl::PointCloud<pcl::PointXYZI>::Ptr& resulting_pcl)  ;
    void aggregatePointCloudFromObservationsRobotFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& resulting_pcl)  ;
         
    //@ For casting visualization
    void addCastingMarker(const pcl::PointXYZI& pt, size_t id, visualization_msgs::msg::MarkerArray& markerArray);
    
    void updateDGraphInWindow();

    void pubUpdateLoop();

    void transformToPlaneEquation(
      const geometry_msgs::msg::TransformStamped& transform,
      pcl::ModelCoefficients::Ptr& coefficients,
      const Eigen::Vector3d& local_normal);
      
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_pc_map_; 
    std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>> observation_buffers_;
    
    bool pub_gbl_marking_for_visualization_;
    double pub_gbl_marking_frequency_;
    bool is_local_planner_;
    double resolution_, height_resolution_;
    double segmentation_ignore_ratio_;
    double perception_window_size_; 
    double marking_height_;
    double euclidean_cluster_extraction_tolerance_;
    int euclidean_cluster_extraction_min_cluster_size_;
    int euclidean_cluster_minimum_accepted_size_;
    geometry_msgs::msg::TransformStamped trans_gbl2b_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current_window_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg_gbl_;
    pcl::PointCloud<pcl::PointXYZI> projected_cloud_clusters_;
};

}//end of name space
#endif