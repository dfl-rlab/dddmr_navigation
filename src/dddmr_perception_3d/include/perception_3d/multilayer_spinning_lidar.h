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
#ifndef PERCEPTION_3D_MULTILAYER_SPINNING_LIDAR_LAYER_H_
#define PERCEPTION_3D_MULTILAYER_SPINNING_LIDAR_LAYER_H_

#include <perception_3d/sensor.h>

#include <perception_3d/cluster_marking.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
/*Point cloud library*/
#include <pcl/point_types.h>

/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

/*pass through*/
#include <pcl/filters/passthrough.h>

/*normal estimation*/
#include <pcl/features/normal_3d.h>

/*voxel*/
#include <pcl/filters/voxel_grid.h>

/*For normal/casting markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*For map*/
#include <map>
#include <set>

/*For sqrt*/
#include <math.h> 

/*This is for euclidean distance segmentation*/
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

/*kd tree for casting*/
#include <pcl/kdtree/kdtree_flann.h>

/*Open MP*/
#include <omp.h>

/*Project to ground*/
#include <pcl/filters/project_inliers.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

/*tf2 to ros msg/vice versa*/
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
/*For shortest angle*/
#include <angles/angles.h>

namespace perception_3d
{

class MultiLayerSpinningLidar: public Sensor{

  public:
    MultiLayerSpinningLidar();
    ~MultiLayerSpinningLidar();

    virtual void onInitialize();
    virtual void selfClear();
    virtual void selfMark();
    virtual void updateLethalPointCloud();
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal();
    virtual void resetdGraph();
    virtual double get_dGraphValue(const unsigned int index);
    virtual bool isCurrent();

  private:
    
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
    rclcpp::CallbackGroup::SharedPtr marking_pub_cb_group_;
    rclcpp::TimerBase::SharedPtr marking_pub_timer_;

    void ptrInitial();

    /*call back of the sensor*/
    void cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /*call back of the map*/
    void cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /*In observation area testing*/
    bool isinLidarObservation(pcl::PointXYZ& pc);
    
    /*Get pc on the line equation of two points*/
    void getCastingPointCloud(pcl::PointXYZ& cluster_center, pcl::PointCloud<pcl::PointXYZI>& pc_for_check);

    /*Thread for gbl marking pub*/
    void pubUpdateLoop();
    
    /*For casting visualization*/
    void addCastingMarker(const pcl::PointXYZ& pt, size_t id, visualization_msgs::msg::MarkerArray& markerArray);

    /*pcl msg in cb*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg_gbl_;

    /*For first TF*/
    bool get_first_tf_;
    geometry_msgs::msg::TransformStamped trans_b2s_, trans_gbl2b_, trans_gbl2s_;
    Eigen::Affine3d trans_gbl2s_af3_;

    /*Voxel structure*/
    std::shared_ptr<Marking> pct_marking_;


    /*plugin parameter*/
    std::string topic_;
    double vertical_FOV_top_,vertical_FOV_bottom_;
    double scan_effective_positive_start_, scan_effective_positive_end_;
    double scan_effective_negative_start_, scan_effective_negative_end_;
    double resolution_, height_resolution_;
    double marking_height_;
    double marking_minimum_height_;
    double perception_window_size_; 
    double segmentation_ignore_ratio_;
    bool pub_gbl_marking_for_visualization_;
    int euclidean_cluster_extraction_min_cluster_size_;
    int euclidean_cluster_minimum_accepted_size_;
    double euclidean_cluster_extraction_tolerance_;
    int stitcher_num_;
    double pub_gbl_marking_frequency_;
    
    /*pub and sub*/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_observation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_segmentation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_projected_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_window_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cleared_window_marking_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_casting_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gbl_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lethal_;
    
    /**/
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current_window_;

    /*For local planner switch*/
    bool is_local_planner_;

    /*For isCurrent feature. Used to protect sensor broken*/
    rclcpp::Time last_observation_time_;

    //@ list of pointcloud sticher for non-repetitive scan lidar
    std::list<pcl::PointCloud<pcl::PointXYZ>> pcl_stitcher_;

    std_msgs::msg::Header last_sensor_receiving_time_;
};

}//end of name space
#endif