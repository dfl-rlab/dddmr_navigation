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
#ifndef PERCEPTION_3D_STATIC_LAYER_H_
#define PERCEPTION_3D_STATIC_LAYER_H_

#include <perception_3d/sensor.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
/*Point cloud library*/
#include <pcl/point_types.h>

/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>

/*For pcl::transformPointCloud, dont use #include <pcl/common/transforms.h>*/
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*Fast triangulation of unordered point clouds*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*pass through*/
#include <pcl/filters/passthrough.h>

namespace perception_3d
{

class StaticLayer: public Sensor{

  public:
    StaticLayer();
    ~StaticLayer();
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
    
    void ptrInitial();
    void radiusSearchConnection();
    void generateStaticGraph();
    
    /*call back of the ground*/
    void cbGround(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    /*call back of the map*/
    void cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /*Subscriber*/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_ground_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_map_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground_;

    bool new_map_, new_ground_;
    rclcpp::CallbackGroup::SharedPtr cbs_group_;
    bool is_local_planner_;

    bool use_adaptive_connection_;
    int adaptive_connection_number_;
    double radius_of_ground_connection_;
    double turning_weight_;
    double intensity_search_radius_;
    double intensity_search_punish_weight_;
    double static_imposing_radius_;
    bool mapping_mode_;
    std::string map_topic_;
    std::string ground_topic_;
    bool is_ground_and_map_being_initialized_once_;
    bool enable_edge_detection_;
    bool generate_static_graph_;
    
};

}//end of name space

#endif