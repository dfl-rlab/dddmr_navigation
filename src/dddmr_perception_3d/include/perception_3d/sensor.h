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
#ifndef PERCEPTION_3D_SENSOR_H_
#define PERCEPTION_3D_SENSOR_H_

#include "pluginlib/class_list_macros.hpp"

/*Global setting from top level*/
#include <perception_3d/global_utils.h>
#include <perception_3d/shared_data.h>

/*Debug*/
#include <chrono>

// Required OpenMP header
#include <omp.h> 

namespace perception_3d
{

enum PerceptionOpinion {
  PASS,
  PATH_BLOCKED_WAIT,
  PATH_BLOCKED_REPLANNING
};

class Sensor{

  public:
    Sensor();

    void initialize(std::string name,
        const rclcpp::Node::WeakPtr& weak_node,
        const std::shared_ptr<perception_3d::GlobalUtils>& gbl_utils);

    void setSharedData(std::shared_ptr<perception_3d::SharedData> shared_data);

    virtual void selfClear(){}

    virtual void selfMark(){}

    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation() = 0;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal() = 0;

    virtual void resetdGraph(){}

    virtual double get_dGraphValue(const unsigned int index) = 0;

    virtual bool isCurrent() = 0;
    
    virtual void updateLethalPointCloud(){}

    perception_3d::PerceptionOpinion getOpinion(){return opinion_;}

    std::shared_ptr<GlobalUtils> getGlobalUtils(){return gbl_utils_;}
    
    std::string getName(){return name_;}
    
    unsigned long get_dGraphSize();

  protected:

    virtual void onInitialize() {}

    rclcpp::Node::SharedPtr node_;

    bool current_;
    double expected_sensor_time_;
    std::string name_;
    std::shared_ptr<perception_3d::GlobalUtils> gbl_utils_;
    std::shared_ptr<perception_3d::SharedData> shared_data_;
   
    //@ For TF affine3d
    geometry_msgs::msg::TransformStamped trans_b2s_, trans_gbl2b_;

    //@ Graph, basically like the costmap_2d char
    DynamicGraph dGraph_;

    //@ current observation for local planner
    pcl::PointCloud<pcl::PointXYZI>::Ptr sensor_current_observation_;

    //@ opinion of plugin, default is pass
    PerceptionOpinion opinion_;
    
    //@ lethal point cloud, that will be merged together for line-of-sight check
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_lethal_;

};


}//end of name space
#endif