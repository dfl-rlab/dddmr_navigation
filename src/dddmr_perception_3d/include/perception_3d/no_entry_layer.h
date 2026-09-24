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
#ifndef PERCEPTION_3D_NOENTRY_LAYER_H_
#define PERCEPTION_3D_NOENTRY_LAYER_H_

#include <perception_3d/sensor.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

/*boundingbox*/
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/filters/extract_indices.h>

//@Create folder
#include <filesystem>

namespace perception_3d
{

class NoEntryZone{
  public:
    NoEntryZone(const std::string logger_name, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,std::string zone_name, std::string pcd_file_dir, bool is_enabled){
      
      logger_ = m_logger;
      zone_name_ = zone_name;
      pcd_file_dir_= pcd_file_dir;
      is_enabled_ = is_enabled;
      zone_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>());
      zone_intensity_pc_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      if(!std::filesystem::exists(pcd_file_dir_)){
        RCLCPP_WARN(logger_->get_logger().get_child(logger_name), "Directory is invalid: %s", pcd_file_dir_.c_str());
        return;
      }
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_dir_, *zone_pc_) == -1) //* load the file
      {
        RCLCPP_ERROR(logger_->get_logger().get_child(logger_name), "Read record PCD file fail: %s", pcd_file_dir_.c_str());
        return;
      }
      else{
        RCLCPP_INFO(logger_->get_logger().get_child(logger_name), "Load PCD file: %s", pcd_file_dir_.c_str());
      }

      kdtree_zone_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
      kdtree_zone_->setInputCloud(zone_pc_);

      for(auto i=zone_pc_->points.begin(); i!=zone_pc_->points.end();i++){
        pcl::PointXYZI ipt;
        ipt.x = (*i).x;
        ipt.y = (*i).y;
        ipt.z = (*i).z;
        ipt.intensity = 1000000; //@ mm/sec
        zone_intensity_pc_->push_back(ipt);
      }

    }

    std::string zone_name_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr zone_pc_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr zone_intensity_pc_;
    std::string pcd_file_dir_;
    bool is_enabled_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_zone_;
    visualization_msgs::msg::Marker bb_marker_;
};

class NoEntryLayer: public Sensor{

  public:
    NoEntryLayer();
    ~NoEntryLayer();
    virtual void onInitialize();
    virtual void selfClear();
    virtual void selfMark();
    virtual void updateLethalPointCloud();
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservationRobotFrame();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal();
    virtual void resetdGraph();
    virtual double get_dGraphValue(const unsigned int index);
    virtual bool isCurrent();

  private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_entry_pc_zone_pub_;

    double inflation_distance_;

    std::map<std::string, perception_3d::NoEntryZone> no_entry_zones_;

    void parseYAML(std::string path);
    void getBoundingBox(perception_3d::NoEntryZone& a_zone);

    std::recursive_mutex cb_mutex_;
    
    rclcpp::CallbackGroup::SharedPtr cbs_group_;

    std::string no_entry_zone_pcd_file_dir_;

    std::string current_zone_;

    visualization_msgs::msg::MarkerArray zone_marker_array_;
    pcl::PointCloud<pcl::PointXYZI> zone_intensity_pc_;

    std::vector<bool> zone_enabled_status_;
    bool is_zone_initialized_;
};

}//end of name space

#endif
