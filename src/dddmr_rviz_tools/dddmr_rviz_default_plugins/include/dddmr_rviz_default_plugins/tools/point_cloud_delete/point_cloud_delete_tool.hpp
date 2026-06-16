/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DDDMR_RVIZ_DEFAULT_PLUGINS__TOOLS__PCDT__PCDT_HPP_
#define DDDMR_RVIZ_DEFAULT_PLUGINS__TOOLS__PCDT__PCDT_HPP_

#include <vector>

#include "rviz_common/tool.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "dddmr_rviz_default_plugins/visibility_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include <sstream>

// PCL
#include "pcl/common/transforms.h"
#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

// ROS msg
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// tf2
#include "tf2/LinearMath/Transform.h"


namespace Ogre
{
class Viewport;
}

namespace dddmr_rviz_default_plugins
{
namespace tools
{

class MoveTool;

class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudDeleteTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  PointCloudDeleteTool();
  virtual ~PointCloudDeleteTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event);
  virtual int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel);

  virtual void update(float wall_dt, float ros_dt);

private:

  MoveTool * move_tool_;

  bool selecting_;
  int sel_start_x_;
  int sel_start_y_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_bb_vertex_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr selected_bb_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_map_editor_panel_command_;

  rviz_common::interaction::M_Picked highlight_;

  bool moving_;

  void publishSelected(rviz_common::ViewportMouseEvent & event);
  void addMarkerEdge(visualization_msgs::msg::Marker& marker, tf2::Transform pa, tf2::Transform pb);

  pcl::PointCloud<pcl::PointXYZ> accumulated_selected_pc_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> operation_steps_;

  void publishResultingPCL();
  void panelCommandCb(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace tools
}  // namespace point_cloud_bounding_box

#endif  // DDDMR_RVIZ_DEFAULT_PLUGINS__TOOLS__PCDT__PCDT_HPP_
