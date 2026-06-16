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

#include "dddmr_rviz_default_plugins/tools/point_cloud_delete/point_cloud_delete_tool.hpp"
#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreMovableObject.h>
#include <OgreRectangle2D.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <QKeyEvent>  // NOLINT cpplint cannot handle include order

#include "dddmr_rviz_default_plugins/tools/move/move_tool.hpp"

#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/load_resource.hpp"

namespace dddmr_rviz_default_plugins
{
namespace tools
{


PointCloudDeleteTool::PointCloudDeleteTool()
: Tool(),
  move_tool_(new MoveTool()),
  selecting_(false),
  sel_start_x_(0),
  sel_start_y_(0),
  moving_(false)
{
  shortcut_key_ = 's';
  access_all_keys_ = true;
}

PointCloudDeleteTool::~PointCloudDeleteTool()
{
  delete move_tool_;
}

void PointCloudDeleteTool::onInitialize()
{
  move_tool_->initialize(context_);
  //@ ros stuff can only be placed here instead of in constructor
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  selected_bb_pub_ = raw_node->create_publisher<visualization_msgs::msg::MarkerArray>("point_cloud_delete/selected_bb", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  selected_points_pub_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_delete/selected_points", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  selected_bb_vertex_pub_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_delete/selected_bb_vertex", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_map_editor_panel_command_ = raw_node->create_subscription<std_msgs::msg::String>("point_cloud_delete/panel_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&PointCloudDeleteTool::panelCommandCb, this, std::placeholders::_1));
}

void PointCloudDeleteTool::activate()
{
  setStatus("Click and drag to select objects on the screen.");
  context_->getSelectionManager()->setTextureSize(512);
  selecting_ = false;
  moving_ = false;
//  context_->getSelectionManager()->enableInteraction(true);
}

void PointCloudDeleteTool::deactivate()
{
  context_->getSelectionManager()->removeHighlight();
}

void PointCloudDeleteTool::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  auto sel_manager = context_->getSelectionManager();

  if (!selecting_) {
    sel_manager->removeHighlight();
  }
}

void PointCloudDeleteTool::panelCommandCb(const std_msgs::msg::String::SharedPtr msg){
  if(msg->data=="clear"){
    if(operation_steps_.size()>0){
      RCLCPP_INFO(rclcpp::get_logger("PointCloudDeleteTool"), "Clear selected point cloud.");
      operation_steps_.clear();
      publishResultingPCL();
    }
  }
  else if(msg->data=="last_step"){
    if(operation_steps_.size()>0){
      operation_steps_.pop_back();
      publishResultingPCL();
    }
  }
}

int PointCloudDeleteTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  auto sel_manager = context_->getSelectionManager();

  int flags = 0;

  if (event.alt()) {
    moving_ = true;
    selecting_ = false;
  } else {
    moving_ = false;

    if (event.leftDown()) {
      selecting_ = true;

      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
    }
  }

  if (selecting_) {
    sel_manager->highlight(
      event.panel->getRenderWindow(),
      sel_start_x_,
      sel_start_y_,
      event.x,
      event.y);

    if (event.leftUp()) {
      rviz_common::interaction::SelectionManager::SelectType type =
        rviz_common::interaction::SelectionManager::Replace;

      rviz_common::interaction::M_Picked selection;

      if (event.shift()) {
        type = rviz_common::interaction::SelectionManager::Add;
      } else if (event.control()) {
        type = rviz_common::interaction::SelectionManager::Remove;
      }

      sel_manager->select(
        event.panel->getRenderWindow(),
        sel_start_x_,
        sel_start_y_,
        event.x,
        event.y,
        type);

      selecting_ = false;
      publishSelected(event);
    }

    flags |= Render;
  } else if (moving_) {
    sel_manager->removeHighlight();

    flags = move_tool_->processMouseEvent(event);

    if (event.type == QEvent::MouseButtonRelease) {
      moving_ = false;
    }
  } else {
    sel_manager->highlight(
      event.panel->getRenderWindow(),
      event.x,
      event.y,
      event.x,
      event.y);
  }

  return flags;
}

void PointCloudDeleteTool::publishSelected(rviz_common::ViewportMouseEvent & event){
  auto sel_manager = context_->getSelectionManager();
  rviz_common::interaction::M_Picked selection = sel_manager->getSelection();
  rviz_common::properties::PropertyTreeModel* model = sel_manager->getPropertyModel();

  pcl::PointCloud<pcl::PointXYZ> selected_points_pcl;
  pcl::PointXYZ center_mass_point;

  int num_points = model->rowCount();

  for( int i = 0; i < num_points; i++ )
  {
    QModelIndex child_index = model->index( i, 0 );
    rviz_common::properties::Property* child = model->getProp( child_index );
    rviz_common::properties::VectorProperty* subchild = (rviz_common::properties::VectorProperty*) child->childAt( 0 );
    Ogre::Vector3 vec = subchild->getVector();
    RCLCPP_DEBUG(rclcpp::get_logger("PointCloudDeleteTool"), "%s, %.2f, %.2f, %.2f", child->getNameStd().c_str(), vec.x, vec.y, vec.z);
    //Marker pg_0_node/88, 30.49, -66.05, 1.24
    //Marker pg_0_edge_155/90, 0.00, 0.00, 0.00
    //Point 141488 [cloud 0x94354302778400], 41.52, -17.00, -1.88
    if (child->getNameStd().find("cloud") != std::string::npos) {
      RCLCPP_DEBUG(rclcpp::get_logger("PointCloudDeleteTool"), "%s, %.2f, %.2f, %.2f", child->getNameStd().c_str(), vec.x, vec.y, vec.z);
      pcl::PointXYZ pt;
      pt.x = vec.x;
      pt.y = vec.y;
      pt.z = vec.z;
      selected_points_pcl.push_back(pt);
      center_mass_point.x += pt.x;
      center_mass_point.y += pt.y;
      center_mass_point.z += pt.z;
    }
  }
  
  if(selected_points_pcl.points.size()<2){
    return;
  }
  
  operation_steps_.push_back(selected_points_pcl);
  publishResultingPCL();

  /*

  RCLCPP_INFO(rclcpp::get_logger("PointCloudDeleteTool"), "Selected points size: %lu", accumulated_selected_pc_.points.size());
  if(accumulated_selected_pc_.points.size()<3){
    RCLCPP_INFO(rclcpp::get_logger("PointCloudDeleteTool"), "Require at least 4 points");
    return;
  }

  //accumulated_selected_pc_ = selected_points_pcl;
  // Generate an oriented bounding box around the selected points in RVIZ
  // Compute principal direction
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(accumulated_selected_pc_, centroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(accumulated_selected_pc_, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  // Move the points to the that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ> cPoints;
  pcl::transformPointCloud(accumulated_selected_pc_, cPoints, p2w);

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cPoints, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
  // Final transform and bounding box size
  const Eigen::Quaternionf qfinal(eigDx);
  const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
  double bb_size_x = max_pt.x - min_pt.x;
  double bb_size_y = max_pt.y - min_pt.y;
  double bb_size_z = max_pt.z - min_pt.z;

  double z_patch = 0.4;
  bb_size_z+=z_patch;

  tf2::Transform tf2_bb_center; //generate center pose
  tf2_bb_center.setOrigin(tf2::Vector3(tfinal.x(), tfinal.y(), tfinal.z()));
  tf2_bb_center.setRotation(tf2::Quaternion(qfinal.x(), qfinal.y(), qfinal.z(), qfinal.w()));

  tf2::Transform tf2_bb_pxpypz, tf2_bb_g_pxpypz; //center2vertice
  tf2_bb_pxpypz.setOrigin(tf2::Vector3(bb_size_x/2, bb_size_y/2, bb_size_z/2));
  tf2_bb_pxpypz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_pxpypz.mult(tf2_bb_center, tf2_bb_pxpypz);

  tf2::Transform tf2_bb_pxpynz, tf2_bb_g_pxpynz; //center2vertice
  tf2_bb_pxpynz.setOrigin(tf2::Vector3(bb_size_x/2, bb_size_y/2, -bb_size_z/2));
  tf2_bb_pxpynz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_pxpynz.mult(tf2_bb_center, tf2_bb_pxpynz);

  tf2::Transform tf2_bb_pxnypz, tf2_bb_g_pxnypz; //center2vertice
  tf2_bb_pxnypz.setOrigin(tf2::Vector3(bb_size_x/2, -bb_size_y/2, bb_size_z/2));
  tf2_bb_pxnypz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_pxnypz.mult(tf2_bb_center, tf2_bb_pxnypz);

  tf2::Transform tf2_bb_pxnynz, tf2_bb_g_pxnynz; //center2vertice
  tf2_bb_pxnynz.setOrigin(tf2::Vector3(bb_size_x/2, -bb_size_y/2, -bb_size_z/2));
  tf2_bb_pxnynz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_pxnynz.mult(tf2_bb_center, tf2_bb_pxnynz);

  tf2::Transform tf2_bb_nxnynz, tf2_bb_g_nxnynz; //center2vertice
  tf2_bb_nxnynz.setOrigin(tf2::Vector3(-bb_size_x/2, -bb_size_y/2, -bb_size_z/2));
  tf2_bb_nxnynz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_nxnynz.mult(tf2_bb_center, tf2_bb_nxnynz);

  tf2::Transform tf2_bb_nxpypz, tf2_bb_g_nxpypz; //center2vertice
  tf2_bb_nxpypz.setOrigin(tf2::Vector3(-bb_size_x/2, bb_size_y/2, bb_size_z/2));
  tf2_bb_nxpypz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_nxpypz.mult(tf2_bb_center, tf2_bb_nxpypz);

  tf2::Transform tf2_bb_nxpynz, tf2_bb_g_nxpynz; //center2vertice
  tf2_bb_nxpynz.setOrigin(tf2::Vector3(-bb_size_x/2, bb_size_y/2, -bb_size_z/2));
  tf2_bb_nxpynz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_nxpynz.mult(tf2_bb_center, tf2_bb_nxpynz);

  tf2::Transform tf2_bb_nxnypz, tf2_bb_g_nxnypz; //center2vertice
  tf2_bb_nxnypz.setOrigin(tf2::Vector3(-bb_size_x/2, -bb_size_y/2, bb_size_z/2));
  tf2_bb_nxnypz.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  tf2_bb_g_nxnypz.mult(tf2_bb_center, tf2_bb_nxnypz);

  // marker array of edge and bb
  visualization_msgs::msg::MarkerArray markerArray;
  // Publish the bounding box as a rectangular marker
  visualization_msgs::msg::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
  marker.ns = "selected_bb";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = tfinal.x();
  marker.pose.position.y = tfinal.y();
  marker.pose.position.z = tfinal.z();
  marker.pose.orientation.x = qfinal.x();
  marker.pose.orientation.y = qfinal.y();
  marker.pose.orientation.z = qfinal.z();
  marker.pose.orientation.w = qfinal.w();
  marker.scale.x = bb_size_x;
  marker.scale.y = bb_size_y;
  marker.scale.z = bb_size_z;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;
  markerArray.markers.push_back(marker);
  
  //connect edge
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = context_->getFixedFrame().toStdString().c_str();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.pose.orientation.w = 1.0;
  markerEdge.ns = "selected_bb_edge";
  markerEdge.id = 0;
  markerEdge.scale.x = 0.05;
  markerEdge.color.r = 0.1;
  markerEdge.color.g = 0.0;
  markerEdge.color.b = 0.7;
  markerEdge.color.a = 0.7;
  addMarkerEdge(markerEdge, tf2_bb_g_pxpypz, tf2_bb_g_nxpypz);
  addMarkerEdge(markerEdge, tf2_bb_g_pxpypz, tf2_bb_g_pxnypz);
  addMarkerEdge(markerEdge, tf2_bb_g_pxpypz, tf2_bb_g_pxpynz);
  
  addMarkerEdge(markerEdge, tf2_bb_g_pxnypz, tf2_bb_g_nxnypz);
  addMarkerEdge(markerEdge, tf2_bb_g_pxnypz, tf2_bb_g_pxnynz);
  addMarkerEdge(markerEdge, tf2_bb_g_pxpynz, tf2_bb_g_pxnynz);

  addMarkerEdge(markerEdge, tf2_bb_g_nxpypz, tf2_bb_g_nxnypz);
  addMarkerEdge(markerEdge, tf2_bb_g_nxpypz, tf2_bb_g_nxpynz);
  addMarkerEdge(markerEdge, tf2_bb_g_nxpynz, tf2_bb_g_pxpynz);

  addMarkerEdge(markerEdge, tf2_bb_g_nxnypz, tf2_bb_g_nxnynz);
  addMarkerEdge(markerEdge, tf2_bb_g_nxnynz, tf2_bb_g_pxnynz);
  addMarkerEdge(markerEdge, tf2_bb_g_nxnynz, tf2_bb_g_nxpynz);
  
  markerArray.markers.push_back(markerEdge);

  selected_bb_pub_->publish(markerArray);
  
  //--------------------------------------------------------------------------------------
  //@ Check --Start to parse cuboid-- section in dd_simple_trajectory_generator_theory.cpp
  
  pcl::PointCloud<pcl::PointXYZI> cuboid_vertex;

  pcl::PointXYZI pt_nxpynz;
  pt_nxpynz.x = tf2_bb_g_nxpynz.getOrigin().x();
  pt_nxpynz.y = tf2_bb_g_nxpynz.getOrigin().y();
  pt_nxpynz.z = tf2_bb_g_nxpynz.getOrigin().z();
  pt_nxpynz.intensity = 1000;
  cuboid_vertex.push_back(pt_nxpynz);
  
  pcl::PointXYZI pt_nxnynz;
  pt_nxnynz.x = tf2_bb_g_nxnynz.getOrigin().x();
  pt_nxnynz.y = tf2_bb_g_nxnynz.getOrigin().y();
  pt_nxnynz.z = tf2_bb_g_nxnynz.getOrigin().z();
  pt_nxnynz.intensity = 2000;
  cuboid_vertex.push_back(pt_nxnynz);

  pcl::PointXYZI pt_nxpypz;
  pt_nxpypz.x = tf2_bb_g_nxpypz.getOrigin().x();
  pt_nxpypz.y = tf2_bb_g_nxpypz.getOrigin().y();
  pt_nxpypz.z = tf2_bb_g_nxpypz.getOrigin().z();
  pt_nxpypz.intensity = 3000;
  cuboid_vertex.push_back(pt_nxpypz);

  pcl::PointXYZI pt_pxpynz;
  pt_pxpynz.x = tf2_bb_g_pxpynz.getOrigin().x();
  pt_pxpynz.y = tf2_bb_g_pxpynz.getOrigin().y();
  pt_pxpynz.z = tf2_bb_g_pxpynz.getOrigin().z();
  pt_pxpynz.intensity = 4000;
  cuboid_vertex.push_back(pt_pxpynz);

  //params_->cuboid.push_back(pt_blb);// back left bottom
  //params_->cuboid.push_back(pt_brb);// back right bottom
  //params_->cuboid.push_back(pt_blt);// back left top
  //params_->cuboid.push_back(pt_flb);// front left bottom

  //@ push the point by following sequence, because when doing the point in cuboid test we leverage blb/brb/blt/flb
  //
  //          ----------
  //         /|        /|
  //        / |       / |
  //  2nxpypz---------  |
  //       | /3pxpynz| /
  //       |/        |/
  // 0nxpynz---------1nxnynz

  sensor_msgs::msg::PointCloud2 cloudMsgTemp;
  pcl::toROSMsg(cuboid_vertex, cloudMsgTemp);
  cloudMsgTemp.header.frame_id = context_->getFixedFrame().toStdString();;
  selected_bb_vertex_pub_->publish(cloudMsgTemp);

  sel_manager->select(
    event.panel->getRenderWindow(),
    sel_start_x_,
    sel_start_y_,
    event.x,
    event.y,
    rviz_common::interaction::SelectionManager::Remove);
  */
}

void PointCloudDeleteTool::addMarkerEdge(visualization_msgs::msg::Marker& marker, tf2::Transform pa, tf2::Transform pb){
  
  geometry_msgs::msg::Point pta;
  pta.x = pa.getOrigin().x();
  pta.y = pa.getOrigin().y();
  pta.z = pa.getOrigin().z();
  marker.points.push_back(pta);
  RCLCPP_DEBUG(rclcpp::get_logger("PointCloudDeleteTool"), "%.2f, %.2f, %.2f", pta.x, pta.y, pta.z);
  geometry_msgs::msg::Point ptb;
  ptb.x = pb.getOrigin().x();
  ptb.y = pb.getOrigin().y();
  ptb.z = pb.getOrigin().z();
  marker.points.push_back(ptb);

}

void PointCloudDeleteTool::publishResultingPCL(){
  
  accumulated_selected_pc_.clear();
  for(auto it=operation_steps_.begin(); it!=operation_steps_.end();it++){
    accumulated_selected_pc_ += (*it);
  }

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(accumulated_selected_pc_, output);
  output.header.frame_id = context_->getFixedFrame().toStdString();
  selected_points_pub_->publish(output);
  RCLCPP_INFO(rclcpp::get_logger("PointCloudDeleteTool"), "Publish new cloud with size: %lu", accumulated_selected_pc_.points.size());
}

int PointCloudDeleteTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
{
  (void) panel;
  auto sel_manager = context_->getSelectionManager();

  if (event->key() == Qt::Key_F) {
    sel_manager->focusOnSelection();
  }

  if (event->key() == Qt::Key_Z) {
    if(operation_steps_.size()>0){
      operation_steps_.pop_back();
      publishResultingPCL();
    }
  }

  if (event->key() == Qt::Key_C) {
    if(operation_steps_.size()>0){
      operation_steps_.clear();
      publishResultingPCL();
    }
  }

  return Render;
}

}  // namespace tools
}  // namespace dddmr_rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(dddmr_rviz_default_plugins::tools::PointCloudDeleteTool, rviz_common::Tool)
