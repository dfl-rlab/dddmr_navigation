#include "pc2depthimg_panel/pc2depthimg_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace pc2depthimg_panel
{

Pc2DepthImgPanel::Pc2DepthImgPanel(QWidget *parent) : rviz_common::Panel(parent)
{  
  //@---------- operation button
  QGroupBox *operation_groupBox = new QGroupBox(tr("PC to Depth Image Panel"));
  pause_btn_ = new QPushButton(this);
  pause_btn_->setText("Pause");
  connect(pause_btn_, SIGNAL(clicked()), this, SLOT(pause()));

  resume_btn_ = new QPushButton(this);
  resume_btn_->setText("Resume");
  connect(resume_btn_, SIGNAL(clicked()), this, SLOT(resume()));

  next_frame_btn_ = new QPushButton(this);
  next_frame_btn_->setText("Next Frame");
  connect(next_frame_btn_, SIGNAL(clicked()), this, SLOT(nextFrame()));

  export_selection_btn_ = new QPushButton(this);
  export_selection_btn_->setText("Export Current Selection");
  connect(export_selection_btn_, SIGNAL(clicked()), this, SLOT(exportCurrentSelection()));

  //addWidget(*Widget, row, column, rowspan, colspan)
  QGridLayout *pause_resume_grid_layout = new QGridLayout;
  pause_resume_grid_layout->addWidget(pause_btn_, 0, 0, 1, 1);
  pause_resume_grid_layout->addWidget(resume_btn_, 0, 1, 1, 1);
  pause_resume_grid_layout->addWidget(next_frame_btn_, 0, 2, 1, 1);
  pause_resume_grid_layout->addWidget(export_selection_btn_, 1, 0, 1, 3);
  operation_groupBox->setLayout(pause_resume_grid_layout);
  
  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(operation_groupBox);
  setLayout(layout);
}

void Pc2DepthImgPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pause_resume_pub_ = raw_node->create_publisher<std_msgs::msg::Bool>("lego_loam_bag_pause", 1);
  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  clock_ = raw_node->get_clock();

}

void Pc2DepthImgPanel::pubOperationCommand(std::string m_string){
  std_msgs::msg::String tmp_str;
  tmp_str.data = m_string;
  operation_command_pub_->publish(tmp_str);
}


void Pc2DepthImgPanel::pause(){
  std_msgs::msg::Bool tmp_bool;
  tmp_bool.data = true;
  pause_resume_pub_->publish(tmp_bool);
}

void Pc2DepthImgPanel::resume(){
  std_msgs::msg::Bool tmp_bool;
  tmp_bool.data = false;
  pause_resume_pub_->publish(tmp_bool);
}

void Pc2DepthImgPanel::nextFrame(){
  pubOperationCommand("next_frame");
}

void Pc2DepthImgPanel::exportCurrentSelection(){
  pubOperationCommand("export_current_selection");
}


}  // namespace pc2depthimg_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pc2depthimg_panel::Pc2DepthImgPanel, rviz_common::Panel)
