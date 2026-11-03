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

  clear_selection_btn_ = new QPushButton(this);
  clear_selection_btn_->setText("Clear Selection");
  connect(clear_selection_btn_, SIGNAL(clicked()), this, SLOT(clearSelection()));

  //addWidget(*Widget, row, column, rowspan, colspan)
  QGridLayout *pause_resume_grid_layout = new QGridLayout;
  pause_resume_grid_layout->addWidget(pause_btn_, 0, 0, 1, 1);
  pause_resume_grid_layout->addWidget(resume_btn_, 0, 1, 1, 1);
  pause_resume_grid_layout->addWidget(next_frame_btn_, 0, 2, 1, 1);
  pause_resume_grid_layout->addWidget(export_selection_btn_, 1, 0, 1, 2);
  pause_resume_grid_layout->addWidget(clear_selection_btn_, 1, 2, 1, 1);
  operation_groupBox->setLayout(pause_resume_grid_layout);

  //@---------- Bounding Box Sliders
  QGroupBox *bbox_groupBox = new QGroupBox(tr("Bounding Box Filter (m)"));
  QGridLayout *bbox_grid_layout = new QGridLayout;

  auto createSliderRow = [&](int row, const QString& label_text, QSlider*& slider, QLabel*& val_label,
                             int min_val, int max_val, int init_val, void (Pc2DepthImgPanel::*slot)()) {
    QLabel *name_label = new QLabel(label_text, this);
    slider = new QSlider(Qt::Horizontal, this);
    slider->setFocusPolicy(Qt::StrongFocus);
    slider->setMinimum(min_val);
    slider->setMaximum(max_val);
    slider->setValue(init_val);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setTickInterval(10);
    slider->setSingleStep(1);
    connect(slider, &QSlider::valueChanged, this, slot);
    val_label = new QLabel(this);
    val_label->setText(QString::number(slider->value() * 0.1, 'f', 1));
    bbox_grid_layout->addWidget(name_label, row, 0, 1, 1);
    bbox_grid_layout->addWidget(slider, row, 1, 1, 1);
    bbox_grid_layout->addWidget(val_label, row, 2, 1, 1);
  };

  createSliderRow(0, tr("xmax:"), xmax_slider_, xmax_value_, -500, 500, 500, &Pc2DepthImgPanel::setXmaxValue);
  createSliderRow(1, tr("xmin:"), xmin_slider_, xmin_value_, -500, 500, -500, &Pc2DepthImgPanel::setXminValue);
  createSliderRow(2, tr("ymax:"), ymax_slider_, ymax_value_, -500, 500, 500, &Pc2DepthImgPanel::setYmaxValue);
  createSliderRow(3, tr("ymin:"), ymin_slider_, ymin_value_, -500, 500, -500, &Pc2DepthImgPanel::setYminValue);
  createSliderRow(4, tr("zmax:"), zmax_slider_, zmax_value_, -500, 500, 500, &Pc2DepthImgPanel::setZmaxValue);
  createSliderRow(5, tr("zmin:"), zmin_slider_, zmin_value_, -500, 500, -500, &Pc2DepthImgPanel::setZminValue);

  bbox_groupBox->setLayout(bbox_grid_layout);
  
  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(operation_groupBox);
  layout->addWidget(bbox_groupBox);
  setLayout(layout);
}

void Pc2DepthImgPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pause_resume_pub_ = raw_node->create_publisher<std_msgs::msg::Bool>("lego_loam_bag_pause", 1);
  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  selection_panel_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("point_cloud_selection/panel_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  xmax_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("pc2depthimg/xmax", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  xmin_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("pc2depthimg/xmin", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ymax_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("pc2depthimg/ymax", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ymin_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("pc2depthimg/ymin", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  zmax_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("pc2depthimg/zmax", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  zmin_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("pc2depthimg/zmin", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
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

void Pc2DepthImgPanel::clearSelection(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "clear";
  selection_panel_command_pub_->publish(tmp_str);
}

void Pc2DepthImgPanel::setXmaxValue(){
  xmax_value_->setText(QString::number(xmax_slider_->value() * 0.1, 'f', 1));
  std_msgs::msg::Float32 msg;
  msg.data = xmax_slider_->value() * 0.1f;
  xmax_pub_->publish(msg);
}

void Pc2DepthImgPanel::setXminValue(){
  xmin_value_->setText(QString::number(xmin_slider_->value() * 0.1, 'f', 1));
  std_msgs::msg::Float32 msg;
  msg.data = xmin_slider_->value() * 0.1f;
  xmin_pub_->publish(msg);
}

void Pc2DepthImgPanel::setYmaxValue(){
  ymax_value_->setText(QString::number(ymax_slider_->value() * 0.1, 'f', 1));
  std_msgs::msg::Float32 msg;
  msg.data = ymax_slider_->value() * 0.1f;
  ymax_pub_->publish(msg);
}

void Pc2DepthImgPanel::setYminValue(){
  ymin_value_->setText(QString::number(ymin_slider_->value() * 0.1, 'f', 1));
  std_msgs::msg::Float32 msg;
  msg.data = ymin_slider_->value() * 0.1f;
  ymin_pub_->publish(msg);
}

void Pc2DepthImgPanel::setZmaxValue(){
  zmax_value_->setText(QString::number(zmax_slider_->value() * 0.1, 'f', 1));
  std_msgs::msg::Float32 msg;
  msg.data = zmax_slider_->value() * 0.1f;
  zmax_pub_->publish(msg);
}

void Pc2DepthImgPanel::setZminValue(){
  zmin_value_->setText(QString::number(zmin_slider_->value() * 0.1, 'f', 1));
  std_msgs::msg::Float32 msg;
  msg.data = zmin_slider_->value() * 0.1f;
  zmin_pub_->publish(msg);
}


}  // namespace pc2depthimg_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pc2depthimg_panel::Pc2DepthImgPanel, rviz_common::Panel)
