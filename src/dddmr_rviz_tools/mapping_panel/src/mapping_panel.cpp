#include "mapping_panel/mapping_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace mapping_panel
{

MappingPanel::MappingPanel(QWidget *parent) : rviz_common::Panel(parent)
{  
  //@---------- operation button
  QGroupBox *operation_groupBox = new QGroupBox(tr("Mapping Panel"));
  pause_btn_ = new QPushButton(this);
  pause_btn_->setText("Pause");
  connect(pause_btn_, SIGNAL(clicked()), this, SLOT(pause()));

  resume_btn_ = new QPushButton(this);
  resume_btn_->setText("Resume");
  connect(resume_btn_, SIGNAL(clicked()), this, SLOT(resume()));

  //addWidget(*Widget, row, column, rowspan, colspan)
  QGridLayout *pause_resume_grid_layout = new QGridLayout;
  pause_resume_grid_layout->addWidget(pause_btn_, 0, 0, 1, 1);
  pause_resume_grid_layout->addWidget(resume_btn_, 0, 1, 1, 1);
  operation_groupBox->setLayout(pause_resume_grid_layout);
  
  //ICP Score slider
  QGroupBox *icp_scoreBox = new QGroupBox(tr("ICP Score"));
  icp_score_slider_ = new QSlider(Qt::Horizontal,0);
  icp_score_slider_->setFocusPolicy(Qt::StrongFocus);
  icp_score_slider_->setMinimum(1.0);
  icp_score_slider_->setMaximum(100.0);
  icp_score_slider_->setValue(5.0);
  icp_score_slider_->setTickPosition(QSlider::TicksBelow);
  icp_score_slider_->setTickInterval(1.0);
  icp_score_slider_->setSingleStep(1.0);  
  connect(icp_score_slider_, &QSlider::valueChanged, this, &MappingPanel::setICPScoreValue);
  icp_score_value_ = new QLabel(this);
  icp_score_value_->setText(std::to_string(icp_score_slider_->value()*0.1).c_str());
  QVBoxLayout *icp_score_vbox = new QVBoxLayout;
  icp_score_vbox->addWidget(icp_score_slider_);
  icp_score_vbox->addWidget(icp_score_value_);
  icp_score_vbox->addStretch(1);
  icp_scoreBox->setLayout(icp_score_vbox);

  //History Keyframe Search Radius slider
  QGroupBox *history_keyframe_search_radiusBox = new QGroupBox(tr("History Keyframe Search Radius"));
  history_keyframe_search_radius_slider_ = new QSlider(Qt::Horizontal,0);
  history_keyframe_search_radius_slider_->setFocusPolicy(Qt::StrongFocus);
  history_keyframe_search_radius_slider_->setMinimum(1.0);
  history_keyframe_search_radius_slider_->setMaximum(100.0);
  history_keyframe_search_radius_slider_->setValue(10.0);
  history_keyframe_search_radius_slider_->setTickPosition(QSlider::TicksBelow);
  history_keyframe_search_radius_slider_->setTickInterval(1.0);
  history_keyframe_search_radius_slider_->setSingleStep(1.0);  
  connect(history_keyframe_search_radius_slider_, &QSlider::valueChanged, this, &MappingPanel::setHistoryKeyframeSearchRadiusValue);
  history_keyframe_search_radius_value_ = new QLabel(this);
  history_keyframe_search_radius_value_->setText(std::to_string(history_keyframe_search_radius_slider_->value()).c_str());
  QVBoxLayout *history_keyframe_search_radius_vbox = new QVBoxLayout;
  history_keyframe_search_radius_vbox->addWidget(history_keyframe_search_radius_slider_);
  history_keyframe_search_radius_vbox->addWidget(history_keyframe_search_radius_value_);
  history_keyframe_search_radius_vbox->addStretch(1);
  history_keyframe_search_radiusBox->setLayout(history_keyframe_search_radius_vbox);

  //Skip Frame slider
  QGroupBox *skip_frameBox = new QGroupBox(tr("Skip Frame"));
  skip_frame_slider_ = new QSlider(Qt::Horizontal,0);
  skip_frame_slider_->setFocusPolicy(Qt::StrongFocus);
  skip_frame_slider_->setMinimum(1.0);
  skip_frame_slider_->setMaximum(100.0);
  skip_frame_slider_->setValue(10.0);
  skip_frame_slider_->setTickPosition(QSlider::TicksBelow);
  skip_frame_slider_->setTickInterval(1.0);
  skip_frame_slider_->setSingleStep(1.0);  
  connect(skip_frame_slider_, &QSlider::valueChanged, this, &MappingPanel::setSkipFrameValue);
  skip_frame_value_ = new QLabel(this);
  skip_frame_value_->setText(std::to_string(skip_frame_slider_->value()).c_str());
  QVBoxLayout *skip_frame_vbox = new QVBoxLayout;
  skip_frame_vbox->addWidget(skip_frame_slider_);
  skip_frame_vbox->addWidget(skip_frame_value_);
  skip_frame_vbox->addStretch(1);
  skip_frameBox->setLayout(skip_frame_vbox);

  //@---------- Operation group box
  QGroupBox *icp_groupBox = new QGroupBox(tr("ICP tools"));

  do_icp_ = new QPushButton(tr("ICP"));
  connect(do_icp_, SIGNAL(clicked()), this, SLOT(icp_clicked ()));
  accept_result_ = new QPushButton(tr("Accept ICP"));
  connect(accept_result_, SIGNAL(clicked()), this, SLOT(accept_clicked ()));
  set_pose_ = new QPushButton(tr("Set Pose"));
  connect(set_pose_, SIGNAL(clicked()), this, SLOT(set_pose_clicked ()));

  px_increase_ = new QPushButton(tr("px+"));
  px_decrease_ = new QPushButton(tr("px-"));
  py_increase_ = new QPushButton(tr("py+"));
  py_decrease_ = new QPushButton(tr("py-"));
  pz_increase_ = new QPushButton(tr("pz+"));
  pz_decrease_ = new QPushButton(tr("pz-"));
  connect(px_increase_, SIGNAL(clicked()), this, SLOT(pxIncrease ()));
  connect(px_decrease_, SIGNAL(clicked()), this, SLOT(pxDecrease ()));
  connect(py_increase_, SIGNAL(clicked()), this, SLOT(pyIncrease ()));
  connect(py_decrease_, SIGNAL(clicked()), this, SLOT(pyDecrease ()));
  connect(pz_increase_, SIGNAL(clicked()), this, SLOT(pzIncrease ()));
  connect(pz_decrease_, SIGNAL(clicked()), this, SLOT(pzDecrease ()));

  roll_increase_ = new QPushButton(tr("roll+"));
  roll_decrease_ = new QPushButton(tr("roll-"));
  pitch_increase_ = new QPushButton(tr("pitch+"));
  pitch_decrease_ = new QPushButton(tr("pitch-"));
  yaw_increase_ = new QPushButton(tr("yaw+"));
  yaw_decrease_ = new QPushButton(tr("yaw-"));
  connect(roll_increase_, SIGNAL(clicked()), this, SLOT(rollIncrease ()));
  connect(roll_decrease_, SIGNAL(clicked()), this, SLOT(rollDecrease ()));
  connect(pitch_increase_, SIGNAL(clicked()), this, SLOT(pitchIncrease ()));
  connect(pitch_decrease_, SIGNAL(clicked()), this, SLOT(pitchDecrease ()));
  connect(yaw_increase_, SIGNAL(clicked()), this, SLOT(yawIncrease ()));
  connect(yaw_decrease_, SIGNAL(clicked()), this, SLOT(yawDecrease ()));
  
  QGridLayout *icp_grid_layout = new QGridLayout;
  
  //addWidget(*Widget, row, column, rowspan, colspan)
  icp_grid_layout->addWidget(do_icp_, 0, 0, 1, 1);
  icp_grid_layout->addWidget(accept_result_, 0, 1, 1, 1);
  icp_grid_layout->addWidget(set_pose_, 0, 2, 1, 1);

  icp_grid_layout->addWidget(px_increase_, 1, 0, 1, 1);
  icp_grid_layout->addWidget(py_increase_, 1, 1, 1, 1);
  icp_grid_layout->addWidget(pz_increase_, 1, 2, 1, 1);
  icp_grid_layout->addWidget(roll_increase_, 1, 3, 1, 1);
  icp_grid_layout->addWidget(pitch_increase_, 1, 4, 1, 1);
  icp_grid_layout->addWidget(yaw_increase_, 1, 5, 1, 1);

  icp_grid_layout->addWidget(px_decrease_, 2, 0, 1, 1);
  icp_grid_layout->addWidget(py_decrease_, 2, 1, 1, 1);
  icp_grid_layout->addWidget(pz_decrease_, 2, 2, 1, 1);
  icp_grid_layout->addWidget(roll_decrease_, 2, 3, 1, 1);
  icp_grid_layout->addWidget(pitch_decrease_, 2, 4, 1, 1);
  icp_grid_layout->addWidget(yaw_decrease_, 2, 5, 1, 1);

  QLabel *trans_step_label = new QLabel(tr("Trans Step:"), this);
  trans_step_spin_ = new QDoubleSpinBox(this);
  trans_step_spin_->setRange(0.001, 100.0);
  trans_step_spin_->setValue(0.5);
  trans_step_spin_->setSingleStep(0.1);
  trans_step_spin_->setDecimals(3);

  QLabel *rot_step_label = new QLabel(tr("Rot Step:"), this);
  rot_step_spin_ = new QDoubleSpinBox(this);
  rot_step_spin_->setRange(0.001, 3.14159);
  rot_step_spin_->setValue(0.1);
  rot_step_spin_->setSingleStep(0.05);
  rot_step_spin_->setDecimals(3);

  icp_grid_layout->addWidget(trans_step_label, 3, 0, 1, 1);
  icp_grid_layout->addWidget(trans_step_spin_, 3, 1, 1, 2);
  icp_grid_layout->addWidget(rot_step_label, 3, 3, 1, 1);
  icp_grid_layout->addWidget(rot_step_spin_, 3, 4, 1, 2);

  connect(trans_step_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MappingPanel::setTransStepValue);
  connect(rot_step_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MappingPanel::setRotStepValue);

  icp_groupBox->setLayout(icp_grid_layout);

  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(operation_groupBox);
  layout->addWidget(icp_scoreBox);
  layout->addWidget(history_keyframe_search_radiusBox);
  layout->addWidget(skip_frameBox);
  layout->addWidget(icp_groupBox);
  setLayout(layout);
  
}

void MappingPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pause_resume_pub_ = raw_node->create_publisher<std_msgs::msg::Bool>("lego_loam_bag_pause", 1);
  icp_score_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("lego_loam_bag_icp_score", 1);
  history_keyframe_search_radius_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("lego_loam_bag_history_keyframe_search_radius", 1);
  skip_frame_pub_ = raw_node->create_publisher<std_msgs::msg::Int32>("lego_loam_bag_skip_frame", 1);
  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("operation_command_msg", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  clock_ = raw_node->get_clock();

}

void MappingPanel::pubOperationCommand(std::string m_string){
  std_msgs::msg::String tmp_str;
  tmp_str.data = m_string;
  operation_command_pub_->publish(tmp_str);
}

void MappingPanel::icp_clicked(){ pubOperationCommand(std::string("icp")); }
void MappingPanel::accept_clicked(){ pubOperationCommand(std::string("accept")); }
void MappingPanel::set_pose_clicked(){ pubOperationCommand(std::string("set_pose")); }

void MappingPanel::pxIncrease(){ pubOperationCommand(std::string("px+")); }
void MappingPanel::pxDecrease(){ pubOperationCommand(std::string("px-")); }
void MappingPanel::pyIncrease(){ pubOperationCommand(std::string("py+")); }
void MappingPanel::pyDecrease(){ pubOperationCommand(std::string("py-")); }
void MappingPanel::pzIncrease(){ pubOperationCommand(std::string("pz+")); }
void MappingPanel::pzDecrease(){ pubOperationCommand(std::string("pz-")); }

void MappingPanel::rollIncrease(){ pubOperationCommand(std::string("roll+")); }
void MappingPanel::rollDecrease(){ pubOperationCommand(std::string("roll-")); }
void MappingPanel::pitchIncrease(){ pubOperationCommand(std::string("pitch+")); }
void MappingPanel::pitchDecrease(){ pubOperationCommand(std::string("pitch-")); }
void MappingPanel::yawIncrease(){ pubOperationCommand(std::string("yaw+")); }
void MappingPanel::yawDecrease(){ pubOperationCommand(std::string("yaw-")); }


void MappingPanel::pause(){
  std_msgs::msg::Bool tmp_bool;
  tmp_bool.data = true;
  pause_resume_pub_->publish(tmp_bool);
}

void MappingPanel::resume(){
  std_msgs::msg::Bool tmp_bool;
  tmp_bool.data = false;
  pause_resume_pub_->publish(tmp_bool);
}

void MappingPanel::setICPScoreValue(){
  icp_score_value_->setText(std::to_string(icp_score_slider_->value()*0.1).c_str());
  std_msgs::msg::Float32 tmp_float;
  tmp_float.data = icp_score_slider_->value()*0.1;
  icp_score_pub_->publish(tmp_float);
}

void MappingPanel::setHistoryKeyframeSearchRadiusValue(){
  history_keyframe_search_radius_value_->setText(std::to_string(history_keyframe_search_radius_slider_->value()).c_str());
  std_msgs::msg::Float32 tmp_float;
  tmp_float.data = history_keyframe_search_radius_slider_->value();
  history_keyframe_search_radius_pub_->publish(tmp_float);
}

void MappingPanel::setSkipFrameValue(){
  skip_frame_value_->setText(std::to_string(skip_frame_slider_->value()).c_str());
  std_msgs::msg::Int32 tmp_int;
  tmp_int.data = skip_frame_slider_->value();
  skip_frame_pub_->publish(tmp_int);
}

void MappingPanel::setTransStepValue(){
  pubOperationCommand(std::string("trans_step: ") + std::to_string(trans_step_spin_->value()));
}

void MappingPanel::setRotStepValue(){
  pubOperationCommand(std::string("rot_step: ") + std::to_string(rot_step_spin_->value()));
}

}  // namespace mapping_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mapping_panel::MappingPanel, rviz_common::Panel)
