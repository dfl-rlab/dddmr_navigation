#include "map_delete_panel/map_delete_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace map_delete_panel
{

MapDeletePanel::MapDeletePanel(QWidget *parent) : rviz_common::Panel(parent)
{  
  sub_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  map_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  ground_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  output_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  output_ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  //@---------- operation button
  QGroupBox *operation_groupBox = new QGroupBox(tr("Map Delete Panel"));
  save_dir_btn_ = new QPushButton(this);
  save_dir_btn_->setText("Save Modified PointCloud");
  connect(save_dir_btn_, SIGNAL(clicked()), this, SLOT(saveSelectedPC()));

  clear_selection_btn_ = new QPushButton(this);
  clear_selection_btn_->setText("Clear Selection");
  connect(clear_selection_btn_, SIGNAL(clicked()), this, SLOT(clearSelection()));

  last_step_btn_ = new QPushButton(this);
  last_step_btn_->setText("Last Step");
  connect(last_step_btn_, SIGNAL(clicked()), this, SLOT(lastStep()));

  //addWidget(*Widget, row, column, rowspan, colspan)
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->addWidget(save_dir_btn_, 0, 0, 1, 1);
  grid_layout->addWidget(clear_selection_btn_, 0, 1, 1, 1);
  grid_layout->addWidget(last_step_btn_, 0, 2, 1, 1);
  operation_groupBox->setLayout(grid_layout);

  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);

  layout->addWidget(operation_groupBox);
  setLayout(layout);
  
}

void MapDeletePanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  

  operation_command_pub_ = raw_node->create_publisher<std_msgs::msg::String>("point_cloud_delete/panel_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  sub_selected_pc_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud_delete/selected_points", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&MapDeletePanel::selectedPCCb, this, std::placeholders::_1));

  sub_map_pc_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>("mapcloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&MapDeletePanel::mapPCCb, this, std::placeholders::_1));

  sub_ground_pc_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>("mapground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&MapDeletePanel::groundPCCb, this, std::placeholders::_1));
  
  remaining_map_pub_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_delete/remaining_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  remaining_ground_pub_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_delete/remaining_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  clock_ = raw_node->get_clock();

}

void MapDeletePanel::selectedPCCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  pcl::fromROSMsg(*msg, *sub_pc_);
  
  RCLCPP_INFO(rclcpp::get_logger("map_delete_panel"), "Got selected delete point: %lu", sub_pc_->points.size());


  output_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  output_ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::SegmentDifferences<pcl::PointXYZ> segment_map;
  segment_map.setInputCloud(map_pc_);
  segment_map.setTargetCloud(sub_pc_);
  segment_map.setDistanceThreshold(0.01 * 0.01);
  segment_map.segment(*output_map_cloud_);

  sensor_msgs::msg::PointCloud2 output_map;
  pcl::toROSMsg(*output_map_cloud_, output_map);
  output_map.header.frame_id = "map";
  remaining_map_pub_->publish(output_map);


  pcl::SegmentDifferences<pcl::PointXYZ> segment_ground;
  segment_ground.setInputCloud(ground_pc_);
  segment_ground.setTargetCloud(sub_pc_);
  segment_ground.setDistanceThreshold(0.01 * 0.01);
  segment_ground.segment(*output_ground_cloud_);

  sensor_msgs::msg::PointCloud2 output_ground;
  pcl::toROSMsg(*output_ground_cloud_, output_ground);
  output_ground.header.frame_id = "map";
  remaining_ground_pub_->publish(output_ground);

}

void MapDeletePanel::mapPCCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  pcl::fromROSMsg(*msg, *map_pc_);
  remaining_map_pub_->publish(*msg);
}

void MapDeletePanel::groundPCCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  pcl::fromROSMsg(*msg, *ground_pc_);
  remaining_ground_pub_->publish(*msg);
}

void MapDeletePanel::clearSelection(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "clear";
  operation_command_pub_->publish(tmp_str);
}

void MapDeletePanel::lastStep(){
  std_msgs::msg::String tmp_str;
  tmp_str.data = "last_step";
  operation_command_pub_->publish(tmp_str);
}


void MapDeletePanel::saveSelectedPC()
{
  
  QFileDialog FileDialog;
  QString path=FileDialog.getExistingDirectory(this, tr("Select Dir"),  "/", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);

  std::string dir = path.toStdString();
  
  if(dir == ""){
    RCLCPP_INFO(rclcpp::get_logger("map_delete_panel"), "Dir is empty.");
    return;
  }
  else{

    if(output_map_cloud_->points.empty() || output_ground_cloud_->points.empty()){
      RCLCPP_INFO(rclcpp::get_logger("map_delete_panel"), "Map/Ground points is empty.");
      return;
    }
    
    std::string export_dir_string = dir + "/" + currentDateTime()+ "_remaining_map.pcd";
    RCLCPP_INFO(rclcpp::get_logger("map_delete_panel"), "Save modified point cloud to: %s", export_dir_string.c_str());
    pcl::PCDWriter w;
    w.writeASCII (export_dir_string, *output_map_cloud_, 3);

    std::string export_dir_string2 = dir + "/" + currentDateTime()+ "_remaining_ground.pcd";
    RCLCPP_INFO(rclcpp::get_logger("map_delete_panel"), "Save modified point cloud to: %s", export_dir_string2.c_str());
    pcl::PCDWriter w2;
    w2.writeASCII (export_dir_string2, *output_ground_cloud_, 3);

    std_msgs::msg::String tmp_str;
    tmp_str.data = "clear";
    operation_command_pub_->publish(tmp_str);
  }
  

}

}  // namespace map_delete_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_delete_panel::MapDeletePanel, rviz_common::Panel)
