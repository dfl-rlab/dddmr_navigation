#ifndef PC2DEPTHIMG_PANEL_H
#define PC2DEPTHIMG_PANEL_H

#include <QMainWindow>
#include "rviz_common/panel.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QRadioButton>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QSlider>

#include "rclcpp/rclcpp.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/tool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

// PCL
#include "pcl/common/transforms.h"
#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

// ROS msg
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

//@ for mkdir
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

namespace pc2depthimg_panel
{
  class Pc2DepthImgPanel : public rviz_common::Panel // QMainWindow
{

  Q_OBJECT
  
  public:

    explicit Pc2DepthImgPanel(QWidget *parent = nullptr);

    void onInitialize() override;

  private:

    QPushButton *pause_btn_;
    QPushButton *resume_btn_;
    QPushButton *next_frame_btn_;
    QPushButton *export_selection_btn_;
    QPushButton *clear_selection_btn_;

    QSlider *xmax_slider_;
    QLabel *xmax_value_;
    QSlider *xmin_slider_;
    QLabel *xmin_value_;
    QSlider *ymax_slider_;
    QLabel *ymax_value_;
    QSlider *ymin_slider_;
    QLabel *ymin_value_;
    QSlider *zmax_slider_;
    QLabel *zmax_value_;
    QSlider *zmin_slider_;
    QLabel *zmin_value_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_resume_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selection_panel_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xmax_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xmin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ymax_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ymin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr zmax_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr zmin_pub_;

    rclcpp::Clock::SharedPtr clock_;
    
  // Here we declare some internal slots.
  protected Q_SLOTS:

    void pause();
    void resume();
    void nextFrame();
    void exportCurrentSelection();
    void clearSelection();
    void setXmaxValue();
    void setXminValue();
    void setYmaxValue();
    void setYminValue();
    void setZmaxValue();
    void setZminValue();

    void pubOperationCommand(std::string m_string);

};

} // namespace pc2depthimg_panel

#endif // PC2DEPTHIMG_PANEL_H
