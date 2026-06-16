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
#include "rclcpp/rclcpp.hpp"

#include "pcl/common/transforms.h"
#include "pcl/PCLPointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;

// Structure to hold PGM image data
typedef struct PGMImage {
  std::string magicNumber;
  int width;
  int height;
  int maxGrayValue;
  std::vector<unsigned char> pixelData; // For 8-bit grayscale
  // For 16-bit, you might use std::vector<unsigned short>
} PGMImage_t;

class Occupancy2Ground : public rclcpp::Node
{

  public:

    Occupancy2Ground();
    PGMImage_t readPGM(const std::string& filename);
    void img2Ground();

  private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_wall_;

    std::string map_dir_;

    PGMImage_t pgm_t_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_wall_, pc_wall_flat_;

    double inflation_radius_;
    std::map<std::pair<int, int>, size_t> twoD2oneD_;
    std::vector<std::pair<int, int>> obstacles_;
    float ground_voxel_size_;

    double map_rotate_around_x_, map_rotate_around_y_, map_rotate_around_z_;
    double map_translate_x_, map_translate_y_, map_translate_z_;
};


Occupancy2Ground::Occupancy2Ground():Node("occupancy2ground"){

  pc_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_wall_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_wall_flat_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapground",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pub_wall_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  this->declare_parameter("map_dir", rclcpp::ParameterValue(""));
  this->get_parameter("map_dir", map_dir_);
  RCLCPP_INFO(this->get_logger(), "map_dir: %s" , map_dir_.c_str());

  this->declare_parameter("inflation_radius", rclcpp::ParameterValue(1.0));
  this->get_parameter("inflation_radius", inflation_radius_);
  RCLCPP_INFO(this->get_logger(), "inflation_radius: %.2f" , inflation_radius_);

  this->declare_parameter("ground_voxel_size", rclcpp::ParameterValue(0.1f));
  this->get_parameter("ground_voxel_size", ground_voxel_size_);
  RCLCPP_INFO(this->get_logger(), "ground_voxel_size: %.2f" , ground_voxel_size_);

  this->declare_parameter("map_rotate_around_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_rotate_around_x = this->get_parameter("map_rotate_around_x");
  map_rotate_around_x_ = map_rotate_around_x.as_double();
  RCLCPP_INFO(this->get_logger(), "map_rotate_around_x: %.2f", map_rotate_around_x_);

  this->declare_parameter("map_rotate_around_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_rotate_around_y = this->get_parameter("map_rotate_around_y");
  map_rotate_around_y_ = map_rotate_around_y.as_double();
  RCLCPP_INFO(this->get_logger(), "map_rotate_around_y: %.2f", map_rotate_around_y_);

  this->declare_parameter("map_rotate_around_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_rotate_around_z = this->get_parameter("map_rotate_around_z");
  map_rotate_around_z_ = map_rotate_around_z.as_double();
  RCLCPP_INFO(this->get_logger(), "map_rotate_around_z: %.2f", map_rotate_around_z_);

  this->declare_parameter("map_translate_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_translate_x = this->get_parameter("map_translate_x");
  map_translate_x_ = map_translate_x.as_double();
  RCLCPP_INFO(this->get_logger(), "map_translate_x: %.2f", map_translate_x_);

  this->declare_parameter("map_translate_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_translate_y = this->get_parameter("map_translate_y");
  map_translate_y_ = map_translate_y.as_double();
  RCLCPP_INFO(this->get_logger(), "map_translate_y: %.2f", map_translate_y_);

  this->declare_parameter("map_translate_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_translate_z = this->get_parameter("map_translate_z");
  map_translate_z_ = map_translate_z.as_double();
  RCLCPP_INFO(this->get_logger(), "map_translate_z: %.2f", map_translate_z_);

  if(!std::filesystem::exists(map_dir_))
  {
    RCLCPP_INFO(this->get_logger(), "File: %s not exist, exit.", map_dir_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Reading file from: %s", map_dir_.c_str());
  pgm_t_ = readPGM(map_dir_);
  img2Ground();
}

void Occupancy2Ground::img2Ground() {

  pc_ground_->points.resize(pgm_t_.pixelData.size());
  
  for (size_t i = 0; i < pgm_t_.pixelData.size(); ++i) {
    if(pgm_t_.pixelData[i]>200){
      pcl::PointXYZI pt;
      pt.x = (i%pgm_t_.width)*0.05;
      pt.y = pgm_t_.height*0.05 - (int)(i/pgm_t_.width)*0.05;
      pt.z = 0.0;
      pc_ground_->points[i] = pt;
      twoD2oneD_[std::make_pair(i%pgm_t_.width, (int)(i/pgm_t_.width))] = i;
    }
    else{
      //@ create vertical structure
      for(int j=0;j<5;j++){
        pcl::PointXYZI pt;
        pt.x = (i%pgm_t_.width)*0.05;
        pt.y = pgm_t_.height*0.05 - (int)(i/pgm_t_.width)*0.05;
        pt.z = j*0.2;
        pc_wall_->push_back(pt);
        if(j==0)
          pc_wall_flat_->push_back(pt);
      }
      pcl::PointXYZI pt;
      pt.x = (i%pgm_t_.width)*0.05;
      pt.y = pgm_t_.height*0.05 - (int)(i/pgm_t_.width)*0.05;
      pt.z = 0.0;
      pt.intensity = 1000;
      pc_ground_->points[i] = pt;
      twoD2oneD_[std::make_pair(i%pgm_t_.width, (int)(i/pgm_t_.width))] = i;
      obstacles_.push_back(std::make_pair(i%pgm_t_.width, (int)(i/pgm_t_.width)));
    }
  }
  
  for(auto it=obstacles_.begin(); it!=obstacles_.end();it++){

    int sx = (*it).first;
    int sy = (*it).second;
    int step = inflation_radius_/0.05;
    for(int dx = -step; dx<=step; dx++){
      for(int dy = -step; dy<=step; dy++){
        if(dx==0 && dy==0)
          continue;
        double d = 1/hypot(dx, dy);
        if (twoD2oneD_.find(std::make_pair(sx+dx,sy+dy)) != twoD2oneD_.end()) {
          size_t i = twoD2oneD_[std::make_pair(sx+dx,sy+dy)];
          if(pc_ground_->points[i].intensity < d)
            pc_ground_->points[i].intensity = d;
        } else {
          // Key does not exist
        }

      }
    }
  }

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (pc_ground_);
  sor.setLeafSize (ground_voxel_size_, ground_voxel_size_, ground_voxel_size_);
  sor.filter (*pc_ground_);

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 0.0 meters on the axis.
  transform_2.translation() << map_translate_x_, map_translate_y_, map_translate_z_;

  // The same rotation matrix as before; theta radians around X axis
  if(fabs(map_rotate_around_x_)>0.01)
    transform_2.rotate (Eigen::AngleAxisf (map_rotate_around_x_, Eigen::Vector3f::UnitX()));
  if(fabs(map_rotate_around_y_)>0.01)
    transform_2.rotate (Eigen::AngleAxisf (map_rotate_around_y_, Eigen::Vector3f::UnitY()));
  if(fabs(map_rotate_around_z_)>0.01)
    transform_2.rotate (Eigen::AngleAxisf (map_rotate_around_z_, Eigen::Vector3f::UnitZ()));  
  
  pcl::transformPointCloud(*pc_ground_, *pc_ground_, transform_2);
  pcl::transformPointCloud(*pc_wall_, *pc_wall_, transform_2);

  sensor_msgs::msg::PointCloud2 ros_msg_map_ground;
  pcl::toROSMsg(*pc_ground_, ros_msg_map_ground);
  ros_msg_map_ground.header.frame_id = "map";
  pub_ground_->publish(ros_msg_map_ground);

  sensor_msgs::msg::PointCloud2 ros_msg_map_wall;
  pcl::toROSMsg(*pc_wall_, ros_msg_map_wall);
  ros_msg_map_wall.header.frame_id = "map";
  pub_wall_->publish(ros_msg_map_wall);
  
  RCLCPP_INFO(this->get_logger(), "All cloud published");

}

PGMImage_t Occupancy2Ground::readPGM(const std::string& filename) {

    PGMImage_t image;

    std::ifstream file(filename, std::ios::binary); // Open in binary mode for both P2 and P5

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
      return image; // Return empty image
    }

    // Read header
    file >> image.magicNumber;
    if (image.magicNumber != "P2" && image.magicNumber != "P5") {
      RCLCPP_ERROR(this->get_logger(), "Invalid PGM magic number.");
      file.close();
      return image;
    }

    // Skip comments
    std::string line;
    std::getline(file, line); // Consume the rest of the magic number line
    while (file.peek() == '#') {
      std::getline(file, line);
    }

    file >> image.width >> image.height >> image.maxGrayValue;

    // Consume the newline character after maxGrayValue
    std::getline(file, line);

    // Read pixel data
    int numPixels = image.width * image.height;
    image.pixelData.resize(numPixels);

    if (image.magicNumber == "P2") { // ASCII PGM
      for (int i = 0; i < numPixels; ++i) {
        int pixelValue;
        file >> pixelValue;
        image.pixelData[i] = static_cast<unsigned char>(pixelValue);
      }
    } else if (image.magicNumber == "P5") { // Binary PGM
      file.read(reinterpret_cast<char*>(image.pixelData.data()), numPixels * sizeof(unsigned char));
    }

    file.close();

    if (!image.pixelData.empty()) {
      RCLCPP_INFO(this->get_logger(), "Width: %d, Height: %d", image.width, image.height);
      //for (int i = 0; i < std::min(10, (int)myImage.pixelData.size()); ++i) {
      //  std::cout << "Pixel " << i << ": " << (int)myImage.pixelData[i] << std::endl;
      //}
    }

    return image;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Occupancy2Ground>());
  rclcpp::shutdown();
  return 0;
}
