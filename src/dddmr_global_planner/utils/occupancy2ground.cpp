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
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/filters/voxel_grid.h>

// omp voxel
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <omp.h>

// kdtree
#include <pcl/kdtree/kdtree_flann.h>

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
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_ds_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_wall_ds_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_wall_, pc_wall_flat_;

    double inflation_radius_;
    std::vector<std::pair<int, int>> obstacles_;
    float ground_voxel_size_;

    double obstacle_weight_;
    float resolution_;
    std::vector<float> origin_;
    double free_thresh_;
    double occupied_thresh_;
    int negate_;
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

  this->declare_parameter("obstacle_weight", rclcpp::ParameterValue(100.0));
  this->get_parameter("obstacle_weight", obstacle_weight_);
  RCLCPP_INFO(this->get_logger(), "obstacle_weight: %.2f" , obstacle_weight_);


  if(!std::filesystem::exists(map_dir_))
  {
    RCLCPP_INFO(this->get_logger(), "Directory: %s not exist, exit.", map_dir_.c_str());
    return;
  }

  std::string yaml_path = map_dir_ + "/map.yaml";
  if(!std::filesystem::exists(yaml_path))
  {
    RCLCPP_INFO(this->get_logger(), "File: %s not exist, exit.", yaml_path.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Reading yaml from: %s", yaml_path.c_str());
  YAML::Node config = YAML::LoadFile(yaml_path);
  std::string image_name = config["image"].as<std::string>();
  resolution_ = config["resolution"].as<float>();
  origin_ = config["origin"].as<std::vector<float>>();
  free_thresh_ = config["free_thresh"] ? config["free_thresh"].as<double>() : 0.196;
  occupied_thresh_ = config["occupied_thresh"] ? config["occupied_thresh"].as<double>() : 0.65;
  negate_ = config["negate"] ? config["negate"].as<int>() : 0;

  std::string pgm_path = map_dir_ + "/" + image_name;

  RCLCPP_INFO(this->get_logger(), "Reading file from: %s", pgm_path.c_str());
  pgm_t_ = readPGM(pgm_path);
  img2Ground();
}

void Occupancy2Ground::img2Ground() {

  std::vector<pcl::PointXYZI> grid(pgm_t_.pixelData.size());
  std::vector<uint8_t> valid(pgm_t_.pixelData.size(), 0);
  
  int num_threads = omp_get_max_threads();
  std::vector<pcl::PointCloud<pcl::PointXYZI>> local_pc_walls(num_threads);
  std::vector<pcl::PointCloud<pcl::PointXYZI>> local_pc_wall_flats(num_threads);
  std::vector<std::vector<std::pair<int, int>>> local_obstacles(num_threads);

  #pragma omp parallel for
  for (size_t i = 0; i < pgm_t_.pixelData.size(); ++i) {
    int tid = omp_get_thread_num();
    double p = negate_ ? (pgm_t_.pixelData[i] / 255.0) : ((255.0 - pgm_t_.pixelData[i]) / 255.0);
    if(p < free_thresh_){
      pcl::PointXYZI pt;
      pt.x = (i%pgm_t_.width)*resolution_ + origin_[0];
      pt.y = pgm_t_.height*resolution_ - (int)(i/pgm_t_.width)*resolution_ + origin_[1];
      pt.z = 0.0;
      pt.intensity = 0.0;
      grid[i] = pt;
      valid[i] = 1;
    }
    else if(p > occupied_thresh_){
      //@ create vertical structure
      for(int j=0;j<5;j++){
        pcl::PointXYZI pt;
        pt.x = (i%pgm_t_.width)*resolution_ + origin_[0];
        pt.y = pgm_t_.height*resolution_ - (int)(i/pgm_t_.width)*resolution_ + origin_[1];
        pt.z = j*0.2;
        local_pc_walls[tid].push_back(pt);
        if(j==0)
          local_pc_wall_flats[tid].push_back(pt);
      }
      pcl::PointXYZI pt;
      pt.x = (i%pgm_t_.width)*resolution_ + origin_[0];
      pt.y = pgm_t_.height*resolution_ - (int)(i/pgm_t_.width)*resolution_ + origin_[1];
      pt.z = 0.0;
      pt.intensity = obstacle_weight_;
      grid[i] = pt;
      valid[i] = 1;
      local_obstacles[tid].push_back(std::make_pair(i%pgm_t_.width, (int)(i/pgm_t_.width)));
    }
  }
  
  for(int t = 0; t < num_threads; ++t) {
    *pc_wall_ += local_pc_walls[t];
    *pc_wall_flat_ += local_pc_wall_flats[t];
    obstacles_.insert(obstacles_.end(), local_obstacles[t].begin(), local_obstacles[t].end());
  }

  pc_ground_->points.clear();
  for (size_t i = 0; i < grid.size(); ++i) {
    if (valid[i]) {
      pc_ground_->points.push_back(grid[i]);
    }
  }
  pc_ground_->width = pc_ground_->points.size();
  pc_ground_->height = 1;
  
  pc_ground_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_ground_ds_ = small_gicp::voxelgrid_sampling_omp(*pc_ground_, ground_voxel_size_, 6);
  RCLCPP_INFO(this->get_logger(), "Origin ground: %lu, DS: %lu", pc_ground_->points.size(), pc_ground_ds_->points.size());
  
  //@ now loop voxelized ground and doing z search
  pcl::KdTreeFLANN<pcl::PointXYZI> map_kdtree;
  map_kdtree.setInputCloud(pc_wall_flat_);
  for(auto i=pc_ground_ds_->points.begin(); i!=pc_ground_ds_->points.end(); i++){
    pcl::PointXYZI query_point = (*i);
    std::vector<int> point_idx_radius;
    std::vector<float> point_radius_squared_distance;
    if (map_kdtree.radiusSearch(query_point, inflation_radius_, point_idx_radius, point_radius_squared_distance) > 0){
      double dis2ob = (inflation_radius_+0.01)-sqrt(point_radius_squared_distance[0]);
      (*i).intensity = obstacle_weight_*dis2ob;
    }
  }

  sensor_msgs::msg::PointCloud2 ros_msg_map_ground;
  pcl::toROSMsg(*pc_ground_ds_, ros_msg_map_ground);
  ros_msg_map_ground.header.frame_id = "map";
  pub_ground_->publish(ros_msg_map_ground);
  
  pc_wall_ds_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_wall_ds_ = small_gicp::voxelgrid_sampling_omp(*pc_wall_, 0.2, 6);
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
