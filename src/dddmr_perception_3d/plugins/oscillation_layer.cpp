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
#include <perception_3d/oscillation_layer.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::OscillationLayer, perception_3d::Sensor)

namespace perception_3d
{

OscillationLayer::OscillationLayer(){
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

OscillationLayer::~OscillationLayer(){
}

void OscillationLayer::onInitialize()
{ 
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void OscillationLayer::updateLethalPointCloud(){
}

void OscillationLayer::selfMark(){
  //@ check timestamp closed between latest two
  //@ check if paths are deviated at 3 meters length, if yes, use kdtree_ground to find those close to t-1 path
  
}

void OscillationLayer::selfClear(){
}

void OscillationLayer::resetdGraph(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
}

double OscillationLayer::get_dGraphValue(const unsigned int index){
  return 1000000.0;
}

bool OscillationLayer::isCurrent(){
  current_ = true;
  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OscillationLayer::getObservation(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr OscillationLayer::getLethal(){
  return current_lethal_;
}
}//end of name space

