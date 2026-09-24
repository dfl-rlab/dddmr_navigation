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
#include <perception_3d/sensor.h>

namespace perception_3d
{

Sensor::Sensor(){
  return;
}

void Sensor::setSharedData(std::shared_ptr<perception_3d::SharedData> shared_data){
  shared_data_ = shared_data;
}

void Sensor::initialize(std::string name,
        const rclcpp::Node::WeakPtr& weak_node,
        const std::shared_ptr<perception_3d::GlobalUtils>& gbl_utils){
    
  node_ = weak_node.lock();
  name_ = name;
  gbl_utils_ = gbl_utils;
  shared_data_->dgraph_update_request_[name_] = true;
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_current_observation_robot_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  expected_sensor_time_ = 0.1;
  current_ = false;
  onInitialize();
  opinion_ = perception_3d::PASS;
}

unsigned long Sensor::get_dGraphSize(){
  return dGraph_.getdGraphSize();
}

}//end of name space