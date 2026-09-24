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
#include "perception_3d/shared_data.h"

namespace perception_3d
{

SharedData::SharedData(){
  is_static_layer_ready_ = false;
  static_map_size_ = 0;
  static_ground_size_ = 0;
  aggregate_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  aggregate_observation_robot_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  current_allowed_max_linear_speed_ = -1.0;
}

void SharedData::requestAllLayersToResetDGraph(){
  for(auto i=dgraph_update_request_.begin();i!=dgraph_update_request_.end();i++){
    (*i).second = true;
  }
}

bool SharedData::isAllLayersBeenReset(){
  for(auto i=dgraph_update_request_.begin();i!=dgraph_update_request_.end();i++){
    if((*i).second)
      return false;
  }
  return true;
}

}//end of name space