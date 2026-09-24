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
#include <perception_3d/stacked_perception.h>

namespace perception_3d
{


StackedPerception::StackedPerception(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger){
  shared_data_ = std::make_shared<perception_3d::SharedData>();
  access_ = new mutex_t();
  logger_ = m_logger;
  return;
}

StackedPerception::~StackedPerception()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/
void StackedPerception::addPlugin(std::shared_ptr<Sensor> plugin)
{
  plugins_.push_back(plugin);
  plugin->setSharedData(shared_data_);
}

std::vector<std::shared_ptr<Sensor>>* StackedPerception::getPlugins()
{
  return &plugins_;
}

/*
Collect marked pcl from all plugins.
*/

void StackedPerception::doClear_then_Mark(){

  /*
  Say we want a velodyne plus realsense, we want realsense to clear what velodyne saw or vise versa.
  Three step for this perception 3d:
  1. aggregated all observed point clouds
  2. clear marking using kd-tree and observation space, that is why we can clearing from other sensors
  3. marking observations
  */

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    (*plugin)->selfClear();
    (*plugin)->selfMark();
    (*plugin)->updateLethalPointCloud();
  }

}

void StackedPerception::resetdGraph(){

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {

    (*plugin)->resetdGraph();

  }

}

unsigned long StackedPerception::getdGraphSize(){
  unsigned long tmp_val = 1000000;
  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    tmp_val = std::min(tmp_val, (*plugin)->get_dGraphSize());
  }
  return tmp_val;
}

double StackedPerception::get_min_dGraphValue(const unsigned int index){
  
  //@ Everything should be less that 100 km?
  double tmp_val = 99999.9;

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    RCLCPP_DEBUG(logger_->get_logger(), "%s with dGraph value of: %.1f",(*plugin)->getName().c_str(),(*plugin)->get_dGraphValue(index));
    tmp_val = std::min(tmp_val, (*plugin)->get_dGraphValue(index));
  }
  return tmp_val;
}

void StackedPerception::aggregateObservations(){

  shared_data_->aggregate_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->aggregate_observation_robot_frame_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    //@aggregate observation for perception
    (*shared_data_->aggregate_observation_) += (*(*plugin)->getObservation());
    shared_data_->aggregate_observation_->header.frame_id = (*plugin)->getGlobalUtils()->getGblFrame();
    
    //@robot frame
    (*shared_data_->aggregate_observation_robot_frame_) += (*(*plugin)->getObservationRobotFrame());
    shared_data_->aggregate_observation_robot_frame_->header.frame_id = (*plugin)->getGlobalUtils()->getRobotFrame();
  }

}

void StackedPerception::aggregateLethal(){
  
  //@ before calling this function, make sure we mutex lock, i.e.
  shared_data_->aggregate_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    //@aggregate observation for perception
    (*shared_data_->aggregate_lethal_) += (*(*plugin)->getLethal());
    shared_data_->aggregate_lethal_->header.frame_id = (*plugin)->getGlobalUtils()->getGblFrame();
  }

}

std::vector<perception_3d::PerceptionOpinion> StackedPerception::getOpinions(){
  
  std::vector<perception_3d::PerceptionOpinion> tmp_vector;
  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    tmp_vector.push_back((*plugin)->getOpinion());
  }
  return tmp_vector;
}

bool StackedPerception::isSensorOK(){

  bool result = true; //@ it is safe to initialize true here, since current_ is initialized as false in sensor.h
  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    result = result && (*plugin)->isCurrent();
  }
  return result;
}

}//end of name space