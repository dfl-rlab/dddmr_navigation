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
#ifndef TRAJECTORY_GENERATOR_ROS_H_
#define TRAJECTORY_GENERATOR_ROS_H_

#include <trajectory_generators/stacked_generator.h>

namespace trajectory_generators
{

class Trajectory_Generators_ROS : public rclcpp::Node {
  
  public:
    Trajectory_Generators_ROS(const std::string& name);
    ~Trajectory_Generators_ROS();
    
    bool hasMoreTrajectories(std::string pname);
    bool nextTrajectory(std::string pname, base_trajectory::Trajectory& comp_traj);
    void initializeTheories_wi_Shared_data();

    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> getSharedDataPtr(){return stacked_generator_->getSharedDataPtr();}

    StackedGenerator* getStackedGeneratorPtr(){return stacked_generator_;}  
    
    bool theoryExists(std::string pname);
    void initial();

  private:
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    /*For plugin loader*/
    pluginlib::ClassLoader<trajectory_generators::TrajectoryGeneratorTheory> theory_loader_;
    std::vector<std::string> plugins_;
    
  protected:

    StackedGenerator* stacked_generator_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
    std::string name_;


};

}//end of name space
#endif
