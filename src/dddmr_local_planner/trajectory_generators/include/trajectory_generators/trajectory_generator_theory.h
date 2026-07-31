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
/*Debug*/
#include <chrono>

#include <pluginlib/class_list_macros.hpp>

/*path for trajectory*/
#include <base_trajectory/trajectory.h>
/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/transforms.h>

/*Data shared (if needed) between trajectory theories*/
#include <trajectory_generators/trajectory_shared_data.h>

namespace trajectory_generators
{

class TrajectoryGeneratorTheory{

  public:

    TrajectoryGeneratorTheory();

    void initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node);

    void setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data);

    virtual bool hasMoreTrajectories() = 0;
    virtual bool nextTrajectory(base_trajectory::Trajectory& _traj) = 0;
    //@ initialise is used for stacked generators to call every time to initialize the genertator
    virtual void initialise() = 0;

  protected:

    rclcpp::Node::SharedPtr node_;
    //@onInitialize is used to read ros param for the generator
    virtual void onInitialize() = 0;
    virtual void configurateActuatorType() = 0;
    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data_;
    std::string name_;
    dddmr_sys_core::ActuatorType actuator_type_;

};


}//end of name space