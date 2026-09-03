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
#include <trajectory_generators/trajectory_generator_theory.h>

namespace trajectory_generators
{

TrajectoryGeneratorTheory::TrajectoryGeneratorTheory(){

}

void TrajectoryGeneratorTheory::initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node){
  name_ = name;
  node_ = weak_node.lock();
  // Each concrete generator declares the command interface it produces
  // (MOTOR/Twist or STEERING/AckermannDrive).  This must happen before a
  // generated Trajectory copies actuator_type_.
  configurateActuatorType();
  onInitialize();
}

void TrajectoryGeneratorTheory::setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data){
  shared_data_ = shared_data;
}

void TrajectoryGeneratorTheory::expertScoring(std::vector<base_trajectory::Trajectory>& accepted_trajectories,
                                                std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories, 
                                                  base_trajectory::Trajectory& best_traj){

    best_traj.cost_ = -1;
    best_traj.xv_ = 0.0;
    best_traj.yv_ = 0.0;
    double minimum_cost = 9999999;

    for(auto& a_traj:accepted_trajectories){
      if(a_traj.getPosesSize()>0 && a_traj.cost_>=0 && a_traj.cost_<=minimum_cost){
        best_traj = a_traj;
        minimum_cost = a_traj.cost_;
      }
    }
}

}//end of name space
