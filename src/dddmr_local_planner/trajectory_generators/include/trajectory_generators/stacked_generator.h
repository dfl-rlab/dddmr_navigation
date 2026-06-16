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
/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins*/
#include <pluginlib/class_loader.hpp>

namespace trajectory_generators
{

class StackedGenerator{

  public:

    StackedGenerator(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
                    std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer);
    ~StackedGenerator();

    /*Plugin operations*/
    void addPlugin(std::string, std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory> theory);

    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> getSharedDataPtr(){return shared_data_;}

    void initializeTheories_wi_Shared_data();
    bool hasMoreTrajectories(std::string pname);
    bool nextTrajectory(std::string pname, base_trajectory::Trajectory& comp_traj);
    bool theoryExists(std::string pname);
    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex theory_mutex_t;
    theory_mutex_t* getMutex()
    {
      return access_;
    }

  private:

    std::map<std::string, std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory>> theories_;
    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data_;
    /*mutex*/
    theory_mutex_t* access_;
    
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

};

}//end of name space