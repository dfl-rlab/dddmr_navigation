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
#include <trajectory_generators/uuv_simple_trajectory_generator_limits.h>
#include <trajectory_generators/uuv_simple_trajectory_generator_params.h>
#include <trajectory_generators/velocity_iterator.h>

/*getMinMax3D*/
#include <pcl/common/common.h>

namespace trajectory_generators
{

class UUVSimpleTrajectoryGeneratorTheory: public TrajectoryGeneratorTheory{

  public:
    
    UUVSimpleTrajectoryGeneratorTheory();
    virtual size_t getSamplingSize();
    virtual void getSamplingTrajectoryByIndex(size_t index, base_trajectory::Trajectory& _traj);
    void expertScoring(std::vector<base_trajectory::Trajectory>& accepted_trajectories,
                        std::map<std::string, std::vector<base_trajectory::Trajectory>>& rejected_trajectories, 
                          base_trajectory::Trajectory& best_traj) override;

  private:
    void initialise();
    bool isPowerConstraintSatisfied(Eigen::VectorXf& vel6d);

    bool generateTrajectory(
        Eigen::VectorXf& sample_target_vel,
        base_trajectory::Trajectory& traj);

    Eigen::VectorXf computeNewPositions(const Eigen::VectorXf& pos,
                                        const Eigen::VectorXf& vel6d, double dt);

  protected:

    virtual void onInitialize();
    virtual void configurateActuatorType();
    
    std::shared_ptr<trajectory_generators::UUVTrajectoryGeneratorLimits> limits_;
    std::shared_ptr<trajectory_generators::UUVTrajectoryGeneratorParams> params_;

    unsigned int next_sample_index_;
    // to store sample params of each sample between init and generation
    std::vector<Eigen::VectorXf> sample_params_;
};

}//end of name space
