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

#ifndef _ACKERMANN_SIMPLE_TRAJECTORY_GENERATOR_PARAMS_H__
#define _ACKERMANN_SIMPLE_TRAJECTORY_GENERATOR_PARAMS_H__

#include <Eigen/Core>
//@ pcl for cuboid
#include <pcl/point_cloud.h>

namespace trajectory_generators
{
class AckermannTrajectoryGeneratorParams
{
public:

  double controller_frequency;
  double sim_time;
  double linear_x_sample;
  double angular_z_sample;
  double sim_granularity;
  double angular_sim_granularity;
  pcl::PointCloud<pcl::PointXYZ> cuboid;

  AckermannTrajectoryGeneratorParams() {}

  AckermannTrajectoryGeneratorParams(
      double ncontroller_frequency,
      double nsim_time,
      double nlinear_x_sample,
      double nangular_z_sample,
      double nsim_granularity,
      double nangular_sim_granularity):
        controller_frequency(ncontroller_frequency),
        sim_time(nsim_time),
        linear_x_sample(nlinear_x_sample),
        angular_z_sample(nangular_z_sample),
        sim_granularity(nsim_granularity),
        angular_sim_granularity(nangular_sim_granularity)

{}

  ~AckermannTrajectoryGeneratorParams() {}


};

}
#endif // _ACKERMANN_SIMPLE_TRAJECTORY_GENERATOR_PARAMS_H__
