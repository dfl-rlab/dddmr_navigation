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
#include <base_trajectory/trajectory.h>

namespace base_trajectory {
  Trajectory::Trajectory()
    : xv_(0.0), yv_(0.0), thetav_(0.0), cost_(-1.0)
  {
  }

  Trajectory::Trajectory(double xv, double yv, double thetav, double time_delta, unsigned int num_pts)
    : xv_(xv), yv_(yv), thetav_(thetav), cost_(-1.0), time_delta_(time_delta)
  {
  }

  geometry_msgs::msg::PoseStamped Trajectory::getPose(unsigned int index) const {
    return trajectory_path_.poses[index];
  }

  pcl::PointXYZI Trajectory::getPCLPoint(unsigned int index) const{
    return pcl_trajectory_path_.points[index];
  }

  pcl::PointCloud<pcl::PointXYZ> Trajectory::getCuboid(unsigned int index) const {
    return cuboids_[index];
  }

  cuboid_min_max_t Trajectory::getCuboidMinMax(unsigned int index) const {
    return cuboids_min_max_[index];
  }

  bool Trajectory::addPoseCuboid(const geometry_msgs::msg::PoseStamped& pos, 
                            const pcl::PointCloud<pcl::PointXYZ>& cuboid,
                            const cuboid_min_max_t& cuboid_min_max){
    trajectory_path_.poses.push_back(pos);
    cuboids_.push_back(cuboid);
    cuboids_min_max_.push_back(cuboid_min_max);
    //@ for pcl
    pcl::PointXYZI ipt;
    ipt.x = pos.pose.position.x;
    ipt.y = pos.pose.position.y;
    ipt.z = pos.pose.position.z;
    ipt.intensity = 0.;
    pcl_trajectory_path_.push_back(ipt);

    return true;

  }

  void Trajectory::resetPoses(){
    trajectory_path_.poses.clear();
    cuboids_.clear();
    cuboids_min_max_.clear();
    pcl_trajectory_path_.points.clear();
  }

  void Trajectory::getEndpoint(double& x, double& y, double& th) const {

  }

  unsigned int Trajectory::getPosesSize() const {
    return trajectory_path_.poses.size();
  }
};
