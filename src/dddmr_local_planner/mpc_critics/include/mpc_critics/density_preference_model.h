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
#ifndef MPC_CRITICS_DENSITY_PREFERENCE_MODEL_H_
#define MPC_CRITICS_DENSITY_PREFERENCE_MODEL_H_

#include <mpc_critics/scoring_model.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace mpc_critics
{

class DensityPreferenceModel: public ScoringModel{

  public:
    
    DensityPreferenceModel();
    virtual double scoreTrajectory(base_trajectory::Trajectory &traj);

  protected:

    virtual void onInitialize();

  private:

    void densityCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::string topic_name_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr density_cloud_sub_;
    geometry_msgs::msg::TransformStamped target_to_base_tf_;
    bool got_tf_{false};
    pcl::PointCloud<pcl::PointXYZ>::Ptr density_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr density_cloud_kd_tree_;
};

}//end of name space

#endif  // MPC_CRITICS_DENSITY_PREFERENCE_MODEL_H_
