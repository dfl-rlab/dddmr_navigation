/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MCL_3DL_CLASS_H
#define MCL_3DL_CLASS_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <chrono>

#include <mcl_3dl/parameters.h>


#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

//@kdtree and normals
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <mcl_3dl/filter.h>
#include <mcl_3dl/filter_vec3.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>
#include <mcl_3dl/motion_prediction_model_base.h>
#include <mcl_3dl/motion_prediction_models/motion_prediction_model_differential_drive.h>
#include <mcl_3dl/nd.h>
#include <mcl_3dl/noise_generators/multivariate_noise_generator.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

/*sync topics from lego loam from featureAssociation node.*/
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>

/*For pcl::transformPointCloud, dont use #include <pcl/common/transforms.h> ???*/
#include <pcl/common/transforms.h>


/*For normal markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


/*thread/mutex for tf pub
#include <thread>
#include <mutex>
*/

/*This is for euclidean distance segmentation*/
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

/*Estimate normal*/
#include <pcl/features/normal_3d.h>

/*sub maps*/
#include <mcl_3dl/sub_maps.h>

namespace mcl_3dl
{
class MCL3dlNode : public rclcpp::Node 
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, 
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> LegoSyncPolicy;

  public:
    MCL3dlNode(std::string name);

    bool configure(const std::shared_ptr<mcl_3dl::SubMaps>& sub_maps);
    ~MCL3dlNode();

    //@Prepare all subscribers from LegoLoam
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lc_sharp_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lc_less_sharp_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lc_flat_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lc_less_flat_;
    std::shared_ptr<message_filters::Synchronizer<LegoSyncPolicy>> syncApproximate_;

  private:

    bool is_trans_b2s_initialized_;
    bool tf_ready_;
    bool first_tf_;
    
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_position_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_normal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_ec_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particle_;

    void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool getBaselink2SensorAF3(std_msgs::msg::Header sensor_header, Eigen::Affine3d& trans_b2s_af3);

    void cbLeGoFeatureCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc_sharpMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_less_sharpMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_flatMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_less_flatMsg);
    
    void measure(std::map<std::string, pcl::PointCloud<pcl_t>::Ptr> pcl_segmentations);
    
    void publishParticles();

    void publishTFThread();
    
    void cbPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    /*
    ros::Subscriber sub_landmark_;

    ros::Publisher pub_pc_normal_;
    ros::ServiceServer srv_expansion_reset_;
    */
    std::shared_ptr<tf2_ros::Buffer> tfbuf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    
    geometry_msgs::msg::TransformStamped trans_b2s_;

    rclcpp::CallbackGroup::SharedPtr cbs_group_;
    rclcpp::CallbackGroup::SharedPtr tf_pub_group_;   
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    rclcpp::TimerBase::SharedPtr tf_pub_timer_;
    
    geometry_msgs::msg::TransformStamped odom_trans_;
    geometry_msgs::msg::TransformStamped map2odom_trans_;
    //std::mutex tf_pub_mutex_;
    
    std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> pcl_segmentations_;
    
    /*
    
    void cbLandmark(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    bool cbExpansionReset(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

    void normal2quaternion(pcl::PointCloud<pcl::PointNormal>::Ptr i_normals);

    */
  protected:

    size_t global_localization_fix_cnt_;
    std::random_device seed_gen_;
    std::default_random_engine engine_;
    
    std::shared_ptr<Parameters> params_;
    
    std::shared_ptr<LidarMeasurementModelLikelihood> lidar_measurements_;
    
    std::shared_ptr<pf::ParticleFilter<State6DOF, float, ParticleWeightedMeanQuat, std::default_random_engine>> pf_;
    
    std::shared_ptr<FilterVec3> f_pos_;
    std::shared_ptr<FilterVec3> f_ang_;
    std::shared_ptr<FilterVec3> f_acc_;
    
    rclcpp::Time localized_last_;
    //rclcpp::Duration tf_tolerance_base_;

    rclcpp::Time match_output_last_;
    rclcpp::Time odom_last_;
    bool has_odom_;
    std_msgs::msg::Header laser_header_;
    std_msgs::msg::Header odom_header_;
    State6DOF odom_;
    State6DOF odom_prev_;
    State6DOF state_prev_;
    
    MotionPredictionModelBase::Ptr motion_prediction_model_;

    std::shared_ptr<mcl_3dl::SubMaps> sub_maps_;
    std::mutex protect_measure_in_odomcb_;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_CLASS_H
