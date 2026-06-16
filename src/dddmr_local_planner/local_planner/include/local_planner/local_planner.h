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

#ifndef DDDMR_LOCAL_PLANNER_H_
#define DDDMR_LOCAL_PLANNER_H_

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 
#include <angles/angles.h>
/*For perception plugin*/
#include <perception_3d/perception_3d_ros.h>

/*For critics plugin*/
#include <mpc_critics/mpc_critics_ros.h>

/*For trajectory generators plugin*/
#include <trajectory_generators/trajectory_generators_ros.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

//@planner state
#include <dddmr_sys_core/dddmr_enum_states.h>

namespace local_planner {

class Local_Planner : public rclcpp::Node {

    public:
      Local_Planner(const std::string& name);
      
      void initial(
        const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
        const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
        const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators);

      ~Local_Planner();

      void setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan);
      dddmr_sys_core::PlannerState computeVelocityCommand(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);
      void getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);

      //@ shared data for trajectory generator, we manage the variables by this way for future changed to ROS2
      std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> traj_shared_data_;
      
      bool isGoalReached();
      
      bool isInitialHeadingAligned();
      bool isGoalHeadingAligned();

      void updateGlobalPose();
      geometry_msgs::msg::TransformStamped getGlobalPose();
      std::string getControlFrame();
      
    private: 
      
      rclcpp::Clock::SharedPtr clock_;

      rclcpp::CallbackGroup::SharedPtr cbs_group_;
      rclcpp::CallbackGroup::SharedPtr tf_listener_group_;

      //@ for percertion
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      //@ for critics
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
      //@ for trajectory generator
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;

      std::string global_frame_;
      std::string robot_frame_;
      std::string odom_topic_;
      std::string odom_topic_qos_;
      std::string ackermann_topic_;

      /*For cuboid visualization*/
      visualization_msgs::msg::MarkerArray robot_cuboid_;
      visualization_msgs::msg::Marker marker_edge_;

      /*Original point cloud*/
      pcl::PointCloud<pcl::PointNormal>::Ptr ground_with_normals_;
      
      /*global plan from global planner*/
      std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_global_plan_; 
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_plan_;
      
      /*store global pose*/
      geometry_msgs::msg::TransformStamped trans_gbl2b_;
      
      /*Compute shortest between robot heading and given pose*/
      double getShortestAngleFromPose2RobotHeading(tf2::Transform m_pose);

      /*Pub*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_robot_cuboid_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aggregate_observation_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_prune_plan_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_accepted_trajectory_pose_array_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_best_trajectory_pose_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
      //ros::Publisher pub_pc_normal_;
      //ros::Publisher pub_trajectory_cuboids_;
      

      /*Sub*/
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_ros_sub_;
      rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_ros_sub_;
      
      /*cb*/
      void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
      void cbAckermannDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

      void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids);
      
      void parseCuboid();
      //void normal2quaternion();
      void prunePlan(double forward_distance, double backward_distance);
      double getDistanceBTWPoseStamp(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b);

      bool compute_best_trajectory_in_odomCb_;

      /* Variables for prune. I have no idea to put these variables now.
         This should be optimize in future. The variables affect the control behavior.
         The variable should be adapt to vehicle speed!!!
      */
      double forward_prune_, backward_prune_, heading_tracking_distance_, heading_align_angle_;

      /*Timer for robust system design*/
      double prune_plane_timeout_;
      rclcpp::Time last_valid_prune_plan_;
      bool got_odom_;

      double xy_goal_tolerance_, yaw_goal_tolerance_;
      double controller_frequency_;

      rclcpp::Time control_loop_time_;

      std::map<std::string, std::vector<base_trajectory::Trajectory>> rejected_trajectories_;

    protected:

      std::shared_ptr<tf2_ros::TransformListener> tfl_;
      std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
      nav_msgs::msg::Odometry robot_state_;
      ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_state_;
      std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;
      nav_msgs::msg::Path prune_plan_;
      pcl::PointCloud<pcl::PointXYZI> pcl_prune_plan_; //@ will be copied to perception_ros, so do not use shared_ptr
      std::string name_;
      
};

} // end of name space

#endif  // DDDMR_LOCAL_PLANNER_H_