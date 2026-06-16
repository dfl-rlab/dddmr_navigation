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
#include <recovery_behaviors/rotate_inplace_behavior.h>

PLUGINLIB_EXPORT_CLASS(recovery_behaviors::RotateInPlaceBehavior, recovery_behaviors::RobotBehavior)

namespace recovery_behaviors
{

RotateInPlaceBehavior::RotateInPlaceBehavior(){
  return;
  
}

RotateInPlaceBehavior::~RotateInPlaceBehavior(){
  trajectories_.reset();
}

void RotateInPlaceBehavior::onInitialize(){

  node_->declare_parameter(name_ + ".frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".frequency", frequency_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "frequency: %.2f", frequency_);

  node_->declare_parameter(name_ + ".tolerance", rclcpp::ParameterValue(0.3));
  node_->get_parameter(name_ + ".tolerance", tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "tolerance: %.2f", tolerance_);

  node_->declare_parameter(name_ + ".trajectory_generator_name", rclcpp::ParameterValue("differential_drive_rotate_inplace"));
  node_->get_parameter(name_ + ".trajectory_generator_name", trajectory_generator_name_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "trajectory_generator_name: %s", trajectory_generator_name_.c_str());  

  
  clock_ = node_->get_clock();

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pub_trajectory_pose_array_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(name_ + "_trajectory", 1);
}

void RotateInPlaceBehavior::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr, pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl){

  for(unsigned int i=0;i<a_traj.getPointsSize();i++){
      auto p = a_traj.getPoint(i);
      pose_arr.poses.push_back(p.pose);  
      //@ For cuboids debug
      //cuboids_pcl += a_traj.getCuboid(i);   
  }

}

void RotateInPlaceBehavior::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

  //@ in case we have collision
  best_traj.cost_ = -1;

  double minimum_cost = 9999999;
  geometry_msgs::msg::PoseArray accepted_pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  for(auto traj_it=trajectories_->begin();traj_it!=trajectories_->end();traj_it++){

    mpc_critics_ros_->scoreTrajectory(traj_gen_name, (*traj_it));
    
    if((*traj_it).cost_>=0 && (*traj_it).cost_<=minimum_cost){
      best_traj = (*traj_it);
      minimum_cost = (*traj_it).cost_;
    }

    if((*traj_it).cost_>=0){
      trajectory2posearray_cuboids((*traj_it), accepted_pose_arr, cuboids_pcl);
    }

  }

  geometry_msgs::msg::PoseArray best_pose_arr;
  trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
  best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  best_pose_arr.header.stamp = node_->get_clock()->now();
  pub_trajectory_pose_array_->publish(best_pose_arr);

}

void RotateInPlaceBehavior::trans2Pose(geometry_msgs::msg::TransformStamped& trans, geometry_msgs::msg::PoseStamped& pose){
  pose.header = trans.header;
  pose.pose.position.x = trans.transform.translation.x;
  pose.pose.position.y = trans.transform.translation.y;
  pose.pose.position.z = trans.transform.translation.z;
  pose.pose.orientation.x = trans.transform.rotation.x;
  pose.pose.orientation.y = trans.transform.rotation.y;
  pose.pose.orientation.z = trans.transform.rotation.z;
  pose.pose.orientation.w = trans.transform.rotation.w;
}

void RotateInPlaceBehavior::pubZeroVelocity(){
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

dddmr_sys_core::RecoveryState RotateInPlaceBehavior::runBehavior(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle){

  //@Print thread for debug
  std::stringstream ss;
  ss << std::this_thread::get_id();
  uint64_t id = std::stoull(ss.str());
  //RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior started, thread ID : %lu", id);

  rclcpp::Rate r(frequency_);

  geometry_msgs::msg::PoseStamped global_pose;
  geometry_msgs::msg::TransformStamped trans_gbl2b;
  perception_3d_ros_->getGlobalPose(trans_gbl2b);
  trans2Pose(trans_gbl2b, global_pose);
  
  tf2::Quaternion global_pose_orientation_initial(
    global_pose.pose.orientation.x, 
    global_pose.pose.orientation.y, 
    global_pose.pose.orientation.z, 
    global_pose.pose.orientation.w);

  double current_angle = tf2::impl::getYaw(global_pose_orientation_initial);
  double start_angle = current_angle;

  bool got_180 = false;
  
  rclcpp::Time last_valid_control_ = clock_->now();
  
  dddmr_sys_core::RecoveryState m_recovery_result;
  auto result = std::make_shared<dddmr_sys_core::action::RecoveryBehaviors::Result>();
  while (rclcpp::ok() &&
         (!got_180 ||
          std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
  {

    RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *clock_, 5000, "Behavior: %s is running with thread ID : %lu", name_.c_str(), id);
    if(!goal_handle->is_active()){
      pubZeroVelocity();
      RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior preempted.");
      m_recovery_result = dddmr_sys_core::RecoveryState::INTERRUPT_BY_NEW_GOAL;
      break;
    }
    if(goal_handle->is_canceling()){
      pubZeroVelocity();
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior cancelled.");
      m_recovery_result = dddmr_sys_core::RecoveryState::INTERRUPT_BY_CANCEL;
      break;
      
    }

    // Update Current Angle
    std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(perception_3d_ros_->getStackedPerception()->getMutex()));

    //@ update current observation for scoring
    //@ we need to visualized this for debug/justification
    perception_3d_ros_->getStackedPerception()->aggregateObservations();
    //pub_aggregate_observation_.publish(aggregate_observation_);

    perception_3d_ros_->getGlobalPose(trans_gbl2b);
    trans2Pose(trans_gbl2b, global_pose);

    tf2::Quaternion global_pose_orientation(
    global_pose.pose.orientation.x, 
    global_pose.pose.orientation.y, 
    global_pose.pose.orientation.z, 
    global_pose.pose.orientation.w);

    current_angle = tf2::impl::getYaw(global_pose_orientation);

    // compute the distance left to rotate
    double dist_left;
    if (!got_180)
    {
      // If we haven't hit 180 yet, we need to rotate a half circle plus the distance to the 180 point
      double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
      dist_left = M_PI + distance_to_180;

      if (distance_to_180 < tolerance_)
      {
        got_180 = true;
      }
    }
    else
    {
      // If we have hit the 180, we just have the distance back to the start
      dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
    }

    //Do not create a function to set the parameters unless a nice structure is found
    //Below assignment of variables is useful when migrate to ROS2
    trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
    trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = shared_data_->robot_state_;
    trajectory_generators_ros_->initializeTheories_wi_Shared_data();

    geometry_msgs::msg::PoseArray pose_arr;
    pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

    trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

    //@ We queue all trajectories in trajectories_, then score them one by one in getBestTrajectory()
    while(trajectory_generators_ros_->hasMoreTrajectories(trajectory_generator_name_)){
      base_trajectory::Trajectory a_traj;
      if(trajectory_generators_ros_->nextTrajectory(trajectory_generator_name_, a_traj)){
        //@ collected all trajectories here, for later scoring
        trajectories_->push_back(a_traj);
        trajectory2posearray_cuboids(a_traj, pose_arr, cuboids_pcl);
      }

    }

    pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    pose_arr.header.stamp = node_->get_clock()->now();
    pub_trajectory_pose_array_->publish(pose_arr);

    /*Update data for critics*/
    std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(*(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
    //@ unless we come up with a better strcuture
    //@ keep below for easy migration for ROS2
    mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
    mpc_critics_ros_->getSharedDataPtr()->robot_state_ = shared_data_->robot_state_;
    mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = perception_3d_ros_->getSharedDataPtr()->aggregate_observation_;
    //@ Below function transform prune_plane from nav::msg to pcl type
    //@ Below function put new perception in kdtree for critics to avoid obstacles
    mpc_critics_ros_->updateSharedData();
    base_trajectory::Trajectory best_traj;
    getBestTrajectory(trajectory_generator_name_, best_traj);

    //@ Reset kd tree/observations because it is shared_ptr and copied from perception_ros
    mpc_critics_ros_->getSharedDataPtr()->pcl_perception_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    if(got_180){
      pubZeroVelocity();
      goal_handle->succeed(result);
      RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior succeed.");
      m_recovery_result = dddmr_sys_core::RecoveryState::RECOVERY_DONE;
      break;
    }

    if(best_traj.cost_<0){
      RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *clock_, 5000,"%s: All trajectories are rejected by critics.", name_.c_str());
      auto valid_time = clock_->now() - last_valid_control_;
      if( valid_time.seconds() > 5.0){
        pubZeroVelocity();
        goal_handle->abort(result);
        RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior abort.");
        m_recovery_result = dddmr_sys_core::RecoveryState::RECOVERY_FAIL;
        break;
      }
      pubZeroVelocity();    
    }
    else{
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = best_traj.xv_;
      cmd_vel.angular.z = best_traj.thetav_;
      cmd_vel_pub_->publish(cmd_vel);
      last_valid_control_ = clock_->now();
    }
    
    r.sleep();
  }
  
  return m_recovery_result;
}

}//end of name space