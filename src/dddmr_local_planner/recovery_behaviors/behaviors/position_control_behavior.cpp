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
#include <recovery_behaviors/position_control_behavior.h>

PLUGINLIB_EXPORT_CLASS(recovery_behaviors::PositionControlBehavior, recovery_behaviors::RobotBehavior)

namespace recovery_behaviors
{

PositionControlBehavior::PositionControlBehavior(){
  return;
  
}

PositionControlBehavior::~PositionControlBehavior(){
  trajectories_.reset();
}

void PositionControlBehavior::onInitialize(){

  node_->declare_parameter(name_ + ".frequency", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".frequency", frequency_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "frequency: %.2f", frequency_);

  node_->declare_parameter(name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".yaw_tolerance", yaw_tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "yaw_tolerance %.2f", yaw_tolerance_);

  node_->declare_parameter(name_ + ".distance_tolerance", rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".distance_tolerance", distance_tolerance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "distance_tolerance %.2f", distance_tolerance_);

  node_->declare_parameter(name_ + ".trajectory_generator_name", rclcpp::ParameterValue("one_state_trajectory"));
  node_->get_parameter(name_ + ".trajectory_generator_name", trajectory_generator_name_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "trajectory_generator_name: %s", trajectory_generator_name_.c_str());  


  node_->declare_parameter(name_ + ".kp_yaw", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".kp_yaw", kp_yaw_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "kp_yaw: %.3f", kp_yaw_);  

  node_->declare_parameter(name_ + ".ki_yaw", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".ki_yaw", ki_yaw_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "ki_yaw: %.3f", ki_yaw_);  

  node_->declare_parameter(name_ + ".kd_yaw", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".kd_yaw", kd_yaw_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "kd_yaw: %.3f", kd_yaw_);  


  node_->declare_parameter(name_ + ".kp_distance", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".kp_distance", kp_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "kp_distance: %.3f", kp_distance_);  

  node_->declare_parameter(name_ + ".ki_distance", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".ki_distance", ki_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "ki_distance: %.3f", ki_distance_);  

  node_->declare_parameter(name_ + ".kd_distance", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".kd_distance", kd_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "kd_distance: %.3f", kd_distance_);  

  clock_ = node_->get_clock();

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pub_trajectory_pose_array_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(name_ + "_trajectory", 1);
}

void PositionControlBehavior::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr, pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl){

  for(unsigned int i=0;i<a_traj.getPointsSize();i++){
      auto p = a_traj.getPoint(i);
      pose_arr.poses.push_back(p.pose);  
      //@ For cuboids debug
      //cuboids_pcl += a_traj.getCuboid(i);   
  }

}

void PositionControlBehavior::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

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

void PositionControlBehavior::trans2Pose(geometry_msgs::msg::TransformStamped& trans, geometry_msgs::msg::PoseStamped& pose){
  pose.header = trans.header;
  pose.pose.position.x = trans.transform.translation.x;
  pose.pose.position.y = trans.transform.translation.y;
  pose.pose.position.z = trans.transform.translation.z;
  pose.pose.orientation.x = trans.transform.rotation.x;
  pose.pose.orientation.y = trans.transform.rotation.y;
  pose.pose.orientation.z = trans.transform.rotation.z;
  pose.pose.orientation.w = trans.transform.rotation.w;
}

void PositionControlBehavior::pubZeroVelocity(){
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
}

double PositionControlBehavior::getYaw(geometry_msgs::msg::PoseStamped& pose){
  
  tf2::Quaternion q_pose(
    pose.pose.orientation.x, 
    pose.pose.orientation.y, 
    pose.pose.orientation.z, 
    pose.pose.orientation.w);

  return tf2::impl::getYaw(q_pose);
}

void PositionControlBehavior::getDiffFromCurrentPoseToTargetPosition(
            geometry_msgs::msg::PoseStamped& robot_pose, 
            geometry_msgs::msg::PoseStamped& target_pose,
            double& yaw_diff, double& distance_diff){

  //@ Calculate yaw based on two positions (not pose) in global frame
  double vx = target_pose.pose.position.x - robot_pose.pose.position.x;
  double vy = target_pose.pose.position.y - robot_pose.pose.position.y;
  double vz = target_pose.pose.position.z - robot_pose.pose.position.z;
  tf2::Quaternion q;
  if(vz!=0){
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);
    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q_pre(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q_pre.normalize();
    q = q_pre;
  }
  else{
    //@ handle with 2D
    double yaw = atan2(vy, vx);
    tf2::Quaternion q_pre;
    q_pre.setRPY(0.0, 0.0, yaw);
    q = q_pre;
  }

  tf2::Stamped<tf2::Transform> tf2_current_position_2_target_position;
  //@Transform last pose to tf2 type
  //tf2::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW())
  tf2_current_position_2_target_position.setRotation(q);
  tf2_current_position_2_target_position.setOrigin(tf2::Vector3(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z));
  tf2::Stamped<tf2::Transform> tf2_trans_pin_pose;
  rosMsg2Tf2Msg(robot_pose, tf2_trans_pin_pose);
  getYawAndDistanceDiffFromTwoPoses(tf2_trans_pin_pose, tf2_current_position_2_target_position, yaw_diff, distance_diff);

}

void PositionControlBehavior::rosMsg2Tf2Msg(const geometry_msgs::msg::PoseStamped& ros_pose, tf2::Stamped<tf2::Transform>& tf2_pose){
  tf2_pose.setOrigin(tf2::Vector3(ros_pose.pose.position.x, ros_pose.pose.position.y, ros_pose.pose.position.z));
  tf2_pose.setRotation(tf2::Quaternion(ros_pose.pose.orientation.x, ros_pose.pose.orientation.y, ros_pose.pose.orientation.z, ros_pose.pose.orientation.w));
}

void PositionControlBehavior::getYawAndDistanceDiffFromTwoPoses(
            tf2::Stamped<tf2::Transform>& tf2_trans_pin_pose, 
            tf2::Stamped<tf2::Transform>& tf2_trans_target_pose,
            double& yaw_diff, double& distance_diff){

  auto tf2_trans_pin_pose_inverse = tf2_trans_pin_pose.inverse();
  //@Get baselink to last pose
  tf2::Transform tf2_pin2targetpose;
  tf2_pin2targetpose.mult(tf2_trans_pin_pose_inverse, tf2_trans_target_pose);
  //@Get RPY
  tf2::Matrix3x3 m(tf2_pin2targetpose.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //@Although the test shows that yaw is already the shortest, we will use shortest_angular_distance anyway.
  yaw_diff = angles::shortest_angular_distance(0.0, yaw);
  distance_diff = tf2_pin2targetpose.getOrigin().x();

}

void PositionControlBehavior::generateVelocity(double vx, double vy, double angular_z){
  cmd_vel_stamped_.header.stamp = clock_->now();
  cmd_vel_stamped_.twist.linear.x = vx;
  cmd_vel_stamped_.twist.linear.y = vy;
  cmd_vel_stamped_.twist.angular.z = angular_z;
}

dddmr_sys_core::RecoveryState PositionControlBehavior::runBehavior(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle){

  //@Print thread for debug
  std::stringstream ss;
  ss << std::this_thread::get_id();
  uint64_t id = std::stoull(ss.str());
  //RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior started, thread ID : %lu", id);
  
  auto target_pose = goal_handle->get_goal()->target_pose;

  rclcpp::Rate r(frequency_);

  geometry_msgs::msg::PoseStamped global_pose;
  geometry_msgs::msg::TransformStamped trans_gbl2b;
  perception_3d_ros_->getGlobalPose(trans_gbl2b);
  trans2Pose(trans_gbl2b, global_pose);
  double yaw_diff, distance_diff; 
  getDiffFromCurrentPoseToTargetPosition(global_pose, target_pose, yaw_diff, distance_diff);
  
  rclcpp::Time last_valid_control_ = clock_->now();
  
  dddmr_sys_core::RecoveryState m_recovery_result;
  auto result = std::make_shared<dddmr_sys_core::action::RecoveryBehaviors::Result>();

  //@ initial state to align
  fsm_state_ = recovery_behaviors::EnumFSMStatePositionControl::POINTING_ORIENTATION_TO_TARGET;
  yaw_error_ = 0.0;
  yaw_error_i_ = 0.0;
  distance_error_ = 0.0;
  distance_error_i_ = 0.0;
  last_yaw_diff_ = 0.0;
  last_distance_diff_ = 0.0;
  double dt = 1.0/frequency_;

  yaw_converge_count_ = 0;
  distance_converge_count_ = 0;
  yaw_goal_converge_count_ = 0;

  while (rclcpp::ok())
  {

    RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *clock_, 5000, "Behavior is running with thread ID : %lu", id);
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

    // Update Current Diff
    std::unique_lock<perception_3d::StackedPerception::mutex_t> pct_lock(*(perception_3d_ros_->getStackedPerception()->getMutex()));
    perception_3d_ros_->getStackedPerception()->aggregateObservations();
    //@ implement time out

    perception_3d_ros_->getGlobalPose(trans_gbl2b);
    trans2Pose(trans_gbl2b, global_pose);
    getDiffFromCurrentPoseToTargetPosition(global_pose, target_pose, yaw_diff, distance_diff);
    RCLCPP_DEBUG(node_->get_logger(), "Yaw diff: %.2f, distance_diff: %.2f", yaw_diff, distance_diff);

    if(fsm_state_ == recovery_behaviors::EnumFSMStatePositionControl::POINTING_ORIENTATION_TO_TARGET){
      double pid_yaw = 0.0;
      if(fabs(yaw_diff)>0.785){
        pid_yaw = kp_yaw_ * yaw_diff;
      }
      else{
        pid_yaw = kp_yaw_ * yaw_diff + ki_yaw_ * yaw_error_i_ + kd_yaw_ * (yaw_diff - last_yaw_diff_);
      }
      
      if(pid_yaw>=0){
        pid_yaw = std::min(pid_yaw, 0.3);
      }
      else{
        pid_yaw = std::max(pid_yaw, -0.3);
      }
      
      generateVelocity(0, 0, pid_yaw);

      if(fabs(yaw_diff)>0.785){
      }
      else{
        yaw_error_i_ += yaw_diff*dt;
      }
 
      last_yaw_diff_ = yaw_diff;

      if(fabs(yaw_diff)<yaw_tolerance_){
        yaw_converge_count_++;
        if(yaw_converge_count_>10){
          fsm_state_ = recovery_behaviors::EnumFSMStatePositionControl::STRAIGHT;
          RCLCPP_INFO(node_->get_logger(), "Yaw aligned, go to Straight state.");
          pubZeroVelocity();
        }
      }

    }
    else if(fsm_state_ == recovery_behaviors::EnumFSMStatePositionControl::STRAIGHT){
      
      double pid_distance = 0.0;
      if(fabs(ki_distance_>=0.3)){
        pid_distance = kp_distance_ * distance_diff;
      }
      else{
        pid_distance = kp_distance_ * distance_diff + ki_distance_ * distance_error_i_ + kd_distance_ * (distance_diff - last_distance_diff_);
      }
      
      //@ cap max
      if(pid_distance>=0){
        pid_distance = std::min(pid_distance, 0.1);
      }
      else{
        pid_distance = std::max(pid_distance, -0.1);
      }
      
      //@ increase min
      if(pid_distance>=0){
        pid_distance = std::max(pid_distance, 0.05);
      }
      else{
        pid_distance = std::min(pid_distance, -0.05);
      }

      generateVelocity(pid_distance, 0, 0);

      if(fabs(ki_distance_>=0.3)){
      }
      else{
        distance_error_i_ += distance_diff*dt;
      }

      distance_error_i_ += distance_diff*dt;
      last_distance_diff_ = distance_diff;

      if(fabs(distance_diff)<distance_tolerance_){
        distance_converge_count_++;
        if(distance_converge_count_>10){
          fsm_state_ = recovery_behaviors::EnumFSMStatePositionControl::ALIGN_ORIENTATION_FROM_GOAL;
          RCLCPP_INFO(node_->get_logger(), "Distance aligned.");
          yaw_error_i_ = 0.0;
          last_yaw_diff_ = 0.0;
          pubZeroVelocity();
        }
      }

    }

    else if(fsm_state_ == recovery_behaviors::EnumFSMStatePositionControl::ALIGN_ORIENTATION_FROM_GOAL){
      tf2::Stamped<tf2::Transform> tf2_trans_robot_pose;
      rosMsg2Tf2Msg(global_pose, tf2_trans_robot_pose);
      tf2::Stamped<tf2::Transform> tf2_trans_target_pose;
      rosMsg2Tf2Msg(target_pose, tf2_trans_target_pose);
      getYawAndDistanceDiffFromTwoPoses(tf2_trans_robot_pose, tf2_trans_target_pose, yaw_diff, distance_diff);

      double pid_yaw = 0.0;
      if(fabs(yaw_diff)>0.785){
        pid_yaw = kp_yaw_ * yaw_diff;
      }
      else{
        pid_yaw = kp_yaw_ * yaw_diff + ki_yaw_ * yaw_error_i_ + kd_yaw_ * (yaw_diff - last_yaw_diff_);
      }
      
      if(pid_yaw>=0){
        pid_yaw = std::min(pid_yaw, 0.3);
      }
      else{
        pid_yaw = std::max(pid_yaw, -0.3);
      }
      
      generateVelocity(0, 0, pid_yaw);

      if(fabs(yaw_diff)>0.785){
      }
      else{
        yaw_error_i_ += yaw_diff*dt;
      }

      last_yaw_diff_ = yaw_diff;

      if(fabs(yaw_diff)<yaw_tolerance_){
        yaw_goal_converge_count_++;
        if(yaw_goal_converge_count_>10){
          goal_handle->succeed(result);
          RCLCPP_INFO(node_->get_logger().get_child(name_), "Behavior succeed.");
          m_recovery_result = dddmr_sys_core::RecoveryState::RECOVERY_DONE;
          pubZeroVelocity();
          break;
        }
      }

    }
    

    //Do not create a function to set the parameters unless a nice structure is found
    //Below assignment of variables is useful when migrate to ROS2
    trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
    trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = shared_data_->robot_state_;
    trajectory_generators_ros_->getSharedDataPtr()->ref_twist_for_trajectory_generation_ = cmd_vel_stamped_;
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

    if(best_traj.cost_<0){
      RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *clock_, 5000,"%s: All trajectories are rejected by critics.", name_.c_str());
      auto valid_time = clock_->now() - last_valid_control_;
      if(valid_time.seconds() > 5.0){
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