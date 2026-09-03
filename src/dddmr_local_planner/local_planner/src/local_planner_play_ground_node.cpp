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


#include <memory>
#include <perception_3d/perception_3d_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <local_planner/local_planner.h>
#include "rclcpp/rclcpp.hpp"

namespace local_planner {

class PlayGround : public rclcpp::Node
{
public:
  PlayGround(std::string name);
  void initial(
    const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
    const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
    const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators);

private:

  std::string name_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
  std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
  std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;
  std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;

  visualization_msgs::msg::MarkerArray robot_cuboid_;
  visualization_msgs::msg::Marker marker_edge_;
  
  //@ pub and sub
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_prune_plan_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_best_trajectory_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_accepted_trajectory_pose_array_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_robot_cuboid_;

  //@ function
  void parseCuboid();
  void cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal);
  void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl);
  void getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);
}; 

PlayGround::PlayGround(std::string name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();

  pub_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("all_trajectories", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&PlayGround::cbClickedPoint, this, std::placeholders::_1));
  
  pub_prune_plan_ = this->create_publisher<nav_msgs::msg::Path>("prune_plan", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_best_trajectory_pose_ = this->create_publisher<geometry_msgs::msg::PoseArray>("best_trajectory", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_accepted_trajectory_pose_array_ = this->create_publisher<geometry_msgs::msg::PoseArray>("accepted_trajectories", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_robot_cuboid_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_cuboid", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void PlayGround::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d,
                          const std::shared_ptr<mpc_critics::MPC_Critics_ROS>& mpc_critics,
                            const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>& trajectory_generators){
  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;
  parseCuboid();  
}

void PlayGround::parseCuboid(){
  marker_edge_.header.frame_id = perception_3d_ros_->getGlobalUtils()->getRobotFrame();;
  marker_edge_.header.stamp = clock_->now();
  marker_edge_.action = visualization_msgs::msg::Marker::ADD;
  marker_edge_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge_.pose.orientation.w = 1.0;
  marker_edge_.ns = "edges";
  marker_edge_.id = 3; marker_edge_.scale.x = 0.03;
  marker_edge_.color.r = 0.9; marker_edge_.color.g = 1; marker_edge_.color.b = 0; marker_edge_.color.a = 0.8;
  //@ parse cuboid, currently the cuboid in local planner is just for visualization
  RCLCPP_INFO(this->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<std::string> cuboid_vertex_queue = {"cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt", "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  std::map<std::string, std::vector<double>> cuboid_vertex_parameter_map;

  for(auto it=cuboid_vertex_queue.begin(); it!=cuboid_vertex_queue.end();it++){
    std::vector<double> p;
    geometry_msgs::msg::Point pt;
    this->declare_parameter(*it, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param= this->get_parameter(*it);
    p = cuboid_param.as_double_array();
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
    cuboid_vertex_parameter_map[*it] = p;
  }
  RCLCPP_INFO(this->get_logger().get_child(name_), "Cuboid vertex are loaded, start to connect edges.");
  std::vector<std::string> cuboid_vertex_connect = {"cuboid.flb", "cuboid.blb", "cuboid.flt", "cuboid.blt", "cuboid.frb", "cuboid.brb", "cuboid.frt", "cuboid.brt",
                                                      "cuboid.flt", "cuboid.flb", "cuboid.frt", "cuboid.frb", "cuboid.blt", "cuboid.blb", "cuboid.brt", "cuboid.brb"};
  for(auto it=cuboid_vertex_connect.begin(); it!=cuboid_vertex_connect.end();it++){
    auto p = cuboid_vertex_parameter_map[*it];
    geometry_msgs::msg::Point pt;
    pt.x = p[0];pt.y = p[1];pt.z = p[2];
    marker_edge_.points.push_back(pt);
  }
}

void PlayGround::trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr,
                                      pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl){

  for(unsigned int i=0;i<a_traj.getPosesSize();i++){
      auto p = a_traj.getPose(i);
      pose_arr.poses.push_back(p.pose);
      //@ For cuboids debug
      //cuboids_pcl += a_traj.getCuboid(i);       
  }

}

void PlayGround::getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj){

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
  accepted_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  accepted_pose_arr.header.stamp = clock_->now();
  pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);

  geometry_msgs::msg::PoseArray best_pose_arr;
  trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
  best_pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  best_pose_arr.header.stamp = clock_->now();
  pub_best_trajectory_pose_->publish(best_pose_arr);

}

void PlayGround::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  RCLCPP_INFO(this->get_logger(), "Got clicked point: %.2f, %.2f", clicked_goal->point.x, clicked_goal->point.y);
  
  //@ publish robot cuboid
  robot_cuboid_.markers.clear();
  marker_edge_.header.stamp = clock_->now();
  robot_cuboid_.markers.push_back(marker_edge_);
  pub_robot_cuboid_->publish(robot_cuboid_);

  /*
  * Generate necessary information for trajectory usage
  */
  geometry_msgs::msg::TransformStamped trans_gbl2b;
  nav_msgs::msg::Odometry robot_state;
  nav_msgs::msg::Path prune_plan;

  trans_gbl2b.transform.translation.x = 0.0;
  trans_gbl2b.transform.translation.y = 0.0;
  trans_gbl2b.transform.translation.z = 0.0;
  trans_gbl2b.transform.rotation.x = 0.0;
  trans_gbl2b.transform.rotation.y = 0.0;
  trans_gbl2b.transform.rotation.z = 0.0;
  trans_gbl2b.transform.rotation.w = 1.0;
  
  //simulate the robot speed is 0.3 m/s, and no angular speed. Change these two values to see differences of trajectories because of model predictive control
  robot_state.twist.twist.linear.x = 0.4;
  robot_state.twist.twist.angular.z = 0.0;
  
  //simulate a prune plan (which is usually come from pruned global plan)
  prune_plan.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan.header.stamp = clock_->now();
  double dx = clicked_goal->point.x/20;
  double dy = clicked_goal->point.y/20;
  for(int i=0;i<20;i++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    pst.pose.position.x = dx * i;
    pst.pose.position.y = dy * i;
    prune_plan.poses.push_back(pst);
  }
  pub_prune_plan_->publish(prune_plan);

  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan;
  //@ change max speed from perception shared data framework
  trajectory_generators_ros_->getSharedDataPtr()->current_allowed_max_linear_speed_ = -1.0;//set this value to negative to ignore it

  trajectory_generators_ros_->initializeTheories_wi_Shared_data();

  geometry_msgs::msg::PoseArray pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;

  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  //@ Generate trajectory by differential_drive_simple which is defined in config ---> local_planner_play_ground.yaml line: 63-64
  while(trajectory_generators_ros_->hasMoreTrajectories("differential_drive_simple")){
    base_trajectory::Trajectory a_traj;
    if(trajectory_generators_ros_->nextTrajectory("differential_drive_simple", a_traj)){
      //@ collected all trajectories here, for later scoring
      trajectories_->push_back(a_traj);
      trajectory2posearray_cuboids(a_traj, pose_arr, cuboids_pcl);
    }

  }

  pose_arr.header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pose_arr.header.stamp = clock_->now();
  pub_trajectory_pose_array_->publish(pose_arr);
  
  /*
  ~ Above code section complete trajectories generation, now lets pass them to critics to score them
  */

  //@Update data for critics
  std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t> critics_lock(*(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
  //@ unless we come up with a better strcuture
  //@ keep below for easy migration for ROS2
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state;


  //@ Create a point cloud block in front of robot, so some trajectories will be reject due to collision model --> local_planner_play_ground.yaml line: 98-99
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle;
  obstacle.reset(new pcl::PointCloud<pcl::PointXYZI>());
  obstacle->header.frame_id = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pcl::PointXYZI pt1, pt2, pt3, pt4, pt5;
  pt1.x = 0.80; pt1.y = 0.60; pt1.z = 0.2;
  pt2.x = 0.75; pt2.y = 0.65; pt2.z = 0.2;
  pt3.x = 0.85; pt3.y = 0.55; pt3.z = 0.2;
  pt4.x = 0.70; pt4.y = 0.70; pt4.z = 0.2;
  pt5.x = 0.90; pt5.y = 0.50; pt5.z = 0.2;
  //@ Need more than 5 point for kd-tree to generate, see: void updateData() in model_shared_data.h
  obstacle->push_back(pt1);obstacle->push_back(pt2);obstacle->push_back(pt3);obstacle->push_back(pt4);obstacle->push_back(pt5);
  sensor_msgs::msg::PointCloud2 ros_msg_obstacle;
  pcl::toROSMsg(*obstacle, ros_msg_obstacle);
  pub_obstacle_->publish(ros_msg_obstacle);

  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = obstacle;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan;
  //@ Below function transform prune_plane from nav::msg to pcl type
  //@ Below function generate kd-tree using aggregate observation
  mpc_critics_ros_->updateSharedData();
  base_trajectory::Trajectory best_traj;
  getBestTrajectory("differential_drive_simple", best_traj);

  //@ Reset kd tree/observations because it is shared_ptr and copied from perception_ros
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

}
}//ns

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
  auto node_lppg = std::make_shared<local_planner::PlayGround>("local_planner_play_ground");
 
  executor.add_node(node_tg);
  executor.add_node(node_mc);
  executor.add_node(node_p3);
  executor.add_node(node_lppg);

  node_tg->initial();
  node_mc->initial();
  node_p3->initial();
  node_lppg->initial(node_p3, node_mc, node_tg);
  executor.spin();


  rclcpp::shutdown();

}
