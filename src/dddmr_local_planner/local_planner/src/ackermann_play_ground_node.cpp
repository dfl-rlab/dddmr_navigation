/*
* BSD 3-Clause License
* Copyright (c) 2024, DDDMobileRobot
*/

/*
 * Ackermann PlayGround Node
 * Features:
 *   - sim_speed / sim_steering_angle / obstacle from YAML
 *   - Footprint rectangles (white) at sampled poses
 *   - Steering angle lines (blue) at sampled poses
 *   - Display ratio: head, middle, tail guaranteed
 */

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <local_planner/local_planner.h>
#include <memory>
#include <mpc_critics/mpc_critics_ros.h>
#include <perception_3d/perception_3d_ros.h>
#include <set>
#include <trajectory_generators/trajectory_generators_ros.h>

namespace local_planner {

class AckermannPlayGround : public rclcpp::Node {
public:
  AckermannPlayGround(std::string name);
  void initial(
      const std::shared_ptr<perception_3d::Perception3D_ROS> &perception_3d,
      const std::shared_ptr<mpc_critics::MPC_Critics_ROS> &mpc_critics,
      const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>
          &trajectory_generators);

private:
  std::string name_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
  std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
  std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>
      trajectory_generators_ros_;
  std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;

  visualization_msgs::msg::MarkerArray robot_cuboid_;
  visualization_msgs::msg::Marker marker_edge_;

  //@ YAML-parameterized values
  double sim_speed_;
  double sim_steering_angle_;
  double footprint_display_ratio_;  // 0.0~1.0, e.g. 0.05 = 5%
  double footprint_half_length_;    // from cuboid flb.x
  double footprint_half_width_;     // from cuboid flb.y
  double wheelbase_;                // for steering line
  std::vector<double> obstacle_x_;
  std::vector<double> obstacle_y_;
  std::vector<double> obstacle_z_;

  //@ pub and sub
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      pub_trajectory_pose_array_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_prune_plan_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      pub_best_trajectory_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      pub_accepted_trajectory_pose_array_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      pub_rejected_trajectory_pose_array_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_robot_cuboid_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_footprint_markers_;

  //@ function
  void parseCuboid();
  void cbClickedPoint(
      const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal);
  void
  trajectory2posearray_cuboids(const base_trajectory::Trajectory &a_traj,
                               geometry_msgs::msg::PoseArray &pose_arr,
                               pcl::PointCloud<pcl::PointXYZ> &cuboids_pcl);
  void getBestTrajectory(std::string traj_gen_name,
                         base_trajectory::Trajectory &best_traj);

  //@ Build set of indices to display for a trajectory with n points
  std::set<int> getDisplayIndices(int n);

  //@ Add footprint rectangle (white) + steering line (blue) at a pose
  void addFootprintMarker(visualization_msgs::msg::MarkerArray &ma,
                          int &marker_id,
                          const geometry_msgs::msg::Pose &pose,
                          double steering_angle,
                          const std::string &frame_id);
};

AckermannPlayGround::AckermannPlayGround(std::string name) : Node(name) {
  name_ = name;
  clock_ = this->get_clock();

  //@ Declare YAML-parameterized simulation inputs
  this->declare_parameter("sim_speed", 0.5);
  this->get_parameter("sim_speed", sim_speed_);

  this->declare_parameter("sim_steering_angle", 0.0);
  this->get_parameter("sim_steering_angle", sim_steering_angle_);

  this->declare_parameter("footprint_display_ratio", 0.05);
  this->get_parameter("footprint_display_ratio", footprint_display_ratio_);

  this->declare_parameter("obstacle_x", std::vector<double>{0.8, 0.75, 0.85, 0.7, 0.9});
  this->get_parameter("obstacle_x", obstacle_x_);

  this->declare_parameter("obstacle_y", std::vector<double>{0.6, 0.65, 0.55, 0.7, 0.5});
  this->get_parameter("obstacle_y", obstacle_y_);

  this->declare_parameter("obstacle_z", std::vector<double>{0.2, 0.2, 0.2, 0.2, 0.2});
  this->get_parameter("obstacle_z", obstacle_z_);

  pub_trajectory_pose_array_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
          "all_trajectories",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  clicked_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "clicked_point", 1,
          std::bind(&AckermannPlayGround::cbClickedPoint, this,
                    std::placeholders::_1));

  pub_prune_plan_ = this->create_publisher<nav_msgs::msg::Path>(
      "prune_plan",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_best_trajectory_pose_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
          "best_trajectory",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_accepted_trajectory_pose_array_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
          "accepted_trajectories",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_rejected_trajectory_pose_array_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>(
          "rejected_trajectories",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_robot_cuboid_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "robot_cuboid",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_footprint_markers_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "trajectory_footprints",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void AckermannPlayGround::initial(
    const std::shared_ptr<perception_3d::Perception3D_ROS> &perception_3d,
    const std::shared_ptr<mpc_critics::MPC_Critics_ROS> &mpc_critics,
    const std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS>
        &trajectory_generators) {
  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;
  parseCuboid();

  //@ Read footprint half-size from cuboid params (flb = front-left-bottom)
  rclcpp::Parameter cuboid_flb = this->get_parameter("cuboid.flb");
  auto flb = cuboid_flb.as_double_array();
  footprint_half_length_ = std::fabs(flb[0]);
  footprint_half_width_ = std::fabs(flb[1]);

  //@ Read wheelbase from trajectory generator params for steering line
  //@ We just use the same value declared in YAML
  this->declare_parameter("vis_wheelbase", 0.3);
  this->get_parameter("vis_wheelbase", wheelbase_);

  RCLCPP_INFO(this->get_logger(),
              "Footprint: half_L=%.3f half_W=%.3f wheelbase=%.3f ratio=%.2f",
              footprint_half_length_, footprint_half_width_, wheelbase_,
              footprint_display_ratio_);
}

void AckermannPlayGround::parseCuboid() {
  marker_edge_.header.frame_id =
      perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  marker_edge_.header.stamp = clock_->now();
  marker_edge_.action = visualization_msgs::msg::Marker::ADD;
  marker_edge_.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_edge_.pose.orientation.w = 1.0;
  marker_edge_.ns = "edges";
  marker_edge_.id = 3;
  marker_edge_.scale.x = 0.03;
  marker_edge_.color.r = 0.9;
  marker_edge_.color.g = 1;
  marker_edge_.color.b = 0;
  marker_edge_.color.a = 0.8;
  RCLCPP_INFO(this->get_logger().get_child(name_), "Start to parse cuboid.");
  std::vector<std::string> cuboid_vertex_queue = {
      "cuboid.flb", "cuboid.frb", "cuboid.flt", "cuboid.frt",
      "cuboid.blb", "cuboid.brb", "cuboid.blt", "cuboid.brt"};
  std::map<std::string, std::vector<double>> cuboid_vertex_parameter_map;
  for (auto it = cuboid_vertex_queue.begin(); it != cuboid_vertex_queue.end();
       it++) {
    std::vector<double> p;
    geometry_msgs::msg::Point pt;
    this->declare_parameter(*it, rclcpp::PARAMETER_DOUBLE_ARRAY);
    rclcpp::Parameter cuboid_param = this->get_parameter(*it);
    p = cuboid_param.as_double_array();
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
    marker_edge_.points.push_back(pt);
    cuboid_vertex_parameter_map[*it] = p;
  }
  RCLCPP_INFO(this->get_logger().get_child(name_),
              "Cuboid vertex are loaded, start to connect edges.");
  std::vector<std::string> cuboid_vertex_connect = {
      "cuboid.flb", "cuboid.blb", "cuboid.flt", "cuboid.blt",
      "cuboid.frb", "cuboid.brb", "cuboid.frt", "cuboid.brt",
      "cuboid.flt", "cuboid.flb", "cuboid.frt", "cuboid.frb",
      "cuboid.blt", "cuboid.blb", "cuboid.brt", "cuboid.brb"};
  for (auto it = cuboid_vertex_connect.begin();
       it != cuboid_vertex_connect.end(); it++) {
    auto p = cuboid_vertex_parameter_map[*it];
    geometry_msgs::msg::Point pt;
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
    marker_edge_.points.push_back(pt);
  }
}

std::set<int> AckermannPlayGround::getDisplayIndices(int n) {
  std::set<int> indices;
  if (n <= 0) return indices;

  //@ Always include first, middle, last
  indices.insert(0);
  indices.insert(n / 2);
  indices.insert(n - 1);

  //@ Add extra poses based on ratio
  int display_count = std::max(3, (int)std::ceil(n * footprint_display_ratio_));
  if (display_count >= n) {
    for (int i = 0; i < n; i++) indices.insert(i);
  } else {
    for (int k = 0; k < display_count; k++) {
      int idx = (int)std::round((double)k / (display_count - 1) * (n - 1));
      indices.insert(idx);
    }
  }
  return indices;
}

void AckermannPlayGround::addFootprintMarker(
    visualization_msgs::msg::MarkerArray &ma, int &marker_id,
    const geometry_msgs::msg::Pose &pose, double steering_angle,
    const std::string &frame_id) {

  //@ Extract yaw from quaternion
  double siny = 2.0 * (pose.orientation.w * pose.orientation.z +
                        pose.orientation.x * pose.orientation.y);
  double cosy = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                               pose.orientation.z * pose.orientation.z);
  double yaw = std::atan2(siny, cosy);
  double cx = pose.position.x;
  double cy = pose.position.y;
  double cos_y = std::cos(yaw);
  double sin_y = std::sin(yaw);

  double hl = footprint_half_length_;
  double hw = footprint_half_width_;

  //@ ---- Footprint rectangle (white) ----
  // FL, FR, BR, BL in body frame
  double corners_x[4] = { hl,  hl, -hl, -hl};
  double corners_y[4] = { hw, -hw, -hw,  hw};
  geometry_msgs::msg::Point pts[4];
  for (int i = 0; i < 4; i++) {
    pts[i].x = cx + corners_x[i] * cos_y - corners_y[i] * sin_y;
    pts[i].y = cy + corners_x[i] * sin_y + corners_y[i] * cos_y;
    pts[i].z = 0.02;
  }

  visualization_msgs::msg::Marker fp;
  fp.header.frame_id = frame_id;
  fp.header.stamp = clock_->now();
  fp.ns = "footprint";
  fp.id = marker_id++;
  fp.type = visualization_msgs::msg::Marker::LINE_LIST;
  fp.action = visualization_msgs::msg::Marker::ADD;
  fp.pose.orientation.w = 1.0;
  fp.scale.x = 0.005;
  fp.color.r = 1.0; fp.color.g = 1.0; fp.color.b = 1.0; fp.color.a = 0.6;
  fp.lifetime = rclcpp::Duration(0, 0);
  for (int i = 0; i < 4; i++) {
    fp.points.push_back(pts[i]);
    fp.points.push_back(pts[(i + 1) % 4]);
  }
  ma.markers.push_back(fp);

  //@ ---- 4 Tires ----
  //@ Tire = short thick line centered at wheel position
  //@ Front axle at x = +wheelbase from base_link (rear axle)
  //@ Rear axle at x = 0 (base_link = rear axle center)
  //@ Track width ≈ footprint width
  double tire_half_len = hw * 0.6;  // tire visual length (proportional to car width)
  double front_axle_x = hl;          // front tires at footprint front corners
  double rear_axle_x = 0.0;         // rear axle at base_link origin

  //@ Helper: transform body-frame point to global
  auto toGlobal = [&](double bx, double by) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = cx + bx * cos_y - by * sin_y;
    p.y = cy + bx * sin_y + by * cos_y;
    p.z = 0.03;
    return p;
  };

  //@ Helper: add one tire line segment
  auto addTire = [&](double wx, double wy, double tire_yaw,
                     float r, float g, float b, const std::string &ns_name) {
    double ct = std::cos(tire_yaw);
    double st = std::sin(tire_yaw);
    // Tire endpoints in body frame
    double dx = tire_half_len * ct;
    double dy = tire_half_len * st;
    // Body-frame endpoints
    double bx1 = wx - dx, by1 = wy - dy;
    double bx2 = wx + dx, by2 = wy + dy;

    visualization_msgs::msg::Marker tire;
    tire.header.frame_id = frame_id;
    tire.header.stamp = clock_->now();
    tire.ns = ns_name;
    tire.id = marker_id++;
    tire.type = visualization_msgs::msg::Marker::LINE_LIST;
    tire.action = visualization_msgs::msg::Marker::ADD;
    tire.pose.orientation.w = 1.0;
    tire.scale.x = 0.012;  // thick line for tire
    tire.color.r = r; tire.color.g = g; tire.color.b = b; tire.color.a = 0.9;
    tire.lifetime = rclcpp::Duration(0, 0);
    tire.points.push_back(toGlobal(bx1, by1));
    tire.points.push_back(toGlobal(bx2, by2));
    ma.markers.push_back(tire);
  };

  //@ Front-left tire (orange) — steered by δ
  addTire(front_axle_x, +hw, steering_angle, 1.0f, 0.6f, 0.0f, "tire_fl");
  //@ Front-right tire (orange) — steered by δ
  addTire(front_axle_x, -hw, steering_angle, 1.0f, 0.6f, 0.0f, "tire_fr");
  //@ Rear-left tire (gray) — fixed, aligned with body (angle=0)
  addTire(rear_axle_x, +hw, 0.0, 0.6f, 0.6f, 0.6f, "tire_rl");
  //@ Rear-right tire (gray) — fixed, aligned with body (angle=0)
  addTire(rear_axle_x, -hw, 0.0, 0.6f, 0.6f, 0.6f, "tire_rr");
}

void AckermannPlayGround::trajectory2posearray_cuboids(
    const base_trajectory::Trajectory &a_traj,
    geometry_msgs::msg::PoseArray &pose_arr,
    pcl::PointCloud<pcl::PointXYZ> &cuboids_pcl) {
  for (unsigned int i = 0; i < a_traj.getPointsSize(); i++) {
    auto p = a_traj.getPoint(i);
    pose_arr.poses.push_back(p.pose);
  }
}

void AckermannPlayGround::getBestTrajectory(
    std::string traj_gen_name, base_trajectory::Trajectory &best_traj) {
  best_traj.cost_ = -1;
  double minimum_cost = 9999999;
  geometry_msgs::msg::PoseArray accepted_pose_arr;
  geometry_msgs::msg::PoseArray rejected_pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;
  for (auto traj_it = trajectories_->begin(); traj_it != trajectories_->end();
       traj_it++) {
    mpc_critics_ros_->scoreTrajectory(traj_gen_name, (*traj_it));
    if ((*traj_it).cost_ >= 0 && (*traj_it).cost_ <= minimum_cost) {
      best_traj = (*traj_it);
      minimum_cost = (*traj_it).cost_;
    }
    if ((*traj_it).cost_ >= 0) {
      trajectory2posearray_cuboids((*traj_it), accepted_pose_arr, cuboids_pcl);
    } else {
      trajectory2posearray_cuboids((*traj_it), rejected_pose_arr, cuboids_pcl);
    }
  }
  std::string gbl = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  accepted_pose_arr.header.frame_id = gbl;
  accepted_pose_arr.header.stamp = clock_->now();
  pub_accepted_trajectory_pose_array_->publish(accepted_pose_arr);

  rejected_pose_arr.header.frame_id = gbl;
  rejected_pose_arr.header.stamp = clock_->now();
  pub_rejected_trajectory_pose_array_->publish(rejected_pose_arr);

  RCLCPP_INFO(rclcpp::get_logger("playground"),
              "Accepted: %lu, Rejected: %lu",
              accepted_pose_arr.poses.size(), rejected_pose_arr.poses.size());

  geometry_msgs::msg::PoseArray best_pose_arr;
  trajectory2posearray_cuboids(best_traj, best_pose_arr, cuboids_pcl);
  best_pose_arr.header.frame_id = gbl;
  best_pose_arr.header.stamp = clock_->now();
  pub_best_trajectory_pose_->publish(best_pose_arr);
}

void AckermannPlayGround::cbClickedPoint(
    const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal) {

  RCLCPP_INFO(this->get_logger(), "Got clicked point: %.2f, %.2f",
              clicked_goal->point.x, clicked_goal->point.y);

  //@ Re-read parameters (allows runtime changes via ros2 param set)
  this->get_parameter("sim_speed", sim_speed_);
  this->get_parameter("sim_steering_angle", sim_steering_angle_);
  this->get_parameter("footprint_display_ratio", footprint_display_ratio_);
  this->get_parameter("obstacle_x", obstacle_x_);
  this->get_parameter("obstacle_y", obstacle_y_);
  this->get_parameter("obstacle_z", obstacle_z_);

  //@ publish robot cuboid
  robot_cuboid_.markers.clear();
  marker_edge_.header.stamp = clock_->now();
  robot_cuboid_.markers.push_back(marker_edge_);
  pub_robot_cuboid_->publish(robot_cuboid_);

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

  //@ Use YAML-parameterized values
  robot_state.twist.twist.linear.x = sim_speed_;
  robot_state.twist.twist.angular.z = 0.0;

  //@ simulate prune plan
  std::string gbl_frame = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  prune_plan.header.frame_id = gbl_frame;
  prune_plan.header.stamp = clock_->now();
  int num_waypoints = 40;
  double dx = clicked_goal->point.x / num_waypoints;
  double dy = clicked_goal->point.y / num_waypoints;
  for (int i = 0; i < num_waypoints; i++) {
    geometry_msgs::msg::PoseStamped pst;
    pst.header.frame_id = gbl_frame;
    pst.pose.position.x = dx * i;
    pst.pose.position.y = dy * i;
    prune_plan.poses.push_back(pst);
  }
  pub_prune_plan_->publish(prune_plan);

  //@ Fill shared data
  trajectory_generators_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
  trajectory_generators_ros_->getSharedDataPtr()->robot_state_ = robot_state;
  trajectory_generators_ros_->getSharedDataPtr()->prune_plan_ = prune_plan;
  trajectory_generators_ros_->getSharedDataPtr()
      ->ackermann_drive_state_.drive.speed = sim_speed_;
  trajectory_generators_ros_->getSharedDataPtr()
      ->ackermann_drive_state_.drive.steering_angle = sim_steering_angle_;
  trajectory_generators_ros_->getSharedDataPtr()
      ->current_allowed_max_linear_speed_ = -1.0;

  trajectory_generators_ros_->initializeTheories_wi_Shared_data();

  geometry_msgs::msg::PoseArray pose_arr;
  pcl::PointCloud<pcl::PointXYZ> cuboids_pcl;
  trajectories_ = std::make_shared<std::vector<base_trajectory::Trajectory>>();

  while (trajectory_generators_ros_->hasMoreTrajectories("ackermann_simple")) {
    base_trajectory::Trajectory a_traj;
    if (trajectory_generators_ros_->nextTrajectory("ackermann_simple",
                                                   a_traj)) {
      trajectories_->push_back(a_traj);
      trajectory2posearray_cuboids(a_traj, pose_arr, cuboids_pcl);
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Generated %lu trajectories (speed=%.1f m/s, steer=%.3f rad)",
              trajectories_->size(), sim_speed_, sim_steering_angle_);

  pose_arr.header.frame_id = gbl_frame;
  pose_arr.header.stamp = clock_->now();
  pub_trajectory_pose_array_->publish(pose_arr);

  //@ ====== Footprint + Steering visualization ======
  visualization_msgs::msg::MarkerArray footprint_ma;
  int marker_id = 0;

  //@ First: delete all old markers
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  footprint_ma.markers.push_back(delete_all);
  marker_id = 0;

  for (auto &traj : *trajectories_) {
    int n = traj.getPointsSize();
    if (n <= 0) continue;
    std::set<int> display_idx = getDisplayIndices(n);
    double steer_angle = traj.thetav_; // steering angle δ for this trajectory

    for (int idx : display_idx) {
      auto pt = traj.getPoint(idx);
      addFootprintMarker(footprint_ma, marker_id, pt.pose, steer_angle,
                         gbl_frame);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Publishing %lu footprint markers",
              footprint_ma.markers.size());
  pub_footprint_markers_->publish(footprint_ma);

  //@ ====== Critics scoring ======
  std::unique_lock<mpc_critics::StackedScoringModel::model_mutex_t>
      critics_lock(
          *(mpc_critics_ros_->getStackedScoringModelPtr()->getMutex()));
  mpc_critics_ros_->getSharedDataPtr()->robot_pose_ = trans_gbl2b;
  mpc_critics_ros_->getSharedDataPtr()->robot_state_ = robot_state;

  //@ Obstacle from YAML parameters
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle;
  obstacle.reset(new pcl::PointCloud<pcl::PointXYZI>());
  obstacle->header.frame_id = gbl_frame;
  size_t obs_count = std::min({obstacle_x_.size(), obstacle_y_.size(),
                               obstacle_z_.size()});
  for (size_t i = 0; i < obs_count; i++) {
    pcl::PointXYZI pt;
    pt.x = obstacle_x_[i];
    pt.y = obstacle_y_[i];
    pt.z = obstacle_z_[i];
    obstacle->push_back(pt);
  }
  sensor_msgs::msg::PointCloud2 ros_msg_obstacle;
  pcl::toROSMsg(*obstacle, ros_msg_obstacle);
  pub_obstacle_->publish(ros_msg_obstacle);

  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_ = obstacle;
  mpc_critics_ros_->getSharedDataPtr()->prune_plan_ = prune_plan;
  mpc_critics_ros_->updateSharedData();
  base_trajectory::Trajectory best_traj;
  getBestTrajectory("ackermann_simple", best_traj);

  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_.reset(
      new pcl::PointCloud<pcl::PointXYZI>());
  mpc_critics_ros_->getSharedDataPtr()->pcl_perception_kdtree_.reset(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
}
} // namespace local_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg =
      std::make_shared<trajectory_generators::Trajectory_Generators_ROS>(
          "trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 =
      std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
  auto node_lppg = std::make_shared<local_planner::AckermannPlayGround>(
      "local_planner_play_ground");

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
