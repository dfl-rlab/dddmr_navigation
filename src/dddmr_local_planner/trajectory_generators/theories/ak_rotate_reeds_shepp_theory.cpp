/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
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
#include <trajectory_generators/ak_rotate_reeds_shepp_theory.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::AKRotateReedsSheppTheory,
                       trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSegmentEpsilon = 1e-6;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& orientation)
{
  const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}
}  // namespace

AKRotateReedsSheppTheory::AKRotateReedsSheppTheory()
    : rs_speed_(0.10),
      path_resolution_(0.05),
      closed_heading_maneuver_(true),
      closed_maneuver_reset_timeout_(1.00),
      goal_heading_distance_(0.30),
      forward_first_penalty_(0.0),
      active_side_(PlanSide::LEFT),
      active_plan_valid_(false),
      active_plan_total_steps_(0),
      active_plan_cost_(0.0),
      command_in_progress_(false),
      has_closed_heading_target_(false),
      closed_target_x_(0.0),
      closed_target_y_(0.0),
      closed_target_yaw_(0.0),
      has_last_rs_trajectory_time_(false),
      selection_cycle_prepared_(false),
      next_trajectory_index_(0)
{
  return;
}

void AKRotateReedsSheppTheory::configurateActuatorType()
{
  actuator_type_ = dddmr_sys_core::ActuatorType::STEERING;
}

void AKRotateReedsSheppTheory::onInitialize()
{
  limits_ = std::make_shared<AckermannTrajectoryGeneratorLimits>();
  params_ = std::make_shared<AckermannTrajectoryGeneratorParams>();

  node_->declare_parameter(name_ + ".rs_speed", rclcpp::ParameterValue(0.10));
  node_->get_parameter(name_ + ".rs_speed", rs_speed_);

  node_->declare_parameter(name_ + ".wheelbase", rclcpp::ParameterValue(0.2255));
  node_->get_parameter(name_ + ".wheelbase", limits_->wheelbase);

  node_->declare_parameter(name_ + ".max_steer_rad",
                           rclcpp::ParameterValue(0.30));
  node_->get_parameter(name_ + ".max_steer_rad", limits_->max_steer_rad);

  node_->declare_parameter(name_ + ".path_resolution",
                           rclcpp::ParameterValue(0.05));
  node_->get_parameter(name_ + ".path_resolution", path_resolution_);

  node_->declare_parameter(name_ + ".closed_heading_maneuver",
                           rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".closed_heading_maneuver",
                       closed_heading_maneuver_);

  node_->declare_parameter(name_ + ".closed_maneuver_reset_timeout",
                           rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".closed_maneuver_reset_timeout",
                       closed_maneuver_reset_timeout_);

  node_->declare_parameter(name_ + ".goal_heading_distance",
                           rclcpp::ParameterValue(0.30));
  node_->get_parameter(name_ + ".goal_heading_distance",
                       goal_heading_distance_);

  node_->declare_parameter(name_ + ".forward_first_penalty",
                           rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".forward_first_penalty",
                       forward_first_penalty_);

  if (rs_speed_ <= 0.0 || limits_->wheelbase <= 0.0 ||
      limits_->max_steer_rad <= 0.0 || path_resolution_ <= 0.0 ||
      forward_first_penalty_ < 0.0 ||
      goal_heading_distance_ <= 0.0 ||
      closed_maneuver_reset_timeout_ <= 0.0) {
    RCLCPP_FATAL(node_->get_logger().get_child(name_),
                 "Invalid Reeds-Shepp queue parameters.");
  }

  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "Reeds-Shepp: speed=%.3f m/s, wheelbase=%.4f m, "
              "max_steer=%.4f rad, resolution=%.3f m",
              rs_speed_, limits_->wheelbase, limits_->max_steer_rad,
              path_resolution_);
  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "RS first-gear preference: forward penalty=%.3f m",
              forward_first_penalty_);
  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "RS closed heading maneuver: %s (reset after %.2f s idle, goal distance %.2f m)",
              closed_heading_maneuver_ ? "enabled" : "disabled",
              closed_maneuver_reset_timeout_, goal_heading_distance_);
  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "RS FIFO queue: choose the lowest-cost candidate from both sides; "
              "each command advances after its planned duration.");

  candidate_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "ak_rotate_reeds_shepp_candidates", rclcpp::QoS(1).transient_local());

  // CollisionModel depends on this exact oriented-cuboid point ordering.
  params_->cuboid.push_back(readCuboidPoint("blb"));
  params_->cuboid.push_back(readCuboidPoint("brb"));
  params_->cuboid.push_back(readCuboidPoint("blt"));
  params_->cuboid.push_back(readCuboidPoint("flb"));
  params_->cuboid.push_back(readCuboidPoint("brt"));
  params_->cuboid.push_back(readCuboidPoint("frt"));
  params_->cuboid.push_back(readCuboidPoint("flt"));
  params_->cuboid.push_back(readCuboidPoint("frb"));
}

pcl::PointXYZ AKRotateReedsSheppTheory::readCuboidPoint(
    const std::string& corner)
{
  const std::string parameter = name_ + ".cuboid." + corner;
  node_->declare_parameter(parameter, rclcpp::PARAMETER_DOUBLE_ARRAY);
  const auto values = node_->get_parameter(parameter).as_double_array();
  if (values.size() != 3) {
    RCLCPP_FATAL(node_->get_logger().get_child(name_),
                 "Cuboid %s must contain exactly [x, y, z].", corner.c_str());
    return pcl::PointXYZ();
  }

  pcl::PointXYZ point;
  point.x = values[0];
  point.y = values[1];
  point.z = values[2];
  return point;
}

void AKRotateReedsSheppTheory::updateClosedHeadingTarget(
    const geometry_msgs::msg::PoseStamped& target)
{
  if (!closed_heading_maneuver_ || has_closed_heading_target_) {
    return;
  }

  clearActivePlan();
  closed_target_x_ = shared_data_->robot_pose_.transform.translation.x;
  closed_target_y_ = shared_data_->robot_pose_.transform.translation.y;
  const bool is_goal_heading = isGoalHeadingManeuver(target);
  if (is_goal_heading) {
    // P2P enters d_align_goal_heading only after XY goal tolerance is met.
    // The local prune path then ends at the global final pose, so its stored
    // quaternion is the terminal goal heading.
    closed_target_yaw_ = yawFromQuaternion(target.pose.orientation);
  }
  else if (!initialPathTangentYaw(closed_target_yaw_)) {
    RCLCPP_WARN(node_->get_logger().get_child(name_),
                "RS could not find a forward path pair; using prune-end yaw for initial alignment.");
    closed_target_yaw_ = yawFromQuaternion(target.pose.orientation);
  }
  has_closed_heading_target_ = true;

  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "RS path change: new %s-heading maneuver, target_yaw=%.3f.",
              is_goal_heading ? "goal" : "initial", closed_target_yaw_);
}

bool AKRotateReedsSheppTheory::isGoalHeadingManeuver(
    const geometry_msgs::msg::PoseStamped& target) const
{
  const auto& robot = shared_data_->robot_pose_.transform.translation;
  return std::hypot(target.pose.position.x - robot.x,
                    target.pose.position.y - robot.y) <= goal_heading_distance_;
}

bool AKRotateReedsSheppTheory::initialPathTangentYaw(double& yaw) const
{
  const auto& poses = shared_data_->prune_plan_.poses;
  if (poses.size() < 2) {
    return false;
  }

  const auto& robot = shared_data_->robot_pose_.transform.translation;
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const double distance = std::hypot(poses[i].pose.position.x - robot.x,
                                       poses[i].pose.position.y - robot.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }

  // prune_plan is ordered along the global path.  Find the first distinct
  // forward point after the closest path point; duplicated prune points are
  // skipped so the tangent remains well-defined.
  for (std::size_t next = nearest + 1; next < poses.size(); ++next) {
    const double dx = poses[next].pose.position.x - poses[nearest].pose.position.x;
    const double dy = poses[next].pose.position.y - poses[nearest].pose.position.y;
    if (std::hypot(dx, dy) > kSegmentEpsilon) {
      yaw = std::atan2(dy, dx);
      return true;
    }
  }
  return false;
}

void AKRotateReedsSheppTheory::initialise()
{
  // StackedGenerator calls initialise() on *every* theory every local-planner
  // cycle, including while ackermann_simple is selected.  Do not create or
  // reset an RS plan here: that work belongs to prepareSelectedCycle(), which
  // is reached only through hasMoreTrajectories() for the selected theory.
  trajectories_.clear();
  next_trajectory_index_ = 0;
  selection_cycle_prepared_ = false;
}

void AKRotateReedsSheppTheory::prepareSelectedCycle()
{
  if (selection_cycle_prepared_) {
    return;
  }
  selection_cycle_prepared_ = true;

  if (shared_data_->prune_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *node_->get_clock(),
                         5000, "Cannot generate Reeds-Shepp path: prune plan is empty.");
    return;
  }

  const geometry_msgs::msg::PoseStamped target =
      shared_data_->prune_plan_.poses.back();
  const rclcpp::Time now = node_->get_clock()->now();
  const bool start_new_maneuver = !has_last_rs_trajectory_time_ ||
      (now - last_rs_trajectory_time_).seconds() < 0.0 ||
      (now - last_rs_trajectory_time_).seconds() > closed_maneuver_reset_timeout_;
  if (start_new_maneuver) {
    clearActivePlan();
    has_closed_heading_target_ = false;
  }
  updateClosedHeadingTarget(target);

  if (active_plan_valid_) {
    advanceElapsedStep();
  }

  if (!active_plan_valid_ && !createActivePlan(target)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger().get_child(name_), *node_->get_clock(),
                         1000, "Cannot create Reeds-Shepp command queue.");
    return;
  }

  base_trajectory::Trajectory trajectory;
  if (buildTrajectory(trajectory)) {
    trajectories_.push_back(trajectory);
  }
}

bool AKRotateReedsSheppTheory::hasMoreTrajectories()
{
  // Unlike initialise(), this method is called only for the trajectory
  // generator requested by P2P's current state.  It is therefore the safe
  // place to update the persistent RS FIFO.
  prepareSelectedCycle();
  return next_trajectory_index_ < trajectories_.size();
}

bool AKRotateReedsSheppTheory::nextTrajectory(
    base_trajectory::Trajectory& trajectory)
{
  if (!hasMoreTrajectories()) {
    return false;
  }
  trajectory = trajectories_[next_trajectory_index_++];
  const rclcpp::Time now = node_->get_clock()->now();
  last_rs_trajectory_time_ = now;
  has_last_rs_trajectory_time_ = true;

  if (active_plan_valid_ && !command_in_progress_ && !remaining_steps_.empty()) {
    in_flight_step_ = remaining_steps_.front();
    in_flight_offer_time_ = now;
    command_in_progress_ = true;
    const std::size_t current_step = active_plan_total_steps_ -
        remaining_steps_.size() + 1;
    RCLCPP_INFO(node_->get_logger().get_child(name_),
                "RS path get %zu, cur at %zu, v=%.3f, angle=%.3f, side=%s",
                active_plan_total_steps_, current_step, in_flight_step_.speed,
                in_flight_step_.steering_angle,
                active_side_ == PlanSide::LEFT ? "left" : "right");
  }
  return true;
}

void AKRotateReedsSheppTheory::clearActivePlan()
{
  active_plan_valid_ = false;
  remaining_steps_.clear();
  active_plan_total_steps_ = 0;
  active_plan_cost_ = 0.0;
  command_in_progress_ = false;
}

bool AKRotateReedsSheppTheory::createActivePlan(
    const geometry_msgs::msg::PoseStamped& target)
{
  RSCandidate candidate;
  if (!selectCandidate(target, candidate) ||
      !buildReferencePlan(candidate)) {
    return false;
  }

  remaining_steps_ = std::deque<RSControlStep>(candidate.steps.begin(),
                                                 candidate.steps.end());
  if (remaining_steps_.empty()) {
    return false;
  }
  active_side_ = candidate.segments.front().steer == 'L'
      ? PlanSide::LEFT : PlanSide::RIGHT;
  active_plan_total_steps_ = remaining_steps_.size();
  active_plan_cost_ = candidateCost(candidate);
  active_plan_valid_ = true;
  command_in_progress_ = false;

  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "RS path change: create %s FIFO path with %zu point(s), %.3f m.",
              active_side_ == PlanSide::LEFT ? "left" : "right",
              remaining_steps_.size(), candidate.total_length);
  return true;
}

bool AKRotateReedsSheppTheory::advanceElapsedStep()
{
  if (!command_in_progress_) {
    return false;
  }
  const double elapsed =
      (node_->get_clock()->now() - in_flight_offer_time_).seconds();
  if (elapsed < in_flight_step_.duration) {
    return false;
  }

  if (!remaining_steps_.empty()) {
    remaining_steps_.pop_front();
  }
  command_in_progress_ = false;
  RCLCPP_DEBUG(node_->get_logger().get_child(name_),
               "Completed one RS command duration; %zu step(s) remain.",
               remaining_steps_.size());

  if (remaining_steps_.empty()) {
    RCLCPP_INFO(node_->get_logger().get_child(name_),
                "RS path change: completed %s FIFO path; reuse %s if P2P asks again.",
                active_side_ == PlanSide::LEFT ? "left" : "right",
                active_side_ == PlanSide::LEFT ? "left" : "right");
    clearActivePlan();
  }
  return true;
}

double AKRotateReedsSheppTheory::candidateCost(
    const RSCandidate& candidate) const
{
  if (candidate.segments.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return candidate.total_length +
      (candidate.segments.front().gear > 0 ? forward_first_penalty_ : 0.0);
}

bool AKRotateReedsSheppTheory::selectCandidate(
    const geometry_msgs::msg::PoseStamped& target, RSCandidate& candidate) const
{
  const auto& current = shared_data_->robot_pose_.transform;
  return selectCandidateFromPose(current.translation.x, current.translation.y,
                                 yawFromQuaternion(current.rotation), target,
                                 candidate);
}

bool AKRotateReedsSheppTheory::selectCandidateFromPose(
    double start_x, double start_y, double start_yaw,
    const geometry_msgs::msg::PoseStamped& target,
    RSCandidate& candidate) const
{
  const double target_yaw = closed_heading_maneuver_ && has_closed_heading_target_
      ? closed_target_yaw_ : yawFromQuaternion(target.pose.orientation);
  const double target_x = closed_heading_maneuver_ && has_closed_heading_target_
      ? closed_target_x_ : target.pose.position.x;
  const double target_y = closed_heading_maneuver_ && has_closed_heading_target_
      ? closed_target_y_ : target.pose.position.y;
  const double dx = target_x - start_x;
  const double dy = target_y - start_y;

  const double tangent = std::tan(limits_->max_steer_rad);
  if (std::fabs(tangent) < kSegmentEpsilon) {
    return false;
  }
  const double minimum_turn_radius = limits_->wheelbase / tangent;
  const double c = std::cos(start_yaw);
  const double s = std::sin(start_yaw);
  const double x = (c * dx + s * dy) / minimum_turn_radius;
  const double y = (-s * dx + c * dy) / minimum_turn_radius;
  const double phi = mod2pi(target_yaw - start_yaw);

  auto candidates = generateCandidates(x, y, phi);
  // Publish every original RS candidate before the existing selection logic
  // ranks them by cost.
  publishCandidateMarkers(candidates, minimum_turn_radius);
  if (candidates.empty()) {
    return false;
  }

  std::sort(candidates.begin(), candidates.end(),
            [this, minimum_turn_radius](const RSCandidate& lhs,
                                        const RSCandidate& rhs) {
              // Candidate lengths are normalized by the minimum radius here;
              // this user-facing preference is expressed in metres.
              const double lhs_score = lhs.total_length +
                  (lhs.segments.front().gear > 0
                       ? forward_first_penalty_ / minimum_turn_radius : 0.0);
              const double rhs_score = rhs.total_length +
                  (rhs.segments.front().gear > 0
                       ? forward_first_penalty_ / minimum_turn_radius : 0.0);
              return lhs_score < rhs_score;
            });
  candidate = candidates.front();
  for (auto& segment : candidate.segments) {
    segment.length *= minimum_turn_radius;
  }
  candidate.total_length *= minimum_turn_radius;
  return true;
}

std::vector<AKRotateReedsSheppTheory::RSCandidate>
AKRotateReedsSheppTheory::generateCandidates(double x, double y,
                                              double phi) const
{
  struct BaseWord
  {
    bool (*solver)(double, double, double, double&, double&, double&);
    std::array<char, 3> steers;
    std::array<int, 3> gears;
  };
  const std::array<BaseWord, 3> base_words = {{
      {&AKRotateReedsSheppTheory::lpSpLp, {'L', 'S', 'L'}, {1, 1, 1}},
      {&AKRotateReedsSheppTheory::lpSpRp, {'L', 'S', 'R'}, {1, 1, 1}},
      {&AKRotateReedsSheppTheory::lpRmLp, {'L', 'R', 'L'}, {1, -1, 1}},
  }};

  std::vector<RSCandidate> candidates;
  for (const auto& word : base_words) {
    for (const int reflect : {1, -1}) {
      for (const int timeflip : {1, -1}) {
        double xx = timeflip * x;
        double yy = y;
        double pphi = timeflip * phi;
        if (reflect == -1) {
          yy = -yy;
          pphi = -pphi;
        }

        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        if (!word.solver(xx, yy, pphi, t, u, v)) {
          continue;
        }

        const std::array<double, 3> lengths = {{t, u, v}};
        RSCandidate candidate;
        candidate.total_length = 0.0;
        for (std::size_t i = 0; i < lengths.size(); ++i) {
          char steer = word.steers[i];
          if (reflect == -1) {
            steer = steer == 'L' ? 'R' : (steer == 'R' ? 'L' : 'S');
          }
          const double length = std::fabs(lengths[i]);
          if (length <= kSegmentEpsilon) {
            continue;
          }
          candidate.segments.push_back(
              {steer, timeflip * word.gears[i], length});
          candidate.total_length += length;
        }
        if (!candidate.segments.empty()) {
          candidates.push_back(candidate);
        }
      }
    }
  }
  return candidates;
}

void AKRotateReedsSheppTheory::publishCandidateMarkers(
    const std::vector<RSCandidate>& candidates,
    double minimum_turn_radius) const
{
  if (!candidate_markers_pub_) {
    return;
  }

  static const std::array<std::array<float, 3>, 12> kColors = {{
      {{0.90F, 0.10F, 0.10F}}, {{0.10F, 0.35F, 0.95F}},
      {{0.10F, 0.75F, 0.25F}}, {{0.95F, 0.60F, 0.05F}},
      {{0.60F, 0.15F, 0.85F}}, {{0.05F, 0.75F, 0.75F}},
      {{0.85F, 0.20F, 0.55F}}, {{0.55F, 0.80F, 0.10F}},
      {{0.95F, 0.45F, 0.35F}}, {{0.20F, 0.20F, 0.20F}},
      {{0.45F, 0.30F, 0.85F}}, {{0.20F, 0.55F, 0.45F}},
  }};

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = shared_data_->robot_pose_.header;
  clear.ns = "ak_rotate_reeds_shepp_candidates";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    RSCandidate visual_candidate = candidates[i];
    for (auto& segment : visual_candidate.segments) {
      segment.length *= minimum_turn_radius;
    }
    visual_candidate.total_length *= minimum_turn_radius;
    if (!buildReferencePlan(visual_candidate)) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = shared_data_->robot_pose_.header;
    marker.ns = "ak_rotate_reeds_shepp_candidates";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.035;
    marker.color.r = kColors[i % kColors.size()][0];
    marker.color.g = kColors[i % kColors.size()][1];
    marker.color.b = kColors[i % kColors.size()][2];
    marker.color.a = 1.0;

    for (const auto& step : visual_candidate.steps) {
      geometry_msgs::msg::Point point;
      point.x = step.pose.pose.position.x;
      point.y = step.pose.pose.position.y;
      point.z = step.pose.pose.position.z + 0.05;
      marker.points.push_back(point);
    }
    markers.markers.push_back(marker);
  }
  candidate_markers_pub_->publish(markers);
}

bool AKRotateReedsSheppTheory::lpSpLp(double x, double y, double phi,
                                      double& t, double& u, double& v)
{
  const auto polar_result = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  u = polar_result.first;
  t = polar_result.second;
  if (t < 0.0) {
    return false;
  }
  v = mod2pi(phi - t);
  return v >= 0.0;
}

bool AKRotateReedsSheppTheory::lpSpRp(double x, double y, double phi,
                                      double& t, double& u, double& v)
{
  const auto polar_result = polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
  const double u1_squared = polar_result.first * polar_result.first;
  if (u1_squared < 4.0) {
    return false;
  }
  u = std::sqrt(u1_squared - 4.0);
  const double theta = std::atan2(2.0, u);
  t = mod2pi(polar_result.second + theta);
  v = mod2pi(t - phi);
  return t >= 0.0 && v >= 0.0;
}

bool AKRotateReedsSheppTheory::lpRmLp(double x, double y, double phi,
                                      double& t, double& u, double& v)
{
  const auto polar_result = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
  if (polar_result.first > 4.0) {
    return false;
  }
  u = -2.0 * std::asin(0.25 * polar_result.first);
  t = mod2pi(polar_result.second + 0.5 * u + kPi);
  v = mod2pi(phi - t + u);
  return t >= 0.0 && u <= 0.0;
}

double AKRotateReedsSheppTheory::mod2pi(double angle)
{
  angle = std::fmod(angle, 2.0 * kPi);
  if (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  if (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  return angle;
}

std::pair<double, double> AKRotateReedsSheppTheory::polar(double x, double y)
{
  return {std::hypot(x, y), std::atan2(y, x)};
}

double AKRotateReedsSheppTheory::steeringFor(char steer) const
{
  if (steer == 'L') {
    return limits_->max_steer_rad;
  }
  if (steer == 'R') {
    return -limits_->max_steer_rad;
  }
  return 0.0;
}

Eigen::Vector3f AKRotateReedsSheppTheory::integrateBicycle(
    const Eigen::Vector3f& pose, double speed, double steering_angle,
    double dt) const
{
  Eigen::Vector3f next = Eigen::Vector3f::Zero();
  const double yaw_rate =
      speed * std::tan(steering_angle) / limits_->wheelbase;
  next[0] = pose[0] + speed * std::cos(pose[2]) * dt;
  next[1] = pose[1] + speed * std::sin(pose[2]) * dt;
  next[2] = pose[2] + yaw_rate * dt;
  return next;
}

bool AKRotateReedsSheppTheory::buildReferencePlan(
    RSCandidate& candidate) const
{
  candidate.steps.clear();
  if (candidate.segments.empty()) {
    return false;
  }

  // This integration is done exactly once, before the L/R candidate is
  // offered to CollisionModel.  After a candidate is selected these absolute
  // map-frame points are the persistent reference plan, not a fresh solve.
  const Eigen::Affine3d global_to_base =
      tf2::transformToEigen(shared_data_->robot_pose_);
  Eigen::Vector3f relative_pose = Eigen::Vector3f::Zero();

  for (const RSSegment& segment : candidate.segments) {
    const double speed = segment.gear * rs_speed_;
    const double steering_angle = steeringFor(segment.steer);
    const int steps = std::max(
        1, static_cast<int>(std::ceil(segment.length / path_resolution_)));
    const double dt = (segment.length / rs_speed_) / steps;

    for (int step = 0; step < steps; ++step) {
      relative_pose = integrateBicycle(relative_pose, speed, steering_angle, dt);

      Eigen::Affine3d base_to_path(
          Eigen::AngleAxisd(relative_pose[2], Eigen::Vector3d::UnitZ()));
      base_to_path.translation().x() = relative_pose[0];
      base_to_path.translation().y() = relative_pose[1];
      const Eigen::Affine3d global_to_path = global_to_base * base_to_path;
      const auto transform = tf2::eigenToTransform(global_to_path);

      RSControlStep reference_step;
      reference_step.pose.header = shared_data_->robot_pose_.header;
      reference_step.pose.pose.position.x = transform.transform.translation.x;
      reference_step.pose.pose.position.y = transform.transform.translation.y;
      reference_step.pose.pose.position.z = transform.transform.translation.z;
      reference_step.pose.pose.orientation = transform.transform.rotation;
      reference_step.speed = speed;
      reference_step.steering_angle = steering_angle;
      reference_step.duration = dt;
      candidate.steps.push_back(reference_step);
    }
  }

  return !candidate.steps.empty();
}

bool AKRotateReedsSheppTheory::buildTrajectory(
    base_trajectory::Trajectory& trajectory)
{
  if (!active_plan_valid_ || remaining_steps_.empty()) {
    return false;
  }

  // P2P publishes xv_/steering_angle_, not Trajectory::points.  The FIFO
  // front is therefore the one command offered this loop; all points remain
  // in the trajectory solely as CollisionModel's future collision horizon.
  const RSControlStep& command = remaining_steps_.front();
  trajectory.actuator_type_ = actuator_type_;
  trajectory.cost_ = active_plan_cost_;
  trajectory.resetPoints();
  trajectory.xv_ = command.speed;
  trajectory.yv_ = 0.0;
  trajectory.thetav_ = command.speed * std::tan(command.steering_angle) /
      limits_->wheelbase;
  trajectory.steering_angle_ = command.steering_angle;
  trajectory.steering_angle_velocity_ = 0.0;
  trajectory.time_delta_ = command.duration;

  for (const RSControlStep& step : remaining_steps_) {
    geometry_msgs::msg::PoseStamped pose = step.pose;
    pose.header.stamp = shared_data_->robot_pose_.header.stamp;

    geometry_msgs::msg::TransformStamped pose_transform;
    pose_transform.transform.translation.x = pose.pose.position.x;
    pose_transform.transform.translation.y = pose.pose.position.y;
    pose_transform.transform.translation.z = pose.pose.position.z;
    pose_transform.transform.rotation = pose.pose.orientation;
    const Eigen::Affine3d global_to_path =
        tf2::transformToEigen(pose_transform);

    pcl::PointCloud<pcl::PointXYZ> transformed_cuboid;
    pcl::transformPointCloud(params_->cuboid, transformed_cuboid,
                             global_to_path);
    base_trajectory::cuboid_min_max_t cuboid_min_max;
    pcl::getMinMax3D(transformed_cuboid, cuboid_min_max.first,
                     cuboid_min_max.second);
    if (!trajectory.addPoint(pose, transformed_cuboid, cuboid_min_max)) {
      return false;
    }
  }

  return trajectory.getPointsSize() > 0;
}
}  // namespace trajectory_generators
