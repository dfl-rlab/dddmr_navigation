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
#ifndef AK_ROTATE_REEDS_SHEPP_THEORY_H_
#define AK_ROTATE_REEDS_SHEPP_THEORY_H_

#include <array>
#include <deque>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <trajectory_generators/trajectory_generator_theory.h>
#include <trajectory_generators/ackermann_simple_trajectory_generator_limits.h>
#include <trajectory_generators/ackermann_simple_trajectory_generator_params.h>

#include <pcl/common/common.h>

namespace trajectory_generators
{

// A single-side Reeds-Shepp word becomes a FIFO command queue.  Heading
// alignment drives its front command for the command's planned duration, then
// advances to the next command.  It deliberately does not use vehicle
// feedback or a timeout to decide whether to change sides.
class AKRotateReedsSheppTheory : public TrajectoryGeneratorTheory
{
public:
  AKRotateReedsSheppTheory();

  bool hasMoreTrajectories() override;
  bool nextTrajectory(base_trajectory::Trajectory& trajectory) override;

private:
  struct RSSegment
  {
    char steer;  // 'L', 'R', or 'S'
    int gear;    // +1 forward, -1 reverse
    double length;
  };

  struct RSControlStep
  {
    geometry_msgs::msg::PoseStamped pose;
    double speed;
    double steering_angle;
    double duration;
  };

  struct RSCandidate
  {
    double total_length;
    std::vector<RSSegment> segments;
    std::vector<RSControlStep> steps;
  };

  enum class PlanSide { LEFT, RIGHT };

  void initialise() override;
  void prepareSelectedCycle();
  bool buildReferencePlan(RSCandidate& candidate) const;
  bool buildTrajectory(base_trajectory::Trajectory& trajectory);
  bool selectCandidate(char first_steer, const geometry_msgs::msg::PoseStamped& target,
                       RSCandidate& candidate) const;
  bool selectCandidateFromPose(char first_steer, double start_x, double start_y,
                               double start_yaw,
                               const geometry_msgs::msg::PoseStamped& target,
                               RSCandidate& candidate) const;
  std::vector<RSCandidate> generateCandidates(double x, double y,
                                              double phi) const;
  void clearActivePlan();
  bool createActivePlan(PlanSide side,
                        const geometry_msgs::msg::PoseStamped& target);
  bool advanceElapsedStep();
  double candidateCost(const RSCandidate& candidate) const;

  static bool lpSpLp(double x, double y, double phi,
                     double& t, double& u, double& v);
  static bool lpSpRp(double x, double y, double phi,
                     double& t, double& u, double& v);
  static bool lpRmLp(double x, double y, double phi,
                     double& t, double& u, double& v);
  static double mod2pi(double angle);
  static std::pair<double, double> polar(double x, double y);

  Eigen::Vector3f integrateBicycle(const Eigen::Vector3f& pose,
                                   double speed, double steering_angle,
                                   double dt) const;
  double steeringFor(char steer) const;
  pcl::PointXYZ readCuboidPoint(const std::string& corner);

  // The P2P state machine uses this generator only while aligning heading.
  // Keep a fixed x/y anchor for that maneuver so the RS endpoint returns to
  // the point where alignment began instead of driving toward prune-plan end.
  void updateClosedHeadingTarget(const geometry_msgs::msg::PoseStamped& target);
  bool initialPathTangentYaw(double& yaw) const;
  bool isGoalHeadingManeuver(const geometry_msgs::msg::PoseStamped& target) const;

  void onInitialize() override;
  void configurateActuatorType() override;

  std::shared_ptr<AckermannTrajectoryGeneratorLimits> limits_;
  std::shared_ptr<AckermannTrajectoryGeneratorParams> params_;
  double rs_speed_;
  double path_resolution_;
  bool closed_heading_maneuver_;
  double closed_maneuver_reset_timeout_;
  double goal_heading_distance_;
  double forward_first_penalty_;
  PlanSide initial_side_;
  PlanSide active_side_;
  bool active_plan_valid_;
  std::deque<RSControlStep> remaining_steps_;
  std::size_t active_plan_total_steps_;
  double active_plan_cost_;
  bool command_in_progress_;
  RSControlStep in_flight_step_;
  rclcpp::Time in_flight_offer_time_;
  bool has_closed_heading_target_;
  double closed_target_x_;
  double closed_target_y_;
  double closed_target_yaw_;
  bool has_last_rs_trajectory_time_;
  rclcpp::Time last_rs_trajectory_time_;
  bool selection_cycle_prepared_;
  unsigned int next_trajectory_index_;
  std::vector<base_trajectory::Trajectory> trajectories_;
};

}  // namespace trajectory_generators

#endif  // AK_ROTATE_REEDS_SHEPP_THEORY_H_
