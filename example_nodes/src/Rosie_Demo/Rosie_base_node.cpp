#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <example_nodes/BaseMotionAction.h>

#include "actionlib/server/simple_action_server.h"

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"

#include <ros/console.h>

namespace hebi {
namespace ros {

// TODO: fix includes here and in Rosie_arm_node file!

// TODO: move these classes/etc to separate header/implementation files!

// TODO: BaseTrajectory is _almost_ identical to ArmTrajectory.  Maybe combine
// these?

// Note: base trajectory doesn't allow for smooth replanning, because that would be...difficult.  It just
// represents relative motion in (x, y, theta)

struct Color {
  Color() = default;
  Color(uint8_t r, uint8_t g, uint8_t b) : set_color_(true), r_(r), g_(g), b_(b) {}
  bool set_color_{false};
  uint8_t r_{0}; 
  uint8_t g_{0}; 
  uint8_t b_{0}; 
};

class BaseTrajectory {
public:
  static BaseTrajectory create(const Eigen::VectorXd& dest_positions, double t_now)
  {
    BaseTrajectory base_trajectory;

    // Set up initial trajectory
    Eigen::MatrixXd positions(3, 1);
    positions.col(0) = dest_positions;
    base_trajectory.replan(t_now, positions);

    return base_trajectory;
  }

  void getState(
    double t_now, 
    Eigen::VectorXd& positions,
    Eigen::VectorXd& velocities,
    Eigen::VectorXd& accelerations) {

    // (Cap the effective time to the end of the trajectory)
    double t = std::min(t_now - trajectory_start_time_,
                        trajectory_->getDuration());

    trajectory_->getState(t, &positions, &velocities, &accelerations);
  }

  // Updates the Arm State by planning a trajectory to a given set of joint
  // waypoints.  Uses the current trajectory/state if defined.
  // NOTE: this call assumes feedback is populated.
  void replan(
    double t_now,
    const Eigen::MatrixXd& new_positions,
    const Eigen::MatrixXd& new_velocities,
    const Eigen::MatrixXd& new_accelerations) {

    int num_joints = new_positions.rows();

    // Start from (0, 0, 0), as this is a relative motion.
    Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);

    int num_waypoints = new_positions.cols() + 1;

    Eigen::MatrixXd positions(num_joints, num_waypoints);
    Eigen::MatrixXd velocities(num_joints, num_waypoints);
    Eigen::MatrixXd accelerations(num_joints, num_waypoints);

    // Initial state
    positions.col(0) = curr_pos;
    velocities.col(0) = curr_vel;
    accelerations.col(0) = curr_accel;

    // Copy new waypoints
    positions.rightCols(num_waypoints - 1) = new_positions;
    velocities.rightCols(num_waypoints - 1) = new_velocities;
    accelerations.rightCols(num_waypoints - 1) = new_accelerations;

    // Get waypoint times
    Eigen::VectorXd trajTime =
      getWaypointTimes(positions, velocities, accelerations);

    // Create new trajectory
    trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                    trajTime, positions, &velocities, &accelerations);
    trajectory_start_time_ = t_now;
  }

  // Updates the Base State by planning a trajectory to a given set of joint
  // waypoints.  Uses the current trajectory/state if defined.
  // NOTE: this is a wrapper around the more general replan that
  // assumes zero end velocity and acceleration.
  void replan(
    double t_now,
    const Eigen::MatrixXd& new_positions) {

    int num_joints = new_positions.rows();
    int num_waypoints = new_positions.cols();

    // Unconstrained velocities and accelerations during the path, but set to
    // zero at the end.
    double nan = std::numeric_limits<double>::quiet_NaN();

    Eigen::MatrixXd velocities(num_joints, num_waypoints);
    velocities.setConstant(nan);
    velocities.rightCols<1>() = Eigen::VectorXd::Zero(num_joints);

    Eigen::MatrixXd accelerations(num_joints, num_waypoints);
    accelerations.setConstant(nan);
    accelerations.rightCols<1>() = Eigen::VectorXd::Zero(num_joints);

    replan(t_now, new_positions, velocities, accelerations);
  }

  // Heuristic to get the timing of the waypoints. This function can be
  // modified to add custom waypoint timing.
  Eigen::VectorXd getWaypointTimes(
    const Eigen::MatrixXd& positions,
    const Eigen::MatrixXd& velocities,
    const Eigen::MatrixXd& accelerations) {

    // TODO: make this configurable!
    double rampTime = 4.0; // Per meter
    double dist = std::pow(positions(0, 1) - positions(0, 0), 2) + 
                  std::pow(positions(1, 1) - positions(1, 0), 2);
    dist = std::sqrt(dist);
    rampTime *= dist;
    rampTime = std::max(rampTime, 1.5);

    size_t num_waypoints = positions.cols();

    Eigen::VectorXd times(num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i)
      times[i] = rampTime * (double)i;

    return times;
  }    

  std::shared_ptr<hebi::trajectory::Trajectory> getTraj() { return trajectory_; }
  double getTrajStartTime() { return trajectory_start_time_; }
  double getTrajEndTime() { return trajectory_start_time_ + trajectory_->getDuration(); }

private:
  // This is private, because we want to ensure the BaseTrajectory is always
  // initialized correctly after creation; use the "create" factory method
  // instead.
  BaseTrajectory() = default;
      
  std::shared_ptr<hebi::trajectory::Trajectory> trajectory_ {};
  double trajectory_start_time_ {};
};

class OmniBase {
public:
  static std::unique_ptr<OmniBase> create(
    const std::vector<std::string>& families,
    const std::vector<std::string>& names,
    double start_time) {

    // Invalid input!  Size mismatch
    if (names.size() != 3 || (families.size() != 1 && families.size() != 3)) {
      assert(false);
      return nullptr;
    }

    //Get a group
    Lookup lookup;
    auto group = lookup.getGroupFromNames(families, names);

    if (!group)
      return nullptr;

    // Load the appropriate gains file
    // TODO: BETTER PACKAGE THIS FILE!!!
    GroupCommand gains_cmd(group -> size());
    gains_cmd.readGains("/home/hebi/catkin_ws/src/HEBI-ROS/example_nodes/include/gains/omnibase_gains.xml");

    constexpr double feedback_frequency = 100;
    group->setFeedbackFrequencyHz(feedback_frequency);
    constexpr long command_lifetime = 250;
    group->setCommandLifetimeMs(command_lifetime);

    // Try to get feedback -- if we don't get a packet in the first N times,
    // something is wrong
    int num_attempts = 0;
    
    // This whole "plan initial trajectory" is a little hokey...but it's better than nothing  
    GroupFeedback feedback(group->size());
    while (!group->getNextFeedback(feedback)) {
      if (num_attempts++ > 20) {
        return nullptr;
      }
    }
 
    // NOTE: I don't like that start time is _before_ the "get feedback"
    // loop above...but this is only during initialization
    BaseTrajectory base_trajectory = BaseTrajectory::create(Eigen::Vector3d::Zero(), start_time);
    return std::unique_ptr<OmniBase>(new OmniBase(group, base_trajectory, start_time));
  }

  // Converts a certain number of radians into radians that each wheel would turn
  // _from theta == 0_ to obtain this rotation.
  // Note: we only do this for velocities in order to combine rotations and translations without doing gnarly
  // integrations.
  double convertSE2ToWheel() {

    double theta = pos_[2];
    double dtheta = vel_[2];

    double offset = 1.0;
    double ctheta = std::cos(-theta);
    double stheta = std::sin(-theta);
    double dx = vel_[0] * ctheta - vel_[1] * stheta;
    double dy = vel_[0] * stheta + vel_[1] * ctheta;
    dx /= wheel_radius_;
    dy /= wheel_radius_;
    double ratio = sqrt(3)/2;

    //////////////
    // Velocity:
    //////////////

    // Rotation
    wheel_vel_[0] =
    wheel_vel_[1] =
    wheel_vel_[2] = -dtheta * base_radius_ / wheel_radius_;
    // Translation
    wheel_vel_[0] += - 0.5 * dy - ratio * dx;
    wheel_vel_[1] += - 0.5 * dy + ratio * dx;
    wheel_vel_[2] += dy;

  }

  bool update(double time) {
  
    double dt = 0; 
    if (last_time_ < 0) { // Sentinal value set when we restart...
      last_time_ = time;
    } else {
      dt = time - last_time_;
    }
 
    if (!group_->getNextFeedback(feedback_))
      return false;

    // Update command from trajectory
    base_trajectory_.getState(time, pos_, vel_, accel_);

    // Convert from x/y/theta to wheel 1/2/3
    convertSE2ToWheel();

    // Integrate position using wheel velocities.
    last_wheel_pos_ += wheel_vel_ * dt;
    command_.setPosition(last_wheel_pos_);

    // Use velocity from trajectory, converted from x/y/theta into wheel velocities above.
    command_.setVelocity(wheel_vel_);

    for (int i = 0; i < 3; ++i) {
      if (color_.set_color_ == false) 
        command_[i].led().set(hebi::Color(0, 0, 0, 0));
      else
        command_[i].led().set(hebi::Color(
          color_.r_, color_.g_, color_.b_));
    }

    group_->sendCommand(command_);

    last_time_ = time;

    return true; 
  }
 
  double trajectoryPercentComplete(double time) {
    return std::min((time - base_trajectory_.getTrajStartTime()) / base_trajectory_.getTraj()->getDuration(), 1.0) * 100;
  }
  bool isTrajectoryComplete(double time) {
    return time > base_trajectory_.getTrajEndTime();
  }

  GroupFeedback& getLastFeedback() { return feedback_; }
  BaseTrajectory& getTrajectory() { return base_trajectory_; }

  void resetStart(Color& color)
  {
    start_wheel_pos_ = feedback_.getPosition();
    last_wheel_pos_ = start_wheel_pos_;
    last_time_ = -1;
    color_ = color;
  }

  void clearColor()
  {
    Color c;
    color_ = c;
  }

private:

  OmniBase(std::shared_ptr<Group> group,
    BaseTrajectory base_trajectory,
    double start_time)
    : group_(group),
      feedback_(group->size()),
      command_(group->size()),
      pos_(Eigen::VectorXd::Zero(group->size())),
      vel_(Eigen::VectorXd::Zero(group->size())),
      accel_(Eigen::VectorXd::Zero(group->size())),
      start_wheel_pos_(Eigen::VectorXd::Zero(group->size())),
      last_wheel_pos_(Eigen::VectorXd::Zero(group->size())),
      wheel_vel_(Eigen::VectorXd::Zero(group->size())),
      wheel_effort_(Eigen::VectorXd::Zero(group->size())),
      base_trajectory_{base_trajectory}
  { }

  /* Declare main kinematic variables */
  static constexpr double wheel_radius_ = 0.0762; // m
  static constexpr double base_radius_ = 0.235; // m (center of omni to origin of base)

  std::shared_ptr<Group> group_;

  GroupFeedback feedback_;
  GroupCommand command_;

  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
  Eigen::VectorXd start_wheel_pos_;
  Eigen::VectorXd last_wheel_pos_;
//  Eigen::VectorXd wheel_pos_;
  Eigen::VectorXd wheel_vel_;
  Eigen::VectorXd wheel_effort_;

  BaseTrajectory base_trajectory_;

  double last_time_{-1};

  Color color_;
};

class BaseNode {
public:
  BaseNode(OmniBase& base) : base_(base) {
    Color c;
    base_.resetStart(c);
  }

  void startBaseMotion(const example_nodes::BaseMotionGoalConstPtr& goal) {

    // Note: this is implemented right now as translation, _THEN_ rotation...
    // we can update this later.

    // We pass in the trajectory points in (x, y, theta)... 
    size_t num_waypoints = 1;
    Eigen::MatrixXd waypoints(3, num_waypoints);

    ROS_INFO("Executing base motion action");
    example_nodes::BaseMotionFeedback feedback;

    ////////////////
    // Translation
    ////////////////
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b);
    base_.resetStart(color);

    waypoints(0, 0) = goal->x;
    waypoints(1, 0) = goal->y;
    waypoints(2, 0) = goal->theta;
    base_.getTrajectory().replan(
      ::ros::Time::now().toSec(),
      waypoints);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);
    bool success = true;
    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Preempted base motion");
        action_server_->setPreempted();
        success = false;
        break;
      }
      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      auto& base_traj = base_.getTrajectory();
      feedback.percent_complete = base_.trajectoryPercentComplete(t) / 2.0;
      action_server_->publishFeedback(feedback);
 
      if (base_.isTrajectoryComplete(t)) {
        ROS_INFO("TRANSLATION COMPLETE");
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }

    base_.clearColor();

    // publish when the base is done with a motion
    ROS_INFO("Completed base motion action");
    example_nodes::BaseMotionResult result;
    result.success = success;
    action_server_->setSucceeded(result); // TODO: set failed?
  }

  void setActionServer(actionlib::SimpleActionServer<example_nodes::BaseMotionAction>* action_server) {
    action_server_ = action_server;
  }

private:
  OmniBase& base_; 

  actionlib::SimpleActionServer<example_nodes::BaseMotionAction>* action_server_ {nullptr};
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "Rosie_base_node");
  ros::NodeHandle node;

  /////////////////// Initialize base ///////////////////

  // Create base and plan initial trajectory
  auto base = hebi::ros::OmniBase::create(
    {"Rosie"},                         // Family
    {"_Wheel1", "_Wheel2", "_Wheel3"}, // Names
    ros::Time::now().toSec());         // Starting time (for trajectory)
  if (!base) {
    ROS_ERROR("Could not initialize base! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::BaseNode base_node(*base);

  // Action server for base motions
  actionlib::SimpleActionServer<example_nodes::BaseMotionAction> base_motion_action(
    node, "/rosie/base_motion",
    boost::bind(&hebi::ros::BaseNode::startBaseMotion, &base_node, _1), false);

  base_node.setActionServer(&base_motion_action);

  base_motion_action.start();

  /////////////////// Main Loop ///////////////////

  double t_now;

  // Main command loop
  while (ros::ok()) {

    auto t = ros::Time::now().toSec();

    // Update feedback, and command the base to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!base->update(t))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
