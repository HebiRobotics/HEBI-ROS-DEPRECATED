#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <example_nodes/TargetWaypoints.h>
#include <example_nodes/State.h>

#include <example_nodes/ArmMotionAction.h>

#include "actionlib/server/simple_action_server.h"

#include "arm.hpp"
#include "robot_model.hpp"
#include "group_command.hpp"

#include <ros/console.h>

namespace hebi {
namespace ros {

class ArmNode {
public:
  ArmNode(arm::Arm& arm)
    : arm_(arm)
  {
  }

  void startArmMotion(const example_nodes::ArmMotionGoalConstPtr& goal)
  {

    // Replan a smooth joint trajectory from the current location through a
    // series of cartesian waypoints.
    // TODO: use a single struct instead of 6 single vectors of the same length;
    // but how do we do hierarchial actions?
    size_t num_waypoints = goal->x.size();

    // These are the joint angles that will be added
    Eigen::MatrixXd positions(arm_.size(), num_waypoints);

    // Plan to each subsequent point from the last position
    // Eigen::VectorXd last_position = arm_.getLastFeedback().getPosition();
    Eigen::VectorXd last_position = arm_.getLastFeedback().getPositionCommand();

    // Get joint angles to move to each waypoint
    for (size_t i = 0; i < num_waypoints; ++i) {

      Eigen::Vector3d xyz;
      Eigen::Vector3d end_tip;

      // Special homing values...
      if (goal->x[i] == 100 && goal->y[i] == 100 && goal->z[i] == 100) {
        xyz = arm_.getHomePositionXYZ();
        end_tip << 1, 0, 0;
      } 

      else if (goal->x[i] == 101 && goal->y[i] == 101 && goal->z[i] == 101) {
        xyz = arm_.getHomePositionXYZ();
        end_tip << 0, 0, -1;
      } 

      else {
        xyz << goal->x[i], goal->y[i], goal->z[i];
        end_tip << goal->tipx[i], goal->tipy[i], goal->tipz[i];
      }

      // Find the joint angles for the next waypoint, starting from the last
      // waypoint
      last_position = arm_.getKinematics().solveIK(last_position, xyz, end_tip);

      // Save the waypoints
      positions.col(i) = last_position; 
    }

    // Replan:
    arm_.getTrajectory().replan(
      ::ros::Time::now().toSec(),
      arm_.getLastFeedback(),
      positions);
    arm::Color color;
    if (goal->set_color)
      color = arm::Color(goal->r, goal->g, goal->b);
    arm_.setColor(color);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);
    bool success = true;

    ROS_INFO("Executing arm motion action");

    example_nodes::ArmMotionFeedback feedback;

    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Preempted arm motion");
        action_server_->setPreempted();
        success = false;
        break;
      }

      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      auto& arm_traj = arm_.getTrajectory();
      feedback.percent_complete = arm_.trajectoryPercentComplete(t);
      action_server_->publishFeedback(feedback);
 
      if (arm_.isTrajectoryComplete(t)) {
        ROS_INFO("COMPLETE");
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    // Clear color:
    color = arm::Color();
    arm_.setColor(color);

    // publish when the arm is done with a motion
    ROS_INFO("Completed arm motion action");
    example_nodes::ArmMotionResult result;
    result.success = success;
    action_server_->setSucceeded(result);
  }

  void setActionServer(actionlib::SimpleActionServer<example_nodes::ArmMotionAction>* action_server) {
    action_server_ = action_server;
  }
  
private:
  arm::Arm& arm_;

  actionlib::SimpleActionServer<example_nodes::ArmMotionAction>* action_server_ {nullptr};

};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "Rosie_arm_node");
  ros::NodeHandle node;

  /////////////////// Initialize arm ///////////////////

  // Create robot model
  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;

  // TODO: LOAD FROM HRDF!!!
  hebi::robot_model::RobotModel model;
  model.addActuator(ActuatorType::X8_9);
  model.addBracket(BracketType::X5HeavyRightInside);
  model.addActuator(ActuatorType::X8_16);
  model.addLink(LinkType::X5, 0.325, M_PI);
  model.addActuator(ActuatorType::X8_9);
  model.addLink(LinkType::X5, 0.325, M_PI);
  model.addActuator(ActuatorType::X5_4);
  model.addBracket(BracketType::X5LightRight);
  model.addActuator(ActuatorType::X5_1);
  model.addBracket(BracketType::X5LightRight);
  model.addActuator(ActuatorType::X5_1);
  hebi::arm::ArmKinematics arm_kinematics(model);

  Eigen::VectorXd home_position(model.getDoFCount());
  home_position << 0.01, M_PI*2/3, M_PI*2/3, 0.01, M_PI_2, 0.01; // avoid 0s in the homeposition

  // Create arm and plan initial trajectory
  auto arm = hebi::arm::Arm::createArm(
    {"Rosie"},                          // Family
    {"Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"}, // Names,
    home_position,                          // Home position
    arm_kinematics,                         // Kinematics object
    ros::Time::now().toSec());              // Starting time (for trajectory)  

  if (!arm) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  // Load the appropriate gains file
  // TODO: BETTER PACKAGE THIS FILE!!!
  hebi::GroupCommand gains_cmd(arm -> size());
  gains_cmd.readGains("/home/hebi/catkin_ws/src/HEBI-ROS/example_nodes/include/gains/6-DoF-Arm-Gains-Rosie.xml");
  arm -> getGroup() -> sendCommand(gains_cmd);

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::ArmNode arm_node(*arm);

  // Action server for arm motions
  actionlib::SimpleActionServer<example_nodes::ArmMotionAction> arm_motion_action(
    node, "/rosie/arm_motion",
    boost::bind(&hebi::ros::ArmNode::startArmMotion, &arm_node, _1), false);

  arm_node.setActionServer(&arm_motion_action);

  arm_motion_action.start();

  /////////////////// Main Loop ///////////////////

  double t_now;

  // Main command loop
  while (ros::ok()) {

    auto t = ros::Time::now().toSec();

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update(t))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
