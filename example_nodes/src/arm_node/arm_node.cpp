#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <example_nodes/TargetWaypoints.h>

#include "arm.hpp"
#include "robot_model.hpp"

#include <ros/console.h>

namespace hebi {
  namespace ros {

    class ArmNode {
    public:
      ArmNode(arm::Arm& arm)
        : arm_(arm)
      { }

      // "Jog" the target end effector location in (x,y,z) space, replanning
      // smoothly to the new location
      void offsetTargetCallback(geometry_msgs::Point data) {
        
        // Only update if target changes!
        if (data.x == 0 && data.y == 0 && data.z == 0)
          return;

        // Initialize target from feedback as necessary
        if (!isTargetInitialized()) {
          auto pos = arm_.getKinematics().FK(arm_.getLastFeedback().getPosition());
          target_xyz_.x() = pos.x();
          target_xyz_.y() = pos.y();
          target_xyz_.z() = pos.z();
        }

        // Update the target point
        target_xyz_.x() += data.x / 100.0; // Converts to cm
        target_xyz_.y() += data.y / 100.0; // Converts to cm
        target_xyz_.z() += data.z / 100.0; // Converts to cm
         
        Eigen::VectorXd ik_result_joint_angles =
          arm_.getKinematics().solveIK(arm_.getLastFeedback().getPosition(),
          target_xyz_);

        // Replan:
        arm_.getTrajectory().replan(
          ::ros::Time::now().toSec(),
          arm_.getLastFeedback(),
          ik_result_joint_angles);
      }

      // Replan a smooth joint trajectory from the current location through a
      // series of cartesian waypoints.
      void updateCartesianWaypoints(example_nodes::TargetWaypoints target_waypoints) {
        size_t num_waypoints = target_waypoints.waypoints_vector.size();

        // These are the joint angles that will be added
        Eigen::MatrixXd positions(arm_.size(), num_waypoints);

        // Plan to each subsequent point from the last position
        Eigen::VectorXd last_position = arm_.getLastFeedback().getPosition();

        // Get joint angles to move to each waypoint
        for (size_t i = 0; i < num_waypoints; ++i) {
          const auto& waypoint = target_waypoints.waypoints_vector[i];

          Eigen::Vector3d xyz(waypoint.x, waypoint.y, waypoint.z);

          // Find the joint angles for the next waypoint, starting from the last
          // waypoint
          last_position = arm_.getKinematics().solveIK(last_position, xyz);

          // Save the waypoints
          positions.col(i) = last_position; 
        }

        // Replan:
        arm_.getTrajectory().replan(
          ::ros::Time::now().toSec(),
          arm_.getLastFeedback(),
          positions);
      }

    private:
      arm::Arm& arm_;
 
      // The end effector location that this arm will target (NaN indicates
      // unitialized state, and will be set from feedback during first offset)
      Eigen::Vector3d target_xyz_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
      bool isTargetInitialized() {
        return !std::isnan(target_xyz_.x()) ||
               !std::isnan(target_xyz_.y()) ||
               !std::isnan(target_xyz_.z());
      }

    };

  }
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle node;

  /////////////////// Initialize arm ///////////////////

  // Create robot model
  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;

  hebi::robot_model::RobotModel model;
  model.addActuator(ActuatorType::X5_9);
  model.addBracket(BracketType::X5HeavyLeftOutside );
  model.addActuator(ActuatorType::X5_9);
  model.addLink(LinkType::X5, 0.325, M_PI);
  model.addActuator(ActuatorType::X5_9);
  model.addLink(LinkType::X5, 0.325, M_PI);
  model.addActuator(ActuatorType::X5_4);
  model.addLink(LinkType::X5, 0.175, 0);
  hebi::arm::ArmKinematics arm_kinematics(model);

  Eigen::VectorXd home_position(model.getDoFCount());
  home_position << 0, -M_PI*2/5, -M_PI*4/5, -M_PI_2;

  // Create arm and plan initial trajectory
  auto arm = hebi::arm::Arm::createArm(
    {"4-DoF Arm"},                          // Family
    {"Base", "Shoulder", "Elbow", "Wrist"}, // Names,
    home_position,                          // Home position
    arm_kinematics,                         // Kinematics object
    ros::Time::now().toSec());              // Starting time (for trajectory)  

  if (!arm) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::ArmNode arm_node(*arm);

  ros::Subscriber key_subscriber =
    node.subscribe<geometry_msgs::Point>("keys/cmd_vel", 50, &hebi::ros::ArmNode::offsetTargetCallback, &arm_node);

  // Subscribe to lists of (x, y, z) waypoints
  ros::Subscriber waypoint_subscriber =
    node.subscribe<example_nodes::TargetWaypoints>("cartesian_waypoints", 50, &hebi::ros::ArmNode::updateCartesianWaypoints, &arm_node);

  /////////////////// Main Loop ///////////////////

  // Main command loop
  while (ros::ok()) {

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update(ros::Time::now().toSec()))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
