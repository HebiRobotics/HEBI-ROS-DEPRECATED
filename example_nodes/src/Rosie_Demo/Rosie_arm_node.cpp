#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <example_nodes/TargetWaypoints.h>
#include <example_nodes/State.h>
// #include "arm_trajectory.hpp"

#include "arm.hpp"
#include "robot_model.hpp"
#include "group_command.hpp"

#include <ros/console.h>





namespace hebi {

// Global Variables
// Eigen::Vector3d end_tip;
// end_tip << 0, 0, 0;

Eigen::Vector3d get_end_tip(Eigen::Vector3d xyz) {
  Eigen::Vector3d output;

  double z = xyz[2];
  if (z <= 0.5) {
    output << 0, 0, -1;
  }

  else {
    output << 0, -1, 0; // this is pretty much only for handoff in this case
  }

  // ROS_INFO("Get End tip fn output: %lg %lg %lg", output[0], output[1], output[2]);
  return output;
}


  namespace ros {

    class ArmNode {
    public:
      ArmNode(arm::Arm& arm)
        : arm_(arm)
      { }


      bool arm_state = false;

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
         
        Eigen::Vector3d end_tip;
        end_tip << 0, 0, -1;

        Eigen::VectorXd ik_result_joint_angles =
          arm_.getKinematics().solveIK(arm_.getLastFeedback().getPosition(),
          target_xyz_, end_tip);

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
        // Eigen::VectorXd last_position = arm_.getLastFeedback().getPosition();
        Eigen::VectorXd last_position = arm_.getLastFeedback().getPositionCommand();

// ROS_INFO("Here: %lg %lg %lg", last_position[0], last_position[1], last_position[2]);

        // Get joint angles to move to each waypoint
        for (size_t i = 0; i < num_waypoints; ++i) {
          const auto& waypoint = target_waypoints.waypoints_vector[i];

          // Eigen::Vector3d xyz(waypoint.x, waypoint.y, waypoint.z);
          Eigen::Vector3d xyz;
          Eigen::Vector3d end_tip;

          if (waypoint.x == 100 && waypoint.y == 100 && waypoint.z == 100) {
            xyz = arm_.getHomePositionXYZ();
            end_tip << 1, 0, 0;
          } 

          else if (waypoint.x == 101 && waypoint.y == 101 && waypoint.z == 101) {
            xyz = arm_.getHomePositionXYZ();
            end_tip << 0, 0, -1;
          } 

          else {
            xyz << waypoint.x, waypoint.y, waypoint.z;
            end_tip = get_end_tip(xyz);
          }


          // Find the joint angles for the next waypoint, starting from the last
          // waypoint
          // ROS_INFO("End_tip: %lg %lg %lg", end_tip[0], end_tip[1], end_tip[2]);
          last_position = arm_.getKinematics().solveIK(last_position, xyz, end_tip);

          // Save the waypoints
          positions.col(i) = last_position; 
        }

        // Replan:
        arm_.getTrajectory().replan(
          ::ros::Time::now().toSec(),
          arm_.getLastFeedback(),
          positions);
        arm_state = true;
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
  ros::init(argc, argv, "Rosie_arm_node");
  ros::NodeHandle node;

  /////////////////// Initialize arm ///////////////////

  // Create robot model
  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using BracketType = hebi::robot_model::RobotModel::BracketType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;

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

  // Load the approriate gains file
  hebi::GroupCommand gains_cmd(arm -> size());
  gains_cmd.readGains("/home/hebi/catkin_ws/src/HEBI-ROS/example_nodes/include/gains/6-DoF-Arm-Gains-Rosie.xml");
  arm -> getGroup() -> sendCommand(gains_cmd);

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::ArmNode arm_node(*arm);

  // Subscribe to lists of (x, y, z) waypoints
  ros::Subscriber waypoint_subscriber =
    node.subscribe<example_nodes::TargetWaypoints>("cartesian_waypoints", 50, &hebi::ros::ArmNode::updateCartesianWaypoints, &arm_node);

  // Publish completion of waypoint movement
  ros::Publisher state_publisher = 
    node.advertise<example_nodes::State>("/demo/arm_state", 60);

  /////////////////// Main Loop ///////////////////

  double t_now;
  std::shared_ptr<hebi::trajectory::Trajectory> arm_traj;
  // hebi::arm::ArmTrajectory arm_traj;

  // Main command loop
  while (ros::ok()) {

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update(ros::Time::now().toSec()))
      ROS_WARN("Error Getting Feedback -- Check Connection");



    /* Short bit of code to publish true when the arm is done with a motion */
    arm_traj = (arm -> getTrajectory().getTraj());
    t_now = std::min(ros::Time::now().toSec() - arm -> getTrajectory().getTrajStartTime(),
                                      arm_traj -> getDuration());

    if (t_now == arm_traj -> getDuration()) {
      if (arm_node.arm_state) {
        // bool output = arm_node.arm_state;
        state_publisher.publish(arm_node.arm_state);
        arm_node.arm_state = false;
      }
    }


    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
