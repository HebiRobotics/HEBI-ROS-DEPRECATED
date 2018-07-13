/**
* Provides an implentation of user controls for a 3-wheeled omnidirectional 
* base (omnibase). This example uses the HEBI trajectory generator to move the
* omnibase more smoothly.
* You can move forward/backward, left/right, or rotate in spot.
* 
* @author Hardik Singh < hardik@hebirobotics.com >
* @since 13 Jul 2018
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "hebi.h"
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"

#include <ros/console.h>

using namespace hebi;

// Global Variables
int num_wheels = 3;
geometry_msgs::Twist directions;
Eigen::VectorXd omniVels(num_wheels); 
Eigen::VectorXd omniPos(num_wheels);
Eigen::VectorXd omniEfforts(num_wheels);
bool keys_init = false;
double rate_of_command = 60;
double speed = 0.3; // m/s
double wheelRotSpeed = M_PI/3; // (rad/s)

// Trajectory items
Eigen::VectorXd vel_traj(num_wheels);
Eigen::VectorXd accel_traj(num_wheels);
Eigen::VectorXd jerk_traj(num_wheels);
Eigen::VectorXd desired_vel(num_wheels);



void directions_callback(geometry_msgs::Twist data) {
  /* Callback function that keeps up to date with key presses and commands */
  directions = data;
  keys_init = true;
}


void updateOmniVels() {
  /* Declare main kinematic variables */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (center of omni to origin of base)
  double ratio = sqrt(3)/2;

  // Wheel 1, front right
  omniVels[0] = vel_traj[0] * 0.5/wheelRadius + 
                vel_traj[1] * -ratio/wheelRadius +
                vel_traj[2] * baseRadius/wheelRadius;

  // Wheel 2, front left
  omniVels[1] = vel_traj[0] * 0.5/wheelRadius + 
                vel_traj[1] * ratio/wheelRadius +
                vel_traj[2] * baseRadius/wheelRadius;

  // Wheel 3, back center
  omniVels[2] = vel_traj[0] * -1/wheelRadius + 
                0 +
                vel_traj[2] * baseRadius/wheelRadius;
}


void updatePoseCmd() { 
  /* We assume that we have access to: */
  // - feedback from modules 
  // - omniVels; the velocities being commanded to each module 
  // - rate-of-command; in hertz

  omniPos[0] = omniPos[0] + omniVels[0] * (1/rate_of_command);
  omniPos[1] = omniPos[1] + omniVels[1] * (1/rate_of_command);
  omniPos[2] = omniPos[2] + omniVels[2] * (1/rate_of_command);
}

void updateOmniEfforts() {
  double chassisMass = 12; //kg
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (center of omni to origin of base)
  double ratio = sqrt(3)/2;

  // Wheel1, front right
  omniEfforts[0] = accel_traj[0] * chassisMass * 0.5 * wheelRadius + 
                   accel_traj[1] * chassisMass * -ratio * wheelRadius + 
                   accel_traj[2] * baseRadius * wheelRadius;

  omniEfforts[1] = accel_traj[0] * chassisMass * 0.5 * wheelRadius + 
                   accel_traj[1] * chassisMass * ratio * wheelRadius + 
                   accel_traj[2] * baseRadius * wheelRadius;

  omniEfforts[2] = accel_traj[0] * chassisMass * -1 * wheelRadius + 
                   0 + 
                   accel_traj[2] * baseRadius * wheelRadius;
}



int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "omnibase_node_trajectory");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ros::Subscriber key_subscriber = node.subscribe("keys/cmd_vel", 20,
                           directions_callback);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Update this with the group name for your modules */
  std::string group_name = "omniGroup";

  //Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({"Rosie"}, 
                                  {"_Wheel1", "_Wheel2", "_Wheel3"});

  if (!group)
  {
    std::cout << "Group not found! Shutting Down...\n";
    return -1;
  }

  GroupCommand group_command(group -> size());
  group_command.readGains("omnibase_gains.xml");
  GroupFeedback feedback(group -> size());
  group -> setFeedbackFrequencyHz(rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                           ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** Trajectory Setup **********/

  double rampTime = 0.33;
  Eigen::VectorXd omniBaseTrajTime(2);
  omniBaseTrajTime << 0, rampTime;

  Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_wheels, 2);
  Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_wheels, 2);
  Eigen::MatrixXd jerks = Eigen::MatrixXd::Zero(num_wheels, 2);

  std::shared_ptr<hebi::trajectory::Trajectory> trajectory;
  ros::Time trajStartTime;

  /******** MAIN LOOP **********/
  bool startup_complete = false;

  while (ros::ok()) {
    if ( !startup_complete ) {
      /* Ensure that we are receiving feedback and key commands */
      /* Store the initial pose of the omnibase in omniPos */
      if (group->getNextFeedback(feedback) && keys_init == true) {
        // grab initial feedback
        omniPos = feedback.getPosition();
        // initialise trajectory object
        trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
                        omniBaseTrajTime, velocities, &accelerations, &jerks);
        trajStartTime = ros::Time::now();
        startup_complete = true;
      }
    } else {

      /* Get new body velocities from trajectory generator */
      double t = std::min((ros::Time::now() - trajStartTime).toSec(),
                                                   trajectory -> getDuration());
      trajectory -> getState(t, &vel_traj, &accel_traj, &jerk_traj);

      /* Convert body velocities to wheel velocities */
      updateOmniVels();
      updatePoseCmd(); 
      updateOmniEfforts();

      /* Set the new positions, velocities, and efforts */
      group_command.setVelocity(omniVels);
      group_command.setPosition(omniPos);
      group_command.setEffort(omniEfforts);

      /* Send the new commands */
      group -> sendCommand(group_command);

      /* Re-evaluate trajectory state */
      desired_vel[0] = directions.linear.x * speed;
      desired_vel[1] = directions.linear.y * speed;
      desired_vel[2] = directions.angular.z * wheelRotSpeed;

      velocities << vel_traj[0], desired_vel[0],
                    vel_traj[1], desired_vel[1],
                    vel_traj[2], desired_vel[2];

      accelerations << accel_traj[0], 0,
                       accel_traj[1], 0,
                       accel_traj[2], 0;

      jerks << jerk_traj[0], 0,
               jerk_traj[1], 0,
               jerk_traj[2], 0;

      trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
                        omniBaseTrajTime, velocities, &accelerations, &jerks);
      trajStartTime = ros::Time::now();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

