/**
* Provides a base level implentation of user controls for a 3-wheeled 
* omnidirectional base (omnibase).
* You can move forward/backward, left/right, or rotate in spot.
* 
* @author Hardik Singh < hardik @ hebirobotics.com >
* @since 10 Jul 2018
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <chrono>
#include <thread>
#include "hebi.h"
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"

#include <ros/console.h>

using namespace hebi;

// Global Variables
geometry_msgs::Twist directions;
Eigen::VectorXd omniVels(3); 
Eigen::VectorXd omniPos(3);
double rate_of_command = 60;
bool keys_init = false;


void directions_callback(geometry_msgs::Twist data) {
  /* Callback function that keeps up to date with key presses and commands */
  directions = data;
  keys_init = true;
}


void updateOmniVels() {
  /* Declare main kinematic variables */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (center of omni to origin of base)
  double speed = 0.3; // m/s
  double wheelRotSpeed = M_PI/3; // (rad/s)
  double ratio = sqrt(3)/2;

  // Wheel 1, front right
  omniVels[0] = directions.linear.x * speed * 0.5/wheelRadius + 
                directions.linear.y * speed * -ratio/wheelRadius +
                directions.angular.z * wheelRotSpeed * baseRadius/wheelRadius;

  // Wheel 2, front left
  omniVels[1] = directions.linear.x * speed * 0.5/wheelRadius + 
                directions.linear.y * speed * ratio/wheelRadius +
                directions.angular.z *wheelRotSpeed *  baseRadius/wheelRadius;

  // Wheel 3, back center
  omniVels[2] = directions.linear.x * -speed * 1/wheelRadius + 
                0 +
                directions.angular.z * wheelRotSpeed * baseRadius/wheelRadius;
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


int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "omnibase_node");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ros::Subscriber key_subscriber = node.subscribe("keys/cmd_vel", 20,
                           directions_callback);

  // ros::Publisher velocity_publisher = node.advertise<geometry_msgs::Twist>(
  //   "omnibase_node/wheelVelocities", 100);

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
  GroupFeedback feedback(group -> size());
  group -> setFeedbackFrequencyHz(rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                           ////////
  ////////////////////////////////////////////////////////////////////////////


  /******** MAIN LOOP **********/
  bool startup_complete = false;

  while (ros::ok()) {
    if ( !startup_complete ) {
      /* Ensure that we are receiving feedback and key commands */
      /* Store the initial pose of the omnibase in omniPos */
      if (group->getNextFeedback(feedback) && keys_init == true) {
        omniPos = feedback.getPosition();
        startup_complete = true;
      }

    } else {
      // Get the new velocities and position commands
      updateOmniVels();
      updatePoseCmd();

      /* Uncomment to see what velocity commands are being sent */
      //ROS_INFO("%lg %lg %lg", omniVels[0], omniVels[1], omniVels[2]);

      /* Uncomment to see what position commands are being sent */
      //ROS_INFO("%lg %lg %lg", omniPos[0], omniPos[1], omniPos[2]);
      
      // Set the new positions and velocities
      group_command.setVelocity(omniVels);
      group_command.setPosition(omniPos);

      // Send the new commands
      group -> sendCommand(group_command);

      // Publish the commands that were sent
      // velocity_publisher.publish(omniVels);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

