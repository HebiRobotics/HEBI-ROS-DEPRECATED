/**
* Provides a base level implementation of odometry calculations for a 3-wheeled
* omnidirectional base (omnibase). This file works in the hebiros setup, and 
* publishes odometry results to an odometry node.
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
double rate_of_command = 60;

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "omnibase_odometry");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ros::Publisher odometry_publisher = node.advertise<geometry_msgs::Twist>(
  "/omnibase_node/odometry", 100);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                           ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Update this with the group name for your modules */
  std::string group_name = "omniGroup";

  /* Find and group the modules needed. Ensure they're on the network. */
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

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                             ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Declare  variables to be used for calculations */
  double d0; double d1; double d2;
  double dx; double dy; 
  double dtheta0; double dtheta1; double dtheta2;
  Eigen::VectorXd prevPose(3);
  Eigen::VectorXd currPose(3);
  geometry_msgs::Twist output;
  bool startup_complete = false;

  /* Base dimensions */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (radius from base center to wheel center)

  /******** MAIN LOOP **********/

   while (ros::ok()) {

    if ( !startup_complete ) { 
      /* Ensure that we are receiving feedback and key commands */
      /* Get the initial position of the wheels at startup */
      if (group->getNextFeedback(feedback)) { 
        prevPose = feedback.getPosition();
        startup_complete = true;
      }

    } else {
      /* get the latest feedback */
      group -> sendFeedbackRequest();
      group -> getNextFeedback(feedback);
      currPose = feedback.getPosition();

      /* Determine change in position for each individual wheel */
      /* Units: meters */
      d0 = (currPose[0] - prevPose[0]) * wheelRadius; //wheel1
      d1 = (currPose[1] - prevPose[1]) * wheelRadius; //wheel2
      d2 = (currPose[2] - prevPose[2]) * wheelRadius; //wheel3

      /* Use an average from all wheels to determine change in body angle */
      /* Units: Radians*/
      dtheta0 = d0 / (baseRadius);
      dtheta1 = d1 / (baseRadius);
      dtheta2 = d2 / (baseRadius); 
      output.angular.z += (dtheta0 + dtheta1 + dtheta2) / 3;

      /* Determine movement of the base in current frame of reference */
      /* Units: Meters*/
      dx = (d0/2 + d1/2 - d2) * 2/3; 
      dy = (d0 * (-sqrt(3)/2) + d1 * (sqrt(3)/2)) * 2/3;

      /* Map movement into the original frame of reference */
      /* Units: Meters*/
      output.linear.x += dy * sin(output.angular.z)
                       + dx * cos(output.angular.z); 

      output.linear.y += dy * cos(output.angular.z)
                       - dx * sin(output.angular.z); 

      /* Broadcast the latest odometry estimate in Twist format to node */
      odometry_publisher.publish(output);

      /* Store the current pose for comparison to next recorded pose */
      prevPose = currPose;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
