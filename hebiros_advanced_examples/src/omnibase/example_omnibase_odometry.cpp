/**
* Provides a base level implementation of odometry calculations for a 3-wheeled
* omnidirectional base (omnibase). This file works in the hebiros setup, and 
* publishes odometry results to an odometry node.
*
* @author Hardik Singh < hardik @ hebirobotics.com >
* @since 6 Jul 2018
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"
#include "hebiros/FeedbackMsg.h"

#include <ros/console.h>

using namespace hebiros;

// Global Variables
sensor_msgs::JointState feedback;
sensor_msgs::JointState commands;
double rate_of_command = 60;
double buffer = 0;
bool feedback_init = false;


void feedback_callback(sensor_msgs::JointState data) {
  /* Callback function which keeps up to date with feedback from modules */
  feedback = data;
  feedback_init = true;
}


int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "example_omnibase_odometry");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Update this with the group name for your modules */
  std::string group_name = "omniGroup";

  // Create a subscriber to receive feedback from group
  // Register a callback that keeps the feedback updated
  ros::Subscriber feedback_subscriber = node.subscribe(
    "/hebiros/"+group_name+"/feedback/joint_state", 100, feedback_callback);

  // Create a publisher to post the calculated odometry to a topic
  ros::Publisher odometry_publisher = node.advertise<geometry_msgs::Twist>(
    "/hebiros/"+group_name+"/odometry", 100);
  
  feedback.position.resize(3);
  commands.velocity.resize(3);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                             ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Declare  variables to be used for calculations */
  double d0; double d1; double d2;
  double dx; double dy; 
  double dtheta0; double dtheta1; double dtheta2;
  sensor_msgs::JointState prevPose;
  sensor_msgs::JointState currPose;
  geometry_msgs::Twist output;
  bool startup_complete = false;

  /* Base dimensions */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (radius from base center to wheel center)

  /******** MAIN LOOP **********/

   while (ros::ok()) {

    if ( !startup_complete ) { 
      /* Get the initial position of the wheels at startup */
      if (feedback_init == true) { 
        prevPose = feedback;
        startup_complete = true;
      }

    } else {
      /* get the latest pose for the base */
      currPose = feedback;

      /* Determine change in position for each individual wheel */
      /* Units: meters */
      d0 = (currPose.position[0] - prevPose.position[0]) * wheelRadius; //wheel1
      d1 = (currPose.position[1] - prevPose.position[1]) * wheelRadius; //wheel2
      d2 = (currPose.position[2] - prevPose.position[2]) * wheelRadius; //wheel3


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

