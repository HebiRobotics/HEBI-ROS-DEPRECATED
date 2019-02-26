/**
* Provides a base level implementation of odometry calculations for a 3-wheeled
* omnidirectional base (omnibase) for a 3D enviroment. This file works in the hebiros setup, and 
* publishes odometry results to an odometry node.
*
* @author Sami Mian < sami @ hebirobotics.com >
* @since 2 Feb 2019
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"
#include "hebiros/FeedbackMsg.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
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

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  feedback.position.resize(3);
  commands.velocity.resize(3);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                             ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Declare  variables to be used for calculations */
  double d0; double d1; double d2;
  double dx; double dy; 
  double dtheta0; double dtheta1; double dtheta2; double thetaChange; double thetaTotalChange;
  double xPoseChange; double yPoseChange;
  sensor_msgs::JointState prevPose;
  sensor_msgs::JointState currPose;
  nav_msgs::Odometry odom;

  odom.twist.twist.angular.z = 0;
  geometry_msgs::Twist output;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;
  ros::Time current_time, last_time;
  bool startup_complete = false;

  /* Base dimensions */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.220; // m (radius from base center to wheel center)

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

      ros::spinOnce(); 
      last_time = current_time;
      current_time = ros::Time::now();
      double dt = current_time.toSec() - last_time.toSec();


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

      //Determine theta (change in angle)
      thetaChange = (dtheta0 + dtheta1 + dtheta2) / -3.0; 
      thetaTotalChange += thetaChange;
      //ROS_INFO_STREAM("TTC " << thetaChange);
      odom.twist.twist.angular.z = thetaChange / dt;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.x = 0;

      /* Determine movement of the base in current frame of reference */
      /* Units: Meters*/
      dy = (d0/2 + d1/2 - d2) * -2/3; 
      dx = (d0 * (-sqrt(3)/2) + d1 * (sqrt(3)/2)) * 2/3;

      /* Map movement into the original frame of reference */
      /* Units: Meters*/
      xPoseChange = -1 * dy * sin(thetaTotalChange) + dx * cos(thetaTotalChange);
      yPoseChange = dy * cos(thetaTotalChange) + dx * sin(thetaTotalChange);

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      odom_quat = tf::createQuaternionMsgFromYaw(thetaTotalChange);

      //Odom data message + output
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";

      //Odom pose values are set here
      odom.pose.pose.position.x += xPoseChange;
      odom.pose.pose.position.y += yPoseChange;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.twist.twist.linear.x = dx / dt;
      odom.twist.twist.linear.y = dy / dt;
      odom.twist.twist.linear.z = 0.0;

        
      //first, we'll publish the transform over tf
      
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint";

      //odom TF values set here
      odom_trans.transform.translation.x = odom.pose.pose.position.x;//dy * sin(thetaChange) + dx * cos(thetaChange);
      odom_trans.transform.translation.y = odom.pose.pose.position.y; //dy * cos(thetaChange) - dx * sin(thetaChange);
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      odom_broadcaster.sendTransform(odom_trans);


      //publish the message
      odom_pub.publish(odom);


      /* Store the current pose for comparison to next recorded pose */
      prevPose = currPose;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}