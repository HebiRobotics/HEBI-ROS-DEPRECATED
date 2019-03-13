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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>

using namespace hebiros;

// Global Variables
sensor_msgs::JointState feedback;
sensor_msgs::JointState commands;
double rate_of_command = 100;
double buffer = 0;
bool feedback_init = false;


void feedback_callback(sensor_msgs::JointState data) {
  /* Callback function which keeps up to date with feedback from modules */
  feedback = data;
  feedback_init = true;
}

//function to update the odometry pose values
void updatePose(nav_msgs::Odometry& odom, double dx_global, double dy_global, geometry_msgs::Quaternion& odom_quat){

  //Odom pose values are set here
  odom.pose.pose.position.x += dx_global;
  odom.pose.pose.position.y += dy_global;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
}

//function to update the odometry velocity values
void updateTwist(nav_msgs::Odometry& odom, double dx_local, double dy_local, double theta_local, double dt){

  //odom linear velocity values are set here
  odom.twist.twist.linear.x = dx_local / dt;
  odom.twist.twist.linear.y = dy_local / dt;
  odom.twist.twist.linear.z = 0.0;

  //odom angular velocity values are set here
  odom.twist.twist.angular.z = theta_local / dt;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.x = 0.0;
}

//function to update the odometry transform values
void updateOdomTransform(geometry_msgs::TransformStamped& odom_trans, nav_msgs::Odometry& odom, geometry_msgs::Quaternion& odom_quat){

  //Set the odometry -> base_footprint transform values here
  odom_trans.transform.translation.x = odom.pose.pose.position.x;
  odom_trans.transform.translation.y = odom.pose.pose.position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
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

  //Create a publisher to post the calculated odometry to a topic
  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 100);

  //Create a broadcaster for the TF for odom -> base_link
  tf::TransformBroadcaster odom_broadcaster;
  
  feedback.position.resize(3);
  commands.velocity.resize(3);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                             ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Declare  variables to be used for calculations */
  double theta_global = 0;
  double dt = 1;
  sensor_msgs::JointState prevPose;
  sensor_msgs::JointState currPose;
  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;
  ros::Time current_time, last_time;
  bool startup_complete = false;

  /*Parameter setup for NAV and Geometry messages   */
  //Odom message setup; stamp and frame id assignment
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  //Odom transform message setup; stamp and frame id assignment   
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  //initial Quaternion values
  odom_quat.x = 0.0; odom_quat.y = 0.0; odom_quat.z = 0.0; odom_quat.w = 1.0;

  /* Base dimensions */
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (radius from base center to wheel center)

  /******** MAIN LOOP **********/

   while (ros::ok()) {

    if ( !startup_complete ) { 
      /* Get the initial position of the wheels at startup */
      if (feedback_init == true) { 
        prevPose = feedback;
        current_time = ros::Time::now();  //set initial time
        startup_complete = true;
      }

    } else {
      /* get the latest pose for the base */
      currPose = feedback;

      ros::spinOnce(); 
      last_time = current_time;
      current_time = ros::Time::now();
      dt = current_time.toSec() - last_time.toSec();

      //Odom message and transform time stamps
      odom.header.stamp = current_time;
      odom_trans.header.stamp = current_time;

      /* Determine change in position for each individual wheel */
      /* Units: meters */
      double distance_wheel1 = (currPose.position[0] - prevPose.position[0]) * wheelRadius; //wheel1
      double distance_wheel2 = (currPose.position[1] - prevPose.position[1]) * wheelRadius; //wheel2
      double distance_wheel3 = (currPose.position[2] - prevPose.position[2]) * wheelRadius; //wheel3


      /* Use an average from all wheels to determine change in body angle */
      /* Units: Radians*/
      double theta_wheel1 = distance_wheel1 / (baseRadius);
      double theta_wheel2 = distance_wheel2 / (baseRadius);
      double theta_wheel3 = distance_wheel3 / (baseRadius);

      //Determine theta (change in angle)
      double theta_local = (theta_wheel1 + theta_wheel2 + theta_wheel3) / -3.0; 
      theta_global += theta_local;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      odom_quat = tf::createQuaternionMsgFromYaw(theta_global);

      /* Determine movement of the base in current frame of reference */
      /* Units: Meters*/
      double dx_local = (distance_wheel1 * (-sqrt(3)/2) + distance_wheel2 * (sqrt(3)/2)) * 2/3;
      double dy_local = (distance_wheel1/2 + distance_wheel2/2 - distance_wheel3) * -2/3; 

      /* Map movement into the original frame of reference */
      /* Units: Meters*/
      double dx_global = -1 * dy_local * sin(theta_global) + dx_local * cos(theta_global);
      double dy_global = dy_local * cos(theta_global) + dx_local * sin(theta_global);


      //Function call to update Odom pose values
      updatePose(odom, dx_global, dy_global, odom_quat);

      //Function call to update Odom velocity values
      updateTwist(odom, dx_local, dy_local, theta_local, dt);

      //Function call to update Odometry transform values
      updateOdomTransform(odom_trans, odom, odom_quat);
      

      //publish odometry transform messaage
      odom_broadcaster.sendTransform(odom_trans);


      //publish the odometry message
      odom_pub.publish(odom);

      /* Store the current pose for comparison to next recorded pose */
      prevPose = currPose;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

