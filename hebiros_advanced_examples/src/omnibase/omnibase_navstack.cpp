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
//  ros::Publisher odometry_publisher = node.advertise<geometry_msgs::Twist>(
//    "/hebiros/"+group_name+"/odometry", 100);
//
//  ros::Publisher odometryMSG_publisher = node.advertise<nav_msgs::Odometry>(
//    "/hebiros/"+group_name+"/odom", 100);

//  ros::Publisher odom_publisher = node.advertise<nav_msgs::Odometry>(
//    "/odom", 100);

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;


  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  
  feedback.position.resize(3);
  commands.velocity.resize(3);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                             ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Declare  variables to be used for calculations */
  double d0; double d1; double d2;
  double dx; double dy; 
  double dtheta0; double dtheta1; double dtheta2; double thetaChange; double thetaTotalChange;
  sensor_msgs::JointState prevPose;
  sensor_msgs::JointState currPose;
  nav_msgs::Odometry odom;
  odom.twist.twist.angular.z = 0;
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
      thetaChange = (dtheta0 + dtheta1 + dtheta2) / 3.0; 
      thetaTotalChange += thetaChange;
      //ROS_INFO_STREAM("TTC " << thetaTotalChange);
      odom.twist.twist.angular.z = thetaChange;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.x = 0;

      /* Determine movement of the base in current frame of reference */
      /* Units: Meters*/
      dx = (d0/2 + d1/2 - d2) * 2/3; 
      dy = (d0 * (-sqrt(3)/2) + d1 * (sqrt(3)/2)) * 2/3;

      /* Map movement into the original frame of reference */
      /* Units: Meters*/
      odom.pose.pose.position.x += dy * sin(thetaChange)
                       + dx * cos(thetaChange); 

      odom.pose.pose.position.y += dy * cos(thetaChange)
                       - dx * sin(thetaChange); 

    //  output.seq++;
    //  output.stamp = ros::Time::now();
    //  output.frame_id = 0;




    
      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(thetaTotalChange);
        
      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = dy * sin(odom.twist.twist.angular.z)
                       + dx * cos(odom.twist.twist.angular.z);

      odom_trans.transform.translation.y = dy * cos(odom.twist.twist.angular.z)
                       - dx * sin(odom.twist.twist.angular.z);

      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      ROS_INFO_STREAM(odom_quat);

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);


    
      //next, we'll publish the odometry message over ROS
    //  nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
   //  odom.pose.pose.position.x = dx;
   //   odom.pose.pose.position.y = dy;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = dx;
      odom.twist.twist.linear.y = dy;
      odom.twist.twist.linear.z = 0.0;

      //publish the message
      odom_pub.publish(odom);



      /* Broadcast the latest odometry estimate in Twist format to node */
  //    odometry_publisher.publish(output);

  //    odometryMSG_publisher.publish(output);

  //    odom_publisher.publish(output);

      /* Store the current pose for comparison to next recorded pose */
      prevPose = currPose;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

