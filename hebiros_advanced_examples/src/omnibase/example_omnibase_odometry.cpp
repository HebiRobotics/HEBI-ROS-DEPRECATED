#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"
#include "hebiros/FeedbackMsg.h"

#include <ros/console.h>

using namespace hebiros;

// Global Variables
sensor_msgs::JointState feedback;
sensor_msgs::JointState commands;
std::array<double, 3> finalPose; // [linear.x linear.y angular.z]
double rate_of_command = 60;
double buffer = 0;
bool feedback_init = false;

void feedback_callback(sensor_msgs::JointState data) {
	/* Callback function which keeps up to date with feedback from modules */
	feedback = data;
	feedback_init = true;
}

void command_callback(sensor_msgs::JointState data) {
	/* Callback function that keeps up to date with the commands being sent */
	commands = data;
}

int main(int argc, char ** argv) {

	// Initialize ROS node
	ros::init(argc, argv, "example_omnibase_odometry");

	ros::NodeHandle node;

	ros::Rate loop_rate(rate_of_command);

	////////////////////////////////////////////////////////////////////////////
	////////									 HEBI API SETUP 	  								 			////////
	////////////////////////////////////////////////////////////////////////////

	/* Update this with the group name for your modules */
	std::string group_name = "omniGroup";

	// Create a subscriber to receive feedback from group
	// Register a callback that keeps the feedback updated
	ros::Subscriber feedback_subscriber = node.subscribe(
		"/hebiros/"+group_name+"/feedback/joint_state", 100, feedback_callback);

	// Create a subscriber to keep track of what commands are being sent to group
	ros::Subscriber command_subscriber = node.subscribe(
		"/hebiros/"+group_name+"/command/joint_state", 100, command_callback);

	// Create a publisher to post the calculated odometry to a topic
	ros::Publisher odometry_publisher = node.advertise<geometry_msgs::Twist>(
		"/hebiros/"+group_name+"/odometry", 100);
	
	feedback.position.resize(3);
	commands.velocity.resize(3);

	////////////////////////////////////////////////////////////////////////////
	////////									 HEBI SETUP END				 										////////
	////////////////////////////////////////////////////////////////////////////

	/* Declare some variables to be used for calculations */
	double d0; double d1;	double d2;
	double dx; double dy; std::array<double,3> theta;
	geometry_msgs::Twist output;
	sensor_msgs::JointState prevPose;
	sensor_msgs::JointState currPose;

	/* Base dimensions */
	double wheelRadius = 0.0762; // m
	double baseRadius = 0.235; // m, radius from base center to wheel center

  /******** MAIN LOOP **********/

 	while (ros::ok()) {

 		if (buffer < 0.5 * rate_of_command) { 
 			/* buffertime to get initial state (pose) for base */
 			/* roughly 1/2 a second */
 			prevPose = feedback;
 			buffer++;
 		} else {
 			/* get the latest pose for the base */
			currPose = feedback;

			/* determine the change in position for each individual wheel (radians) */
		  d0 = currPose.position[0] - prevPose.position[0]; // wheel 1
			d1 = currPose.position[1] - prevPose.position[1]; // wheel 2
			d2 = currPose.position[2] - prevPose.position[2]; // wheel 3

	    if (commands.velocity[0] == commands.velocity[1] 
								&& commands.velocity[1] == commands.velocity[2] 
								&& commands.velocity[0] != 0) {	
	    	/* MODE: The robot is rotating, in spot */
	    	theta[0] = theta[0] + d0 / (baseRadius/wheelRadius);
	    	theta[1] = theta[1] + d1 / (baseRadius/wheelRadius);
	    	theta[2] = theta[2] + d2 / (baseRadius/wheelRadius);

	    	/* Use the average of the three angles to get estimate of  body angle */
	    	finalPose[2] = (theta[0] + theta[1] + theta[2]) / 3;	

	    	/* Store the current pose for comparison to next pose recorded*/
	    	prevPose = currPose;
	    	
  		} else {
  			/* MODE: The robot is moving across the floor */

	    	/* Determine movement of the base in current frame of reference */
	    	dx = (-d2); 	    	
	    	dy = (d1 / (sqrt(3)/2));

	    	/* Map movement into the original frame of reference */
	    	finalPose[0] = finalPose[0]
					    					+ dy * sin(finalPose[2])
					    				 	+ dx * cos(finalPose[2]); // x co-ordinate

	    	finalPose[1] = finalPose[1]
												+ dy * cos(finalPose[2])
						    				- dx * sin(finalPose[2]); // y co-ordinate

		   	/* Store the current pose for comparison to next pose recorded */
		    prevPose = currPose;

			}


	    /* Broadcast the latest odomoetry estimate in Twist format */
	    /* Units: meters */
	    output.linear.x = finalPose[0] * wheelRadius;
	    output.linear.y = finalPose[1] * wheelRadius;
	    output.angular.z = finalPose[2];
	    odometry_publisher.publish(output);
		}

	  ros::spinOnce();
  	loop_rate.sleep();
	}

	return 0;
}




 











