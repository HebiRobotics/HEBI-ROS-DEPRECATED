
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "hebi.h"
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"
#include <example_nodes/TargetWaypoints.h>
#include <example_nodes/State.h>


#include <ros/console.h>

using namespace hebi;

// Global Variables
int num_wheels = 3;
bool keys_init = true;
double rate_of_command = 60;
double speed = 0.3; // m/s
double wheelRotSpeed = M_PI/3; // (rad/s)

//inputs
geometry_msgs::Point target_pos;
bool base_state = false;
bool arm_state = false;
bool gripper_state = false;

//middle
int main_state = 0;
bool cmd_received = false;
geometry_msgs::Point base_target;
geometry_msgs::Point arm_target;
double offset = 1;
bool gripper_cmd = false; // true if closed
ros::Time grip_start_time;
              // grip_cmd

//outputs
geometry_msgs::Twist cmd_vel;
example_nodes::TargetWaypoints arm_cmd;
geometry_msgs::Point arm_handoff;


void target_callback(geometry_msgs::Point data) {
  target_pos = data;
  if (target_pos.x != 0 || target_pos.y != 0 || target_pos.z != 0) {
    cmd_received = true;
  }
}

void base_state_callback(example_nodes::State data) {
  base_state = data.state;
}

void arm_state_callback(example_nodes::State data) {
  arm_state = data.state;
}

void gripper_state_callback(example_nodes::State data) {
  gripper_state = data.state;
}




int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "omnibase_node_trajectory");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  // INPUT
  ros::Subscriber key_subscriber = node.subscribe("/demo/target", rate_of_command, target_callback);
  // ros::Subscriber odom_subscriber = node.subscribe("omnibase_node/odometry", rate_of_command, odom_callback);
  ros::Subscriber base_state_subscriber = node.subscribe("/demo/base_state", rate_of_command, base_state_callback);
  ros::Subscriber arm_state_subscriber = node.subscribe("/demo/arm_state", rate_of_command, arm_state_callback);
  ros::Subscriber gripper_state_subscriber = node.subscribe("/demo/gripper_state", rate_of_command, gripper_state_callback);

  // OUTPUT
  ros::Publisher omni_publisher = node.advertise<geometry_msgs::Twist>("/demo/cmd_vel", rate_of_command);
  ros::Publisher arm_publisher = node.advertise<example_nodes::TargetWaypoints>("cartesian_waypoints", rate_of_command);
  ros::Publisher grip_publisher = node.advertise<example_nodes::State>("/demo/gripper_cmd", rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** MAIN LOOP **********/
  bool startup_complete = false;
  geometry_msgs::Twist curr_pos;

  // Control booleans
  bool cmd_processed = false;
  bool reached_pickup = false;
  bool picked_up = false;
  bool moving_to_handoff = false;
  bool handoff_done = false;
  bool returning_home = false;
  // geometry_msgs::Twist next_pos;

  while (ros::ok()) {
    if ( !startup_complete ) {
      /* Ensure that we are receiving key commands */
      if (keys_init == true) {
        startup_complete = true;
      }
    } else {

      /* Convert Key commands into base and arm commands */

      if (cmd_received) {
        cmd_received = false;

        if (target_pos.x == 0 && target_pos.y == 0) {
        //should never happen but is happening! Somewhere in key_read_rosie I think
          offset = 1;
        } else {
          offset = 1-(0.3/(sqrt(pow(target_pos.x,2) + pow(target_pos.y,2)))); 
          //multiplier
        }
        cmd_vel.linear.x = target_pos.x * offset;
        cmd_vel.linear.y = target_pos.y * offset;

        omni_publisher.publish(cmd_vel);

        // Looks weird because x and y are rotated from omnibase FoR
        if (offset != 1) {
          // arm_target.x = target_pos.x - cmd_vel.linear.x;
          // arm_target.y = target_pos.y - cmd_vel.linear.y;
          arm_target.x = target_pos.y - cmd_vel.linear.y;
          arm_target.y = - (target_pos.x - cmd_vel.linear.x);
          arm_target.z = target_pos.z;
        }

        cmd_processed = true;
      }


      if (cmd_processed && base_state) { // the base has finished executing
        cmd_processed = false;
        base_state = false;

        // ROS_INFO("%lg %lg %lg", arm_target.x, arm_target.y, arm_target.z);

        arm_cmd.waypoints_vector = {arm_target};
        arm_publisher.publish(arm_cmd);

        reached_pickup = true;
      }


      // the arm has finished executing
      if (reached_pickup && arm_state) { 
        reached_pickup = false;
        arm_state = false;

        gripper_cmd = true;
        grip_publisher.publish(gripper_cmd);
        // grip_start_time = ros::Time::now();

        picked_up = true;
      }

      // telling it to be close, and it finished closing
      if (picked_up && gripper_state) { 
        picked_up = false;
        arm_handoff.x = -0.2;
        arm_handoff.y = -0.4;
        arm_handoff.z = 0.5;
        arm_cmd.waypoints_vector = {arm_handoff};
        arm_publisher.publish(arm_cmd);
        moving_to_handoff = true;
      }

      // gripper closed, arm has moved to handoff
      if (moving_to_handoff && gripper_state && arm_state) { 
        moving_to_handoff = false;
        arm_state = false;

        gripper_cmd = false;
        grip_publisher.publish(gripper_cmd);
        handoff_done = true;
      }

      if (handoff_done && !gripper_state) {
        handoff_done = false;
        geometry_msgs::Point arm_reset;
        
        arm_reset.x = 100;
        arm_reset.y = 100;
        arm_reset.z = 100;
        
        arm_cmd.waypoints_vector = {arm_reset};
        arm_publisher.publish(arm_cmd);

        returning_home = true;
      }

      if (returning_home && arm_state) {
        returning_home = false;
        arm_state = false;
      }


    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

