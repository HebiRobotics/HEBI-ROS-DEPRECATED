
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
geometry_msgs::Twist odometry_pos;

//middle
int main_state = 0;
bool cmd_received = false;
geometry_msgs::Point base_target;
geometry_msgs::Point arm_target;
double offset = 1;
example_nodes::State gripper_cmd;
ros::Time grip_start_time;
double travelled_x = 0;
double travelled_y = 0;



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

void odom_callback(geometry_msgs::Twist data) {
  odometry_pos = data;
}



int main(int argc, char ** argv) {

  gripper_cmd.state = false; // true if closed

  // Initialize ROS node
  ros::init(argc, argv, "demo_central");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  // INPUT
  ros::Subscriber key_subscriber = node.subscribe("/demo/target", rate_of_command, target_callback);
  ros::Subscriber odom_subscriber = node.subscribe("omnibase_node/odometry", rate_of_command, odom_callback);
  ros::Subscriber base_state_subscriber = node.subscribe("/demo/base_state", rate_of_command, base_state_callback);
  ros::Subscriber arm_state_subscriber = node.subscribe("/demo/arm_state", rate_of_command, arm_state_callback);
  ros::Subscriber gripper_state_subscriber = node.subscribe("/demo/gripper_state", rate_of_command, gripper_state_callback);

  // OUTPUT
  ros::Publisher omni_publisher = node.advertise<geometry_msgs::Twist>("/demo/cmd_vel", rate_of_command);
  ros::Publisher arm_publisher = node.advertise<example_nodes::TargetWaypoints>("cartesian_waypoints", rate_of_command);
  ros::Publisher grip_publisher = node.advertise<example_nodes::State>("/demo/gripper_cmd", rate_of_command);
  ros::Publisher ready_publisher = node.advertise<example_nodes::State>("/demo/ready_state", rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   MAIN LOOP SETUP                          ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** MAIN LOOP **********/
  bool startup_complete = false;
  geometry_msgs::Twist curr_pos;

  // Control booleans
  bool cmd_processed = false;
  bool reached_pickup = false;
  bool picked_up = false;
  bool back_to_homePosition = false;
  bool reached_dropoff = false;
  bool dropoff_arm = false;
  bool arm_is_done = false;
  bool returning_home = false;

  // edge case booleans
  bool pickup_edge_case = false;
  bool dropoff_edge_case = false;

  while (ros::ok()) {
    if ( !startup_complete ) {
      /* Ensure that we are receiving key commands */
      /* TODO: Add a check that the vision system is online */
      if (keys_init == true) {
        startup_complete = true;
        example_nodes::State ready_tmp;
        ready_tmp.state = true;
        ready_publisher.publish(ready_tmp);
      }
    } else {

      /* Convert Key commands into base and arm commands */

      if (cmd_received) {
        example_nodes::State ready_tmp;
        ready_tmp.state = false;
        ready_publisher.publish(ready_tmp); // tell the vision not to send more commands
        cmd_received = false;


        if (target_pos.x == 0 && target_pos.y == 0) {
        //should never happen but is happening! Somewhere in key_read_rosie I think
          offset = 1;

        // } else if (sqrt(pow(target_pos.x,2) + pow(target_pos.y,2)) < 0.35) {
        //   ROS_INFO("At least I'm in here");
        //   cmd_vel.linear.x = (-target_pos.x);
        //   cmd_vel.linear.y = (-target_pos.y);
        //   omni_publisher.publish(cmd_vel);
        //   pickup_edge_case = true;
        //   continue;

        } else {
          offset = 1-(0.3/(sqrt(pow(target_pos.x,2) + pow(target_pos.y,2)))); 
          //multiplier
        }
        cmd_vel.linear.x = target_pos.x * offset;
        cmd_vel.linear.y = target_pos.y * offset;

        travelled_x = cmd_vel.linear.x;
        travelled_y = cmd_vel.linear.y;

        ROS_INFO("The base is moving to: (%lg, %lg)", cmd_vel.linear.x, cmd_vel.linear.y);

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


      // if (pickup_edge_case && base_state) {
      //   ROS_INFO("finished handling edge case");
      //   base_state = false;
      //   pickup_edge_case = false;
      //   target_pos.x = 2 * target_pos.x;
      //   target_pos.y = 2 * target_pos.y;
      //   // cmd_received = true;
      //   continue;
      // }


      if (cmd_processed && base_state) { // the base has finished executing
        cmd_processed = false;
        base_state = false;

        // ROS_INFO("%lg %lg %lg", arm_target.x, arm_target.y, arm_target.z);
        ROS_INFO("The spot we've reached is: (%lg, %lg)", odometry_pos.linear.x, odometry_pos.linear.y);
        arm_cmd.waypoints_vector = {arm_target};
        arm_publisher.publish(arm_cmd);

        reached_pickup = true;
      }


      // the arm has finished executing
      if (reached_pickup && arm_state) { 
        reached_pickup = false;
        arm_state = false;

        gripper_cmd.state = true;
        grip_publisher.publish(gripper_cmd);
        // grip_start_time = ros::Time::now();

        picked_up = true;
      }

      // telling it to be close, and it finished closing
      if (picked_up && gripper_state) { 
        picked_up = false;
        // arm_handoff.x = -0.2;
        // arm_handoff.y = -0.4;
        // arm_handoff.z = 0.5;
        arm_handoff.x = 101;
        arm_handoff.y = 101;
        arm_handoff.z = 101;
        arm_cmd.waypoints_vector = {arm_handoff};
        arm_publisher.publish(arm_cmd);
        back_to_homePosition = true;
      }


      if (back_to_homePosition && arm_state && gripper_state) {
        back_to_homePosition = false;
        arm_state = false;

        double dropoff_x = 1.00; //0.85;
        double dropoff_y = 0.40; //0.66;

        double new_x = dropoff_x - travelled_x;
        double new_y = dropoff_y - travelled_y;

        offset = 1-(0.4/(sqrt(pow(new_x,2) + pow(new_y,2)))); 

        cmd_vel.linear.x = new_x * offset;
        cmd_vel.linear.y = new_y * offset;

        travelled_x = travelled_x + cmd_vel.linear.x;
        travelled_y = travelled_y + cmd_vel.linear.y;
        omni_publisher.publish(cmd_vel);

        // ROS_INFO("I've picked up the package and should be moving! %lg %lg", new_x, new_y);

        arm_target.x = new_y - cmd_vel.linear.y;
        arm_target.y = - (new_x - cmd_vel.linear.x);
        arm_target.z = 0.1;
        
        reached_dropoff = true;
      }

      if (reached_dropoff && base_state) {
        reached_dropoff = false;
        base_state = false;

        arm_cmd.waypoints_vector = {arm_target};
        arm_publisher.publish(arm_cmd);

        dropoff_arm = true;
      }

      if (dropoff_arm && arm_state) {
        arm_state = false; 
        dropoff_arm = false;
        gripper_cmd.state = false;
        grip_publisher.publish(gripper_cmd);
        arm_is_done = true;
      }

      if (arm_is_done && !gripper_state) {
        arm_is_done = false;
        geometry_msgs::Point arm_reset;
        
        arm_reset.x = 100;
        arm_reset.y = 100;
        arm_reset.z = 100;
        
        arm_cmd.waypoints_vector = {arm_reset};
        arm_publisher.publish(arm_cmd);

        // double dropoff_x = 0.85;
        // double dropoff_y = 0.66;
        ROS_INFO("odom vs travelled: (%lg, %lg) vs (%lg, %lg) ", -odometry_pos.linear.x, -odometry_pos.linear.y, -travelled_x, - travelled_y);
        cmd_vel.linear.x = -odometry_pos.linear.x;
        cmd_vel.linear.y = -odometry_pos.linear.y;

        // cmd_vel.linear.x = -travelled_x;
        // cmd_vel.linear.y = -travelled_y;

        omni_publisher.publish(cmd_vel);

        returning_home = true;
      } 

      if (returning_home && base_state && arm_state) {
        returning_home = false;
        base_state = false;
        arm_state = false;
        example_nodes::State ready_tmp;
        ready_tmp.state = true;
        ready_publisher.publish(ready_tmp); // tell the vision to send more commands
        ROS_INFO("The ending position is: (%lg, %lg)", odometry_pos.linear.x, odometry_pos.linear.y);
      }




      // // gripper closed, arm has moved to handoff
      // if (back_to_homePosition && gripper_state && arm_state) { 
      //   back_to_homePosition = false;
      //   arm_state = false;

      //   gripper_cmd = false;
      //   grip_publisher.publish(gripper_cmd);
      //   handoff_done = true;
      // }

      // if (handoff_done && !gripper_state) {
      //   handoff_done = false;
      //   geometry_msgs::Point arm_reset;
        
      //   arm_reset.x = 100;
      //   arm_reset.y = 100;
      //   arm_reset.z = 100;
        
      //   arm_cmd.waypoints_vector = {arm_reset};
      //   arm_publisher.publish(arm_cmd);

      //   returning_home = true;
      // }

      // if (returning_home && arm_state) {
      //   returning_home = false;
      //   arm_state = false;
      // }


    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

