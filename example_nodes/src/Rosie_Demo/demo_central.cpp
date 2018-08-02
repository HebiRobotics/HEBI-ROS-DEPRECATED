
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

#include <ros/console.h>

using namespace hebi;

// Global Variables
int num_wheels = 3;
geometry_msgs::Point target_pos;
geometry_msgs::Twist odom_pos;
geometry_msgs::Twist cmd_vel;
bool keys_init = true;
double rate_of_command = 60;
double speed = 0.3; // m/s
double wheelRotSpeed = M_PI/3; // (rad/s)



void target_callback(geometry_msgs::Point data) {
  std::cout << data << std::endl;
  target_pos = data;
  keys_init = true;
}

void odom_callback(geometry_msgs::Twist data) {
  odom_pos = data;
}

void go_to_point(geometry_msgs::Twist odom_pos, geometry_msgs::Point target_pos) {
  ROS_INFO("%lg %lg %lg %lg", target_pos.x, odom_pos.linear.x, target_pos.y, odom_pos.linear.y);
  
  double dx;
  double dy;




  // dx = target_pos.x - odom_pos.linear.x;
  // dy = target_pos.y - odom_pos.linear.y;

  // if (std::abs(dx) > 0.01) {
  //   if (dx >= 0) {
  //     cmd_vel.linear.x = 1;
  //   } else {
  //     cmd_vel.linear.x = -1;
  //   }
  // } else {
  //   cmd_vel.linear.x = 0;
  // }


  // if (std::abs(dy) > 0.01) {
  //   if (dy >= 0) {
  //     cmd_vel.linear.y = 1;
  //   } else {
  //     cmd_vel.linear.y = -1;
  //   }
  // } else {
  //   cmd_vel.linear.y = 0;
  // }

  // ROS_INFO("We here yeee");

}


int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "omnibase_node_trajectory");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  // INPUT
  ros::Subscriber key_subscriber = node.subscribe("/demo/target", rate_of_command, target_callback);
  ros::Subscriber odom_subscriber = node.subscribe("omnibase_node/odometry", rate_of_command, odom_callback);

  // OUTPUT
  ros::Publisher cmd_publisher = node.advertise<geometry_msgs::Twist>("/demo/cmd_vel", rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  // /* Update this with the group name for your modules */
  // std::string group_name = "omniGroup";

  // //Get a group
  // Lookup lookup;
  // std::shared_ptr<Group> group = lookup.getGroupFromNames({"Rosie"}, 
  //                                 {"_Wheel1", "_Wheel2", "_Wheel3"});

  // if (!group)
  // {
  //   std::cout << "Group not found! Shutting Down...\n";
  //   return -1;
  // }

  // GroupCommand gains_cmd(group -> size());
  // gains_cmd.readGains("/home/hebi/catkin_ws/src/HEBI-ROS/example_nodes/include/gains/omnibase_gains.xml");
  // GroupCommand group_command(group -> size());
  // GroupFeedback feedback(group -> size());
  // group -> setFeedbackFrequencyHz(rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                           ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** Trajectory Setup **********/

  // double rampTime = 0.33;
  // Eigen::VectorXd omniBaseTrajTime(2);
  // omniBaseTrajTime << 0, rampTime;

  // Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_wheels, 2);
  // Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_wheels, 2);
  // Eigen::MatrixXd jerks = Eigen::MatrixXd::Zero(num_wheels, 2);

  // std::shared_ptr<hebi::trajectory::Trajectory> trajectory;
  // ros::Time trajStartTime;

  /******** MAIN LOOP **********/
  bool startup_complete = false;
  geometry_msgs::Twist curr_pos;
  double offset = 1;
  // geometry_msgs::Twist next_pos;

  while (ros::ok()) {
    if ( !startup_complete ) {
      /* Ensure that we are receiving key commands */
      if (keys_init == true) {
        startup_complete = true;
      }
    } else {
      // ROS_INFO("At least I'm looping");
      // std::cout << "printing to be sure" << std::endl;
      // go_to_point(odom_pos, target_pos);
      if (target_pos.x != 0 || target_pos.y != 0) { 
        offset = 1-(0.3/(sqrt(pow(target_pos.x,2) + pow(target_pos.y,2)))); //multiplier
      } else {
        offset = 1;
      }
      cmd_vel.linear.x = target_pos.x * offset;
      cmd_vel.linear.y = target_pos.y * offset;


      ROS_INFO("%lg %lg", cmd_vel.linear.x, cmd_vel.linear.y);
      cmd_publisher.publish(cmd_vel);

      // take the target pos
      // calculate directional velocity from current position
      // send that velocity until we have reached final position.


    ros::spinOnce();
    loop_rate.sleep();
    }
  }
  return 0;
}

