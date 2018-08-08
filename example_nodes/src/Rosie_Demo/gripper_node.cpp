/**
* Provides a base level implentation of user controls for a 3-wheeled 
* omnidirectional base (omnibase).
* You can move forward/backward, left/right, or rotate in spot.
* 
* @author Hardik Singh < hardik @ hebirobotics.com >
* @since 10 Jul 2018
**/

#include <ros/ros.h>
#include <example_nodes/State.h>

#include <iostream>
#include <chrono>
#include <thread>
#include "hebi.h"
#include "lookup.hpp"
#include "group_command.hpp"

#include <ros/console.h>

using namespace hebi;

// Global Variables
double rate_of_command = 60;
bool gripper_closed_cmd = false;
bool close_published = false;

ros::Time gripper_close_time;

ros::Time gripper_open_time;
bool open_published = false;



void cmd_callback(example_nodes::State data) {
  /* Callback function that keeps up to date with key presses and commands */ 
  gripper_closed_cmd = data.state;
  if (data.state) {
    gripper_close_time = ros::Time::now();
    close_published = true;
  } else {
    gripper_open_time = ros::Time::now();
    open_published = true;
  }
  // false if open, true if closed
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "gripper_node");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ros::Subscriber cmd_subscriber = node.subscribe("/demo/gripper_cmd", rate_of_command,
                           cmd_callback);

  ros::Publisher state_publisher = node.advertise<example_nodes::State>(
    "/demo/gripper_state", 100);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Update this with the group name for your modules */
  std::string group_name = "omniGroup";

  //Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({"Rosie"}, {"Spool"});

  if (!group)
  {
    std::cout << "Group not found! Shutting Down...\n";
    return -1;
  }

  GroupCommand group_command(group -> size());
  // GroupFeedback feedback(group -> size());
  // group -> setFeedbackFrequencyHz(rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                           ////////
  ////////////////////////////////////////////////////////////////////////////


  /******** MAIN LOOP **********/
  Eigen::VectorXd close_effort(group -> size());
  close_effort << -5;
  Eigen::VectorXd open_effort(group -> size());
  open_effort << 1;


  while (ros::ok()) {




    if (gripper_closed_cmd) {
      group_command.setEffort(close_effort);
      
      if (close_published && (ros::Time::now() - gripper_close_time).toSec() >= 0.75) {
        state_publisher.publish(close_published);
        close_published = false;
      }
    }
    else {
      group_command.setEffort(open_effort);

      if (open_published && (ros::Time::now() - gripper_open_time).toSec() >= 1) {
        state_publisher.publish(false);
        open_published = false;
      }
    }

    // ROS_INFO("%d", gripper_closed_cmd);


    // group_command.setEffort(close_effort);
    std::cout << group_command.getEffort() << std::endl;
    group -> sendCommand(group_command);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

