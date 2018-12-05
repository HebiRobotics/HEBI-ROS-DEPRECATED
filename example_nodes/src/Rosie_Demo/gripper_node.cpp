/**
 * Simple node to open and close the gripper via a service.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 24 Sep 2018
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include <chrono>
#include <thread>

#include <example_nodes/GripperSrv.h>

#include "hebi.h"
#include "lookup.hpp"
#include "group_command.hpp"

using namespace hebi;
using namespace example_nodes;

// Global Variables
bool gripper_closed_cmd = false;

bool gripperSrv(GripperSrv::Request& req, GripperSrv::Response& res) {

  ROS_INFO("Gripper Command Received: %d", gripper_closed_cmd);
  gripper_closed_cmd = req.closed;

  return true;
}

int main(int argc, char ** argv) {

  ros::init(argc, argv, "gripper_node");

  ros::NodeHandle node;

  // HEBI API SETUP

  // Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({"Rosie"}, {"Spool"}, 10000);

  if (!group) {
    ROS_ERROR("Group not found! Shutting Down...\n");
    return -1;
  }

  GroupCommand group_command(group -> size());

  // Constants for effort to open or close grippers
  constexpr double close_effort = -5;
  constexpr double open_effort = 1;
  auto& effort_cmd = group_command[0].actuator().effort();

  // ROS NODE SETUP

  ros::ServiceServer gripper_srv =
    node.advertiseService<GripperSrv::Request, GripperSrv::Response>(
      "/rosie/gripper", &gripperSrv);

  constexpr double command_rate = 100; // Hz
  ros::Rate loop_rate(command_rate);

  // MAIN LOOP

  while (ros::ok()) {
    effort_cmd.set(gripper_closed_cmd ? close_effort : open_effort);
    
    group->sendCommand(group_command);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}
