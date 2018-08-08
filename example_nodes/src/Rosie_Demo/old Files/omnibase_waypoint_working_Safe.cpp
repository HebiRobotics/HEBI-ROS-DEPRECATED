/**
* Provides an implentation of user controls for a 3-wheeled omnidirectional 
* base (omnibase). This example uses the HEBI trajectory generator to move the
* omnibase more smoothly.
* You can move forward/backward, left/right, or rotate in spot.
* 
* @author Hardik Singh < hardik@hebirobotics.com >
* @since 13 Jul 2018
**/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <example_nodes/State.h>
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
geometry_msgs::Twist waypoint_cmd; // has a linear x y and a z we dont use
Eigen::VectorXd target_pos(num_wheels);
Eigen::VectorXd curr_pos(num_wheels);
Eigen::VectorXd omniEfforts(num_wheels);
double rate_of_command = 60;
double speed = 0.3; // m/s
double wheelRotSpeed = M_PI/3; // (rad/s)

// Trajectory items
Eigen::VectorXd pos_traj(num_wheels);
Eigen::VectorXd vel_traj(num_wheels);
Eigen::VectorXd accel_traj(num_wheels);

bool trajectory_activate = false;

double rampTime = 0.33;
Eigen::VectorXd omniBaseTrajTime(2);

Eigen::MatrixXd positions; 
Eigen::MatrixXd velocities; 
Eigen::MatrixXd accelerations;

std::shared_ptr<hebi::trajectory::Trajectory> global_trajectory;
ros::Time trajStartTime;


bool base_state = false;
bool publish_state = false;

void waypoint_callback(geometry_msgs::Twist data) {
  /* Callback function that keeps up to date with key presses and commands */
  if ( (!trajectory_activate)
                      && ((data.linear.x != 0) || (data.linear.y != 0)))  {
    waypoint_cmd = data;
    trajectory_activate = true;
  } 
}


void generateWheelTrajectory() {
  // take waypoint command
  // take current position of the wheels, assumed as 0 0 0
  // calculate how many rotations need to be done per 
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (center of omni to origin of base)
  double offset = 1; //1-(0.3/(sqrt(pow(waypoint_cmd.linear.x,2) + pow(waypoint_cmd.linear.y,2)))); //multiplier
  double rotation = atan(waypoint_cmd.linear.x/waypoint_cmd.linear.y) * (baseRadius/wheelRadius);
  double x = (waypoint_cmd.linear.x * offset) / (2*wheelRadius*M_PI);
  double y = (waypoint_cmd.linear.y * offset) / (2*wheelRadius*M_PI);
  double ratio = sqrt(3)/2;
  // so we have a new x and y

  ROS_INFO("Rotation is: %lg", rotation);
  target_pos[0] = curr_pos[0] + (0.5*x + -ratio*y) * (2*M_PI);// + rotation;
  target_pos[1] = curr_pos[1] + (0.5*x + ratio*y) * (2*M_PI);// + rotation;
  target_pos[2] = curr_pos[2] + (-x) * (2*M_PI);// + rotation;

  omniBaseTrajTime << 0, 
        (4 * sqrt(pow(waypoint_cmd.linear.x,2) + pow(waypoint_cmd.linear.y,2)));

  positions << curr_pos[0], target_pos[0],
               curr_pos[1], target_pos[1],
               curr_pos[2], target_pos[2];
  velocities = Eigen::MatrixXd::Zero(num_wheels, 2);
  accelerations = Eigen::MatrixXd::Zero(num_wheels, 2);

  // initialise trajectory object
  global_trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
                  omniBaseTrajTime, positions, &velocities, &accelerations);
  trajStartTime = ros::Time::now();


}


void updateOmniEfforts() {
  double chassisMass = 12; //kg
  double wheelRadius = 0.0762; // m
  double baseRadius = 0.235; // m (center of omni to origin of base)
  double ratio = sqrt(3)/2;

  // Wheel1, front right
  omniEfforts[0] = accel_traj[0] * chassisMass * 0.5 * wheelRadius + 
                   accel_traj[1] * chassisMass * -ratio * wheelRadius + 
                   accel_traj[2] * baseRadius * wheelRadius;

  omniEfforts[1] = accel_traj[0] * chassisMass * 0.5 * wheelRadius + 
                   accel_traj[1] * chassisMass * ratio * wheelRadius + 
                   accel_traj[2] * baseRadius * wheelRadius;

  omniEfforts[2] = accel_traj[0] * chassisMass * -1 * wheelRadius + 
                   0 + 
                   accel_traj[2] * baseRadius * wheelRadius;
}



int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "omnibase_node_waypoint");

  ros::NodeHandle node;

  ros::Rate loop_rate(rate_of_command);

  ros::Subscriber key_subscriber = node.subscribe("/demo/cmd_vel", 20,
                           waypoint_callback);

  ros::Publisher state_publisher = node.advertise<example_nodes::State>("/demo/base_state", rate_of_command);
  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI API SETUP                            ////////
  ////////////////////////////////////////////////////////////////////////////

  /* Update this with the group name for your modules */
  std::string group_name = "omniGroup";

  //Get a group
  Lookup lookup;
  std::shared_ptr<Group> group = lookup.getGroupFromNames({"Rosie"}, 
                                  {"_Wheel1", "_Wheel2", "_Wheel3"});

  if (!group)
  {
    std::cout << "Group not found! Shutting Down...\n";
    return -1;
  }

  GroupCommand gains_cmd(group -> size());
  gains_cmd.readGains("/home/hebi/catkin_ws/src/HEBI-ROS/example_nodes/include/gains/omnibase_gains.xml");
  GroupCommand group_command(group -> size());
  GroupFeedback feedback(group -> size());
  group -> setFeedbackFrequencyHz(rate_of_command);

  ////////////////////////////////////////////////////////////////////////////
  ////////                   HEBI SETUP END                           ////////
  ////////////////////////////////////////////////////////////////////////////

  /******** Trajectory Setup **********/
  waypoint_cmd.linear.x = 0;
  waypoint_cmd.linear.y = 0;
  positions  = Eigen::MatrixXd::Zero(num_wheels,2);
  velocities = Eigen::MatrixXd::Zero(num_wheels, 2);
  accelerations = Eigen::MatrixXd::Zero(num_wheels, 2);
  generateWheelTrajectory();

  /******** MAIN LOOP **********/
  bool startup_complete = false;

  while (ros::ok()) {
    if ( !startup_complete ) {
      /* Ensure that we are receiving feedback and key commands */
      /* Store the initial pose of the omnibase in omniPos */
      if (group->getNextFeedback(feedback)) {
        // grab initial feedback
        curr_pos = feedback.getPosition();
        target_pos = curr_pos;
        group -> sendCommand(gains_cmd);
        // initialise trajectory object
        global_trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
                        omniBaseTrajTime, positions, &velocities, &accelerations);
        trajStartTime = ros::Time::now();
        startup_complete = true;
      }
    } else {


      if (trajectory_activate) {
        ROS_INFO("Generating the new trajectory!");
        generateWheelTrajectory();
        trajectory_activate = false;
        base_state = true;

      } else {
        double t = std::min((ros::Time::now() - trajStartTime).toSec(),
                                            global_trajectory -> getDuration());


        if (t == global_trajectory -> getDuration()) {
          curr_pos = target_pos;
          if (base_state) {
            state_publisher.publish(base_state);
            base_state = false;
          }
        }


        // Get state from trajectory
        global_trajectory -> getState(t, &pos_traj, &vel_traj, &accel_traj);

        /* Convert body velocities to wheel velocities */
        updateOmniEfforts();

        /* Set the new positions, velocities, and efforts */
        group_command.setPosition(pos_traj);
        group_command.setVelocity(vel_traj);
        group_command.setEffort(omniEfforts);

        /* Send the new commands */
        group -> sendCommand(group_command);

      }
    }
    // state_publisher.publish(base_state);



    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

