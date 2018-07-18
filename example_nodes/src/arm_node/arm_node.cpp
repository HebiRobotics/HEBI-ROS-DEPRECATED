
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <example_nodes/TargetWaypoints.h>
#include "sensor_msgs/JointState.h"

#include <iostream>
#include <chrono>
#include <thread>
#include "hebi.h"
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"
#include "robot_model.hpp"
#include "grav_comp.hpp"

#include <ros/console.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

using namespace hebi;
using ActuatorType = robot_model::RobotModel::ActuatorType;
using BracketType = robot_model::RobotModel::BracketType;
using LinkType = robot_model::RobotModel::LinkType;


// Global Variables
bool keys_init = false;
Eigen::VectorXd key_input(4);
example_nodes::TargetWaypoints targetWaypoints;


struct CreateDesc {
  std::vector<std::string> familyName;
  std::vector<std::string> moduleNames;
  std::vector<double> homePosition;
  int commandLifetime;
  double waypointTransitionTime;
  double feedbackFrequency;
};




void directions_callback(geometry_msgs::Point data) {
  key_input[0] = data.x/100;
  key_input[1] = data.y/100;
  key_input[2] = data.z/100;

  keys_init = true;
}



static bool createGroup(CreateDesc& createDesc, std::shared_ptr<Group>& group) {
  Lookup lookup;
  group = lookup.getGroupFromNames(createDesc.familyName, createDesc.moduleNames);

  if (group) {
    group -> setCommandLifetimeMs(createDesc.commandLifetime);
    group -> setFeedbackFrequencyHz(createDesc.feedbackFrequency);
    return true;
  }


  return false;
}




int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_node");

  ros::NodeHandle node;

  ros::Rate loop_rate(100);

  ros::Subscriber key_subscriber = node.subscribe("keys/cmd_vel", 50,
                           directions_callback);

  /////////////////// INITIALISAITON ///////////////////

  CreateDesc desc = {
    {"4-DoF Arm"},
    {"Base", "Shoulder", "Elbow", "Wrist"},
    {0, -M_PI*2/5, -M_PI*4/5, -M_PI_2},
    100,
    0.5,
    500.0
  };

  std::shared_ptr<Group> group;

  if (!createGroup(desc, group)) {
    std::cerr << "Group not found on network! Shutting down..." << std::endl;
    return -1;
  }

  auto num_joints = group -> size();
  Eigen::VectorXd homePosition(num_joints);

  for (int i = 0; i < num_joints; i++) {
    homePosition[i] = desc.homePosition[i];
  }


  GroupFeedback group_fbk(group -> size());
  // feedback frequency was already set

  if (!group->getNextFeedback(group_fbk)) {
    std::cout << "Error Getting Feedback..." << std::endl;
    return -1;
  }


  // Create robot model and get masses/gravity for grav-comp code
  robot_model::RobotModel model;
  model.addActuator(ActuatorType::X5_9);
  model.addBracket(BracketType::X5HeavyLeftOutside ); // fourgrowers has right not left
  model.addActuator(ActuatorType::X5_9);
  model.addLink(LinkType::X5, 0.325, M_PI);
  model.addActuator(ActuatorType::X5_9);
  model.addLink(LinkType::X5, 0.325, M_PI);
  model.addActuator(ActuatorType::X5_4);
  model.addLink(LinkType::X5, 0.175, 0);

  Eigen::VectorXd masses(model.getFrameCount(HebiFrameTypeCenterOfMass));
  model.getMasses(masses);
  Eigen::Vector3d gravity(0, 0, -1);


  Eigen::Vector3d target_xyz;
  target_xyz << 0.4, 0.0, 0.0;
  Eigen::VectorXd initial_joint_angles(group -> size());
  initial_joint_angles = group_fbk.getPosition();
  Eigen::VectorXd ik_result_joint_angles(group -> size());

  Eigen::VectorXd min_positions(group -> size());
  min_positions << -M_PI_2, -(M_PI*7)/8, -(M_PI*7)/8, -M_PI*2/3;
  Eigen::VectorXd max_positions(group -> size());
  max_positions << M_PI_2, 0, (M_PI*7)/8, M_PI*2/3;

  Eigen::Matrix4d transform;
  model.getEndEffector(group_fbk.getPosition(), transform);


  int num_waypoints = 2;
  GroupCommand group_command(group -> size());

  // Trajectory Items
  double rampTime = 0.14;
  Eigen::VectorXd trajTime(num_waypoints);
  trajTime << 0, rampTime;

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd efforts(num_joints, num_waypoints);

  std::shared_ptr<hebi::trajectory::Trajectory> trajectory;
  ros::Time trajStartTime;

  Eigen::VectorXd pos_traj = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd vel_traj = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd accel_traj = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd effort_grav(num_joints);


  /////////////////// Main Loop ///////////////////


  bool startup_complete = false;
  bool fbk_startup = false;
  double t;

  while (ros::ok()) {
    if (!startup_complete) {
        
      if (!fbk_startup) {
        if(group -> getNextFeedback(group_fbk)) {
          positions << group_fbk[0].actuator().position().get(), homePosition[0],
                 group_fbk[1].actuator().position().get(), homePosition[1],
                 group_fbk[2].actuator().position().get(), homePosition[2],
                 group_fbk[3].actuator().position().get(), homePosition[3];
          trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
                                trajTime, positions, nullptr, nullptr);
          trajStartTime = ros::Time::now();
          fbk_startup = true; 
        }
      } else {
        t = std::min((ros::Time::now() - trajStartTime).toSec(),
                                            trajectory -> getDuration());
        trajectory -> getState(t, &pos_traj, &vel_traj, &accel_traj);

        group_command.setPosition(pos_traj);
        group_command.setVelocity(vel_traj);
        effort_grav = util::GravityCompensation::getEfforts(model, masses, group_fbk);
        group_command.setEffort(effort_grav);
        group -> sendCommand(group_command);

        positions << pos_traj[0], homePosition[0],
                     pos_traj[1], homePosition[1],
                     pos_traj[2], homePosition[2],
                     pos_traj[3], homePosition[3];

        velocities << vel_traj[0], 0,
                      vel_traj[1], 0,
                      vel_traj[2], 0,
                      vel_traj[3], 0;

        efforts << accel_traj[0], 0,
                   accel_traj[1], 0,
                   accel_traj[2], 0,
                   accel_traj[3], 0;

        trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(trajTime, positions, &velocities, &efforts);
        trajStartTime = ros::Time::now();

        if ((pos_traj[2] - homePosition[2]) < 0.1){
          startup_complete = true;
        }
      }

    } else {


      double t = std::min((ros::Time::now() - trajStartTime).toSec(),
                                              trajectory -> getDuration());

      trajectory -> getState(t, &pos_traj, &vel_traj, &accel_traj);

      group_command.setPosition(pos_traj);
      group_command.setVelocity(vel_traj);
      effort_grav = util::GravityCompensation::getEfforts(model, masses, group_fbk);
      group_command.setEffort(effort_grav);
      group -> sendCommand(group_command);

      // Check to see what the next trajectory is 
      target_xyz[0] += key_input[0];
      target_xyz[1] += key_input[1];
      target_xyz[2] += key_input[2];

      model.solveIK(
        group_fbk.getPosition(),
        ik_result_joint_angles,
        robot_model::EndEffectorPositionObjective(target_xyz)
        // robot_model::JointLimitConstraint(min_positions, max_positions)
      );


      positions << pos_traj[0], ik_result_joint_angles[0],
                   pos_traj[1], ik_result_joint_angles[1],
                   pos_traj[2], ik_result_joint_angles[2],
                   pos_traj[3], ik_result_joint_angles[3];

      velocities << vel_traj[0], 0,
                    vel_traj[1], 0,
                    vel_traj[2], 0,
                    vel_traj[3], 0;

      efforts << accel_traj[0], 0,
                 accel_traj[1], 0,
                 accel_traj[2], 0,
                 accel_traj[3], 0;

      std::cout << "Target position: " << std::endl << target_xyz.transpose() << std::endl;
      std::cout << "IK joint angles: " << std::endl << ik_result_joint_angles.transpose() << std::endl;
      model.getEndEffector(ik_result_joint_angles, transform);
      std::cout << "FK of IK joint angles: " << std::endl << transform.topRightCorner<3,1>().transpose() << std::endl << std::endl;


      // ROS_INFO("%lg %lg %lg", target_xyz[0], target_xyz[1], target_xyz[2]);

      // ROS_INFO("%lg %lg %lg %lg", ik_result_joint_angles[0], ik_result_joint_angles[1], ik_result_joint_angles[2], ik_result_joint_angles[3]);


      trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
                              trajTime, positions, &velocities, &efforts);
      trajStartTime = ros::Time::now();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

