
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

#include <mutex>

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

// TODO: NAMESPACE THIS UP!!!

// A structure describing the current command state, including any active
// trajectory
struct ArmState {
  ArmState() {
    last_target_xyz.x = 0.4;
    last_target_xyz.y = 0;
    last_target_xyz.z = 0;

    // TODO: CHANGE THIS!
    // TEMPORARY INIT TRAJECTORY CODE
    int num_joints = 4;
    int num_waypoints = 2;
    Eigen::MatrixXd positions(num_joints, num_waypoints);
    Eigen::MatrixXd velocities(num_joints, num_waypoints);
    Eigen::MatrixXd accelerations(num_joints, num_waypoints);
    positions << 0, 0, 0, 0, 0, 0, 0, 0;
    velocities << 0, 0, 0, 0, 0, 0, 0, 0;
    accelerations << 0, 0, 0, 0, 0, 0, 0, 0;

    // Trajectory Items
    double rampTime = 2;
    Eigen::VectorXd trajTime(num_waypoints);
    trajTime << 0, rampTime;

    trajectory = hebi::trajectory::Trajectory::createUnconstrainedQp(
      trajTime, positions, &velocities, &accelerations);
  }
  std::shared_ptr<hebi::trajectory::Trajectory> trajectory {};
//  hebi::trajectory::Trajectory trajectory;
  ros::Time trajectory_start_time {};
  // Temporary code for last point, so we can test with key input -- eventually
  // replace this (although this works for a "jog" example)
  geometry_msgs::Point last_target_xyz {};
};

// The current state of the arm commands, and a mutex to protect this from
// access in different threads
ArmState arm_state;
std::mutex arm_state_mutex;

struct CreateDesc {
  std::vector<std::string> familyName;
  std::vector<std::string> moduleNames;
  std::vector<double> homePosition;
  int commandLifetime;
  double waypointTransitionTime;
  double feedbackFrequency;
};


const robot_model::RobotModel* model_global;
Group* group_global;
const GroupFeedback* group_fbk_global;
const CreateDesc* desc_global;

// Updates the Arm State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this call assumes feedback is populated.
void ReplanTrajectory(
  const hebi::GroupFeedback& feedback,
  const Eigen::MatrixXd& new_positions,
  const Eigen::MatrixXd& new_velocities,
  const Eigen::MatrixXd& new_accelerations) {

  int num_joints = 4; // TODO: get this from somewhere!
  int num_waypoints = 2; // TODO: get this from somewhere!

  // Protect access to the arm state object
  std::lock_guard<std::mutex> guard(arm_state_mutex);

  // If there is a current trajectory, use the commands as a starting point;
  // if not, replan from current feedback.
  Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);
  if (arm_state.trajectory) {
    // Clip time to end of trajectory
    double t = std::min((ros::Time::now() - arm_state.trajectory_start_time).toSec(),
                        arm_state.trajectory->getDuration());
    arm_state.trajectory->getState(t, &curr_pos, &curr_vel, &curr_accel);
  } else {
    curr_pos = feedback.getPosition();
    curr_vel = feedback.getVelocity();
    // (accelerations remain zero)
  }

  // TODO: define times...
  double rampTime = 1.5;
  Eigen::VectorXd trajTime(num_waypoints);
  trajTime << 0, rampTime; // TODO: fixme for multiple waypoints

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // TODO: generalize this for multiple waypoints!
  positions << curr_pos[0], new_positions(0, 0),
               curr_pos[1], new_positions(1, 0),
               curr_pos[2], new_positions(2, 0),
               curr_pos[3], new_positions(3, 0);

  velocities << curr_vel[0], new_velocities(0, 0),
                curr_vel[1], new_velocities(1, 0),
                curr_vel[2], new_velocities(2, 0),
                curr_vel[3], new_velocities(3, 0);

  accelerations << curr_accel[0], new_accelerations(0, 0),
                   curr_accel[1], new_accelerations(1, 0),
                   curr_accel[2], new_accelerations(2, 0),
                   curr_accel[3], new_accelerations(3, 0);

  // Create new trajectory
  arm_state.trajectory = std::move(hebi::trajectory::Trajectory::createUnconstrainedQp(
                                   trajTime, positions, &velocities, &accelerations));
  arm_state.trajectory_start_time = ros::Time::now();
}


// TODO: add callback for lots of points, too!
// TODO: I wish group could be const -- we should fix this in the API!
void directions_callback(
  geometry_msgs::Point data) {
// TODO: change all these to references, but then the stupid bind doesn't work...
//  const robot_model::RobotModel* model, Group* group, const GroupFeedback* group_fbk, const CreateDesc* desc) {

  // Only update if target changes!
  if (data.x == 0 && data.y == 0 && data.z == 0)
    return;

  auto model = model_global;
  auto group = group_global;
  auto group_fbk = group_fbk_global;
  auto desc = desc_global;

  // Replan:
  
//  std::lock_guard<std::mutex> guard(arm_state_mutex);

  // Update the target point
  // TODO: store this elsewhere...
  arm_state.last_target_xyz.x += data.x / 100.0;
  arm_state.last_target_xyz.y += data.y / 100.0;
  arm_state.last_target_xyz.z += data.z / 100.0;
   
  // TODO: (note -- don't double block arm_state)   
  Eigen::VectorXd ik_result_joint_angles(group->size());

  Eigen::Vector3d target_xyz;
  target_xyz <<
    arm_state.last_target_xyz.x, 
    arm_state.last_target_xyz.y,
    arm_state.last_target_xyz.z;

  model->solveIK(
    group_fbk->getPosition(),
    ik_result_joint_angles,
    robot_model::EndEffectorPositionObjective(target_xyz)
    // robot_model::JointLimitConstraint(min_positions, max_positions)
  );


  // TODO: check results...
  // NOTE: this is going from last trajectory location...

  // TODO: handle no last position / initial position

  int num_joints = group->size();
  // Number of _additional_ waypoints
  int num_waypoints = 1;
  Eigen::MatrixXd positions(num_joints, num_waypoints);
  positions = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
  for (int i = 0; i < num_joints; ++i) {
    positions(i, 0) = ik_result_joint_angles[i];
  }
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  velocities = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  accelerations = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
  ReplanTrajectory(*group_fbk, positions, velocities, accelerations);
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

  /////////////////// INITIALISATION ///////////////////

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

  GroupFeedback group_fbk(group->size());
  // feedback frequency was already set

  if (!group->getNextFeedback(group_fbk)) {
    std::cout << "Error Getting Feedback..." << std::endl;
    return -1;
  }
 
  /////////////////// Set up initial trajectory ////////////////////

  // Number of _additional_ waypoints
  int num_waypoints = 1;
  Eigen::MatrixXd positions(num_joints, num_waypoints);
  for (int i = 0; i < num_joints; ++i)
    positions(i, 0) = homePosition[i];
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  velocities = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  accelerations = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
  ReplanTrajectory(group_fbk, positions, velocities, accelerations);

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

  // TODO: set up initial trajectory

  model_global = &model;
  group_global = group.get();
  group_fbk_global = &group_fbk;
  desc_global = &desc;

  ros::Subscriber key_subscriber =
    node.subscribe("keys/cmd_vel", 50, directions_callback);
//    boost::bind(directions_callback, _1, &model, &*group, &group_fbk, &desc));
  // TODO: make that a const ref arg?
    //[&model, &group, &group_fbk, &desc](geometry_msgs::Point data) { directions_callback(model, *group, group_fbk, desc, data); });

  Eigen::VectorXd masses(model.getFrameCount(HebiFrameTypeCenterOfMass));
  model.getMasses(masses);

/*  Eigen::VectorXd initial_joint_angles(group -> size());
  initial_joint_angles = group_fbk.getPosition();

//  Eigen::VectorXd min_positions(group -> size());
//  min_positions << -M_PI_2, -(M_PI*7)/8, -(M_PI*7)/8, -M_PI*2/3;
//  Eigen::VectorXd max_positions(group -> size());
//  max_positions << M_PI_2, 0, (M_PI*7)/8, M_PI*2/3;

  Eigen::Matrix4d transform;
  model.getEndEffector(group_fbk.getPosition(), transform);
*/
  GroupCommand group_command(group -> size());

 /* std::shared_ptr<hebi::trajectory::Trajectory> trajectory;
  ros::Time trajStartTime;

*/
  /////////////////// Main Loop ///////////////////


  double t;

  while (ros::ok()) {
    if (!group->getNextFeedback(group_fbk)) {
      // Didn't get feedback?  Try again!
      continue;
    }
    {
      std::lock_guard<std::mutex> guard(arm_state_mutex);
      
      // Note -- by this time, the trajectory should always be defined!
      // Update command:
      t = std::min((ros::Time::now() - arm_state.trajectory_start_time).toSec(),
                   arm_state.trajectory->getDuration());
      Eigen::VectorXd pos = Eigen::VectorXd::Zero(num_joints);
      Eigen::VectorXd vel = Eigen::VectorXd::Zero(num_joints);
      Eigen::VectorXd accel = Eigen::VectorXd::Zero(num_joints);
      arm_state.trajectory->getState(t, &pos, &vel, &accel);
      group_command.setPosition(pos);
      group_command.setVelocity(vel);

      // Add grav-comp efforts
      //TODO: store "effort_grav" outside of main control loop
      Eigen::VectorXd effort_grav(num_joints);
      effort_grav = util::GravityCompensation::getEfforts(model, masses, group_fbk);
      group_command.setEffort(effort_grav);
      group->sendCommand(group_command);
    }

    ros::spinOnce();
  }

  return 0;
}

