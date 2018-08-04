#include "hebiros_actions.h"

#include "hebiros.h"

#include "hebiros_group_registry.h"

using namespace hebiros;

int count_character_occurences(std::string s, char ch) {
  int count = 0;

  for (int i = 0; i < s.size(); i++) {
    if (s[i] == ch) {
      count++;
    }
  }

  return count;
}

std::map<std::string,
  std::shared_ptr<actionlib::SimpleActionServer<hebiros::TrajectoryAction>>>
  HebirosActions::trajectory_actions;

std::map<std::string,
  std::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>>
  HebirosActions::follow_joint_trajectory_actions;

void HebirosActions::registerGroupActions(std::string group_name) {

  trajectory_actions[group_name] = std::make_shared<
    actionlib::SimpleActionServer<TrajectoryAction>>(
    *HebirosNode::n_ptr, "/hebiros/"+group_name+"/trajectory",
    boost::bind(&HebirosActions::trajectory, this, _1, group_name), false);

  trajectory_actions[group_name]->start();


  follow_joint_trajectory_actions[group_name] = std::make_shared<
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
    *HebirosNode::n_ptr, "/hebiros/"+group_name+"/follow_joint_trajectory",
    boost::bind(&HebirosActions::follow_joint_trajectory, this, _1, group_name), false);

  follow_joint_trajectory_actions[group_name]->start();
}

void HebirosActions::trajectory(const TrajectoryGoalConstPtr& goal, std::string group_name) {

  auto& registry = HebirosGroupRegistry::Instance();
  HebirosGroup* group = registry.getGroup(group_name);

  if (!group) {
    ROS_WARN("Group not found.");
    return;
  }

  std::shared_ptr<actionlib::SimpleActionServer<TrajectoryAction>> action_server =
    trajectory_actions[group_name];

  int num_waypoints = goal->waypoints.size();
  if (num_waypoints < 1) {
    ROS_WARN("No waypoints sent.");
    return;
  }
  int num_joints = goal->waypoints[0].names.size();

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  Eigen::VectorXd time(num_waypoints);

  for (int i = 0; i < num_waypoints; i++) {
    time(i) = goal->times[i];
  }

  for (int i = 0; i < num_joints; i++) {
    std::string joint_name = goal->waypoints[0].names[i];
    int joint_index = group->joints[joint_name];

    for (int j = 0; j < num_waypoints; j++) {
      double position = goal->waypoints[j].positions[i];
      double velocity = goal->waypoints[j].velocities[i];
      double acceleration = goal->waypoints[j].accelerations[i];

      positions(joint_index, j) = position;
      velocities(joint_index, j) = velocity;
      accelerations(joint_index, j) = acceleration;
    }
  }

  auto trajectory = trajectory::Trajectory::createUnconstrainedQp(
    time, positions, &velocities, &accelerations);
  Eigen::VectorXd position_command(num_joints);
  Eigen::VectorXd velocity_command(num_joints);

  double trajectory_duration = trajectory->getDuration();
  double previous_time;
  double current_time;
  double loop_duration;
  TrajectoryFeedback feedback;

  ros::Rate loop_rate(HebirosParameters::getInt("/hebiros/action_frequency"));

  ROS_INFO("Group [%s]: Executing trajectory", group_name.c_str());
  previous_time = ros::Time::now().toSec();
  for (double t = 0; t < trajectory_duration; t += loop_duration)
  {
    if (action_server->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("Group [%s]: Preempted trajectory", group_name.c_str());
      action_server->setPreempted();
      return;
    }

    feedback.percent_complete = (t / trajectory_duration) * 100;
    action_server->publishFeedback(feedback);

    trajectory->getState(t, &position_command, &velocity_command, nullptr);
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(num_joints);
    joint_state_msg.position.resize(num_joints);
    joint_state_msg.velocity.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
      std::string joint_name = goal->waypoints[0].names[i];
      int joint_index = group->joints[joint_name];
      joint_state_msg.name[joint_index] = joint_name;
      joint_state_msg.position[joint_index] = position_command(i);
      joint_state_msg.velocity[joint_index] = velocity_command(i);
    }
    HebirosNode::publishers_physical.commandJointState(joint_state_msg, group_name);

    ros::spinOnce();
    loop_rate.sleep();
    current_time = ros::Time::now().toSec();
    loop_duration = current_time - previous_time;
    previous_time = current_time;
  }

  TrajectoryResult result;
  result.final_state = group->joint_state_msg;
  action_server->setSucceeded(result);
  ROS_INFO("Group [%s]: Finished executing trajectory", group_name.c_str());
}

void HebirosActions::follow_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, std::string group_name) {

  auto& registry = HebirosGroupRegistry::Instance();
  HebirosGroup* group = registry.getGroup(group_name);

  if (!group) {
    ROS_WARN("Group not found.");
    return;
  }

  std::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> action_server =
    follow_joint_trajectory_actions[group_name];

  int num_waypoints = goal->trajectory.points.size();
  if (num_waypoints < 1) {
    ROS_WARN("No waypoints sent.");
    return;
  }
  int num_joints = goal->trajectory.joint_names.size();
  ROS_INFO("num_joints [%i]", num_joints);

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  Eigen::VectorXd time(num_waypoints);

  for (int i = 0; i < num_waypoints; i++) {
    time(i) = goal->trajectory.points[i].time_from_start.toSec();
    ROS_INFO("time [%f]", time(i));
  }

  for (int i = 0; i < num_joints; i++) {
    std::string joint_name = goal->trajectory.joint_names[i];
    std::string hebi_joint_name;
    if (count_character_occurences(joint_name, '/') > 1) {
      int split_index = joint_name.rfind('/');
      hebi_joint_name = joint_name.substr(0, split_index);
    } else {
      hebi_joint_name = joint_name;
    }
    ROS_INFO("hebi_joint_name [%s]", hebi_joint_name.c_str());

    int joint_index = group->joints[hebi_joint_name];
    ROS_INFO("joint_index [%i]", joint_index);

    for (int j = 0; j < num_waypoints; j++) {
      double position = goal->trajectory.points[j].positions[i];
      double velocity = goal->trajectory.points[j].velocities[i];
      double acceleration = goal->trajectory.points[j].accelerations[i];

      positions(joint_index, j) = position;
      velocities(joint_index, j) = velocity;
      accelerations(joint_index, j) = acceleration;
    }
  }

  auto trajectory = trajectory::Trajectory::createUnconstrainedQp(
    time, positions, &velocities, &accelerations);
  Eigen::VectorXd position_command(num_joints);
  Eigen::VectorXd velocity_command(num_joints);

  double trajectory_duration = trajectory->getDuration();
  double previous_time;
  double current_time;
  double loop_duration;
  control_msgs::FollowJointTrajectoryFeedback feedback;

  ros::Rate loop_rate(HebirosParameters::getInt("/hebiros/action_frequency"));

  ROS_INFO("Group [%s]: Executing follow_joint_trajectory", group_name.c_str());
  previous_time = ros::Time::now().toSec();
  for (double t = 0; t < trajectory_duration; t += loop_duration)
  {
    if (action_server->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("Group [%s]: Preempted follow_joint_trajectory", group_name.c_str());
      action_server->setPreempted();
      return;
    }

    feedback.header.stamp = ros::Time::now();
    feedback.joint_names = goal->trajectory.joint_names;
    action_server->publishFeedback(feedback);

    trajectory->getState(t, &position_command, &velocity_command, nullptr);
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(num_joints);
    joint_state_msg.position.resize(num_joints);
    joint_state_msg.velocity.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
      std::string joint_name = goal->trajectory.joint_names[i];
      std::string hebi_joint_name;
      if (count_character_occurences(joint_name, '/') > 1) {
        int split_index = joint_name.rfind('/');
        hebi_joint_name = joint_name.substr(0, split_index);
      } else {
        hebi_joint_name = joint_name;
      }
      int joint_index = group->joints[hebi_joint_name];
      joint_state_msg.name[joint_index] = hebi_joint_name;
      joint_state_msg.position[joint_index] = position_command(i);
      joint_state_msg.velocity[joint_index] = velocity_command(i);
    }
    HebirosNode::publishers_physical.commandJointState(joint_state_msg, group_name);

    ros::spinOnce();
    loop_rate.sleep();
    current_time = ros::Time::now().toSec();
    loop_duration = current_time - previous_time;
    previous_time = current_time;
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  action_server->setSucceeded(result);
  ROS_INFO("Group [%s]: Finished executing follow_joint_trajectory", group_name.c_str());
}
