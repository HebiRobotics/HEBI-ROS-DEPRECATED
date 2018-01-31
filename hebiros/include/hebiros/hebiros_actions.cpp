#include "hebiros.hpp"


//Action callback which controls following a trajectory
void Hebiros_Node::action_trajectory(const TrajectoryGoalConstPtr& goal, std::string group_name) {

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
    int joint_index = group_joints[group_name][joint_name];

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

  ros::Rate loop_rate(action_frequency);

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
    sensor_msgs::JointState command_msg;
    command_msg.name.resize(num_joints);
    command_msg.position.resize(num_joints);
    command_msg.velocity.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
      std::string joint_name = goal->waypoints[0].names[i];
      int joint_index = group_joints[group_name][joint_name];
      command_msg.name[joint_index] = joint_name;
      command_msg.position[joint_index] = position_command(i);
      command_msg.velocity[joint_index] = velocity_command(i);
    }
    publishers["/hebiros/"+group_name+"/command/joint_state"].publish(command_msg);

    ros::spinOnce();
    loop_rate.sleep();
    current_time = ros::Time::now().toSec();
    loop_duration = current_time - previous_time;
    previous_time = current_time;
  }

  TrajectoryResult result;
  result.final_state = group_joint_states[group_name];
  action_server->setSucceeded(result);
  ROS_INFO("Group [%s]: Finished executing trajectory", group_name.c_str());
}


