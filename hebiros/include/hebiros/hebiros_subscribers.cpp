#include "hebiros.hpp"


//Subscriber callback which receives commands and sends them to a group
void Hebiros_Node::sub_command(const boost::shared_ptr<CommandMsg const> data,
  std::string group_name) {

  if (use_gazebo) {

    if (names_in_order(*data)) {
      publishers["/hebiros_gazebo_plugin/command"].publish(*data);
    }
    else {
      ROS_WARN("Simulated commands are assigned with different orders.  Command will not be sent.");
    }

    return;
  }

  std::shared_ptr<Group> group = groups[group_name];
  GroupCommand group_command(group->size());

  sensor_msgs::JointState joint_data;
  joint_data.name = data->name;
  joint_data.position = data->position;
  joint_data.velocity = data->velocity;
  joint_data.effort = data->effort;
  SettingsMsg settings_data;
  settings_data = data->settings;

  add_joint_command(&group_command, joint_data, group_name);
  add_settings_command(&group_command, settings_data, group_name);

  group->sendCommand(group_command);
}

//Subscriber callback which receives a joint state and sends that as a command to a group
void Hebiros_Node::sub_joint_command(const boost::shared_ptr<sensor_msgs::JointState const> data,
  std::string group_name) {

  if (use_gazebo) {

    CommandMsg command_data;
    command_data.name = data->name;
    command_data.position = data->position;
    command_data.velocity = data->velocity;
    command_data.effort = data->effort;

    publishers["/hebiros_gazebo_plugin/command"].publish(command_data);

    return;
  }

  std::shared_ptr<Group> group = groups[group_name];
  GroupCommand group_command(group->size());

  sensor_msgs::JointState joint_data;
  joint_data.name = data->name;
  joint_data.position = data->position;
  joint_data.velocity = data->velocity;
  joint_data.effort = data->effort;

  add_joint_command(&group_command, joint_data, group_name);

  group->sendCommand(group_command);
}


//Subscriber callback which publishes feedback topics for a group in gazebo
void Hebiros_Node::sub_publish_group_gazebo(const boost::shared_ptr<FeedbackMsg const>
  data, std::string group_name, std::string joint_name) {

  std::lock_guard<std::mutex> guard(gazebo_feedback_mutex);

  int size = group_joints[group_name].size();
  FeedbackMsg feedback_msg;

  if (gazebo_feedback.find(group_name) == gazebo_feedback.end()) {

    feedback_msg.name.resize(size);
    feedback_msg.position.resize(size);
    feedback_msg.velocity.resize(size);
    feedback_msg.effort.resize(size);
    feedback_msg.position_command.resize(size);
    feedback_msg.velocity_command.resize(size);
    feedback_msg.effort_command.resize(size);
    feedback_msg.accelerometer.resize(size);
    feedback_msg.gyro.resize(size);

    gazebo_feedback[group_name] = feedback_msg;
  }

  feedback_msg = gazebo_feedback[group_name];

  int joint_index = group_joints[group_name][joint_name];

  feedback_msg.name[joint_index] = joint_name;
  feedback_msg.position[joint_index] = data->position[0];
  feedback_msg.velocity[joint_index] = data->velocity[0];
  feedback_msg.effort[joint_index] = data->effort[0];
  if (data->position_command.size() > 0) {
    feedback_msg.position_command[joint_index] = data->position_command[0];
  }
  if (data->velocity_command.size() > 0) {
    feedback_msg.velocity_command[joint_index] = data->velocity_command[0];
  }
  if (data->effort_command.size() > 0) {
    feedback_msg.effort_command[joint_index] = data->effort_command[0];
  }
  feedback_msg.accelerometer[joint_index] = data->accelerometer[0];
  feedback_msg.gyro[joint_index] = data->gyro[0];

  gazebo_feedback[group_name] = feedback_msg;

  for (int i = 0; i < size; i++) {
    if (feedback_msg.name[i].empty()) {
      return;
    }
  }

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = feedback_msg.name;
  joint_state_msg.position = feedback_msg.position;
  joint_state_msg.velocity = feedback_msg.velocity;
  joint_state_msg.effort = feedback_msg.effort;

  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
  group_joint_states[group_name] = joint_state_msg;
  gazebo_feedback.erase(group_name);
}


