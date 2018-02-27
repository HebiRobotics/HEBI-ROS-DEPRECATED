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
void Hebiros_Node::sub_publish_group_gazebo(const boost::shared_ptr<sensor_msgs::JointState const>
  data, std::string group_name, std::string joint_name) {

  std::lock_guard<std::mutex> guard(gazebo_joint_states_mutex);

  int size = group_joints[group_name].size();
  sensor_msgs::JointState joint_state_msg;

  if (gazebo_joint_states.find(group_name) == gazebo_joint_states.end()) {

    joint_state_msg.name.resize(size);
    joint_state_msg.position.resize(size);
    joint_state_msg.velocity.resize(size);
    joint_state_msg.effort.resize(size);
    gazebo_joint_states[group_name] = joint_state_msg;
  }

  joint_state_msg = gazebo_joint_states[group_name];

  double position = data->position[0];
  double velocity = data->velocity[0];
  double effort = data->effort[0];
  int joint_index = group_joints[group_name][joint_name];

  joint_state_msg.name[joint_index] = joint_name;
  joint_state_msg.position[joint_index] = position;
  joint_state_msg.velocity[joint_index] = velocity;
  joint_state_msg.effort[joint_index] = effort;

  gazebo_joint_states[group_name] = joint_state_msg;

  for (int i = 0; i < size; i++) {
    if (joint_state_msg.name[i].empty()) {
      return;
    }
  }

  FeedbackMsg feedback_msg;
  feedback_msg.name = joint_state_msg.name;
  feedback_msg.position = joint_state_msg.position;
  feedback_msg.velocity = joint_state_msg.velocity;
  feedback_msg.effort = joint_state_msg.effort;

  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
  group_joint_states[group_name] = joint_state_msg;
  gazebo_joint_states.erase(group_name);
}


