#include "hebiros.hpp"


//Subscriber callback which receives commands and sends them to a group
void Hebiros_Node::sub_command(const boost::shared_ptr<CommandMsg const> data,
  std::string group_name) {

  if (use_gazebo) {
    std_msgs::Float64 command_msg;
    for (int i = 0; i < data->name.size(); i++) {
      std::string joint_name = data->name[i];
      command_msg.data = data->effort[i];
      publishers["/hebiros/"+group_name+"/"+joint_name+"/controller/command"].publish(
        command_msg);
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
    std_msgs::Float64 command_msg;
    for (int i = 0; i < data->name.size(); i++) {
      std::string joint_name = data->name[i];
      command_msg.data = data->effort[i];
      publishers["/hebiros/"+group_name+"/"+joint_name+"/controller/command"].publish(
        command_msg);
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

  add_joint_command(&group_command, joint_data, group_name);

  group->sendCommand(group_command);
}


//Subscriber callback which publishes feedback topics for a group in gazebo
void Hebiros_Node::sub_publish_group_gazebo(const boost::shared_ptr<sensor_msgs::JointState const>
  data, std::string group_name) {
  int size = data->name.size();

  FeedbackMsg feedback_msg;
  feedback_msg.name.resize(size);
  feedback_msg.position.resize(size);
  feedback_msg.velocity.resize(size);
  feedback_msg.effort.resize(size);

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name.resize(size);
  joint_state_msg.position.resize(size);
  joint_state_msg.velocity.resize(size);
  joint_state_msg.effort.resize(size);

  for (int i = 0; i < size; i++) {

    std::string joint_name = data->name[i];
    double position = data->position[i];
    double velocity = data->velocity[i];
    double effort = data->effort[i];
    int joint_index = group_joints[group_name][joint_name];

    joint_state_msg.name[joint_index] = joint_name;
    joint_state_msg.position[joint_index] = position;
    joint_state_msg.velocity[joint_index] = velocity;
    joint_state_msg.effort[joint_index] = effort;

    feedback_msg.name[joint_index] = joint_name;
    feedback_msg.position[joint_index] = position;
    feedback_msg.velocity[joint_index] = velocity;
    feedback_msg.effort[joint_index] = effort;
  }

  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
  group_joint_states[group_name] = joint_state_msg;
}


