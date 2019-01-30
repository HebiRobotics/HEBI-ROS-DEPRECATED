#include "hebiros_subscribers_gazebo.h"
#include "hebiros_group_registry.h"
#include "hebiros.h"

using namespace hebiros;


void HebirosSubscribersGazebo::registerGroupSubscribers(std::string group_name) {

  subscribers["hebiros/"+group_name+"/command"] =
    HebirosNode::n_ptr->subscribe<CommandMsg>("hebiros/"+group_name+"/command", 100,
    boost::bind(&HebirosSubscribersGazebo::command, this, _1, group_name));

  subscribers["hebiros/"+group_name+"/command/joint_state"] =
    HebirosNode::n_ptr->subscribe<sensor_msgs::JointState>(
    "hebiros/"+group_name+"/command/joint_state", 100,
    boost::bind(&HebirosSubscribersGazebo::jointCommand, this, _1, group_name));

  subscribers["hebiros_gazebo_plugin/feedback/"+group_name] =
    HebirosNode::n_ptr->subscribe<FeedbackMsg>(
    "hebiros_gazebo_plugin/feedback/"+group_name, 100,
    boost::bind(&HebirosSubscribersGazebo::feedback, this, _1, group_name));
}

void HebirosSubscribersGazebo::command(const boost::shared_ptr<CommandMsg const> data,
  std::string group_name) {

  HebirosNode::publishers_gazebo.command(*data, group_name);
}

void HebirosSubscribersGazebo::jointCommand(
  const boost::shared_ptr<sensor_msgs::JointState const> data, std::string group_name) {

  CommandMsg command_msg;
  command_msg.name = data->name;
  command_msg.position = data->position;
  command_msg.velocity = data->velocity;
  command_msg.effort = data->effort;

  HebirosNode::publishers_gazebo.command(command_msg, group_name);
}

void HebirosSubscribersGazebo::feedback(const boost::shared_ptr<FeedbackMsg const> data,
  std::string group_name) {

  // TODO: replace with better abstraction later
  HebirosGroupGazebo* group = dynamic_cast<HebirosGroupGazebo*>
    (hebiros::HebirosGroupRegistry::Instance().getGroup(group_name));
  if (!group) {
    ROS_WARN("Improper group type during feedback call");
    return;
  }

  FeedbackMsg feedback_msg = *data;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.name = feedback_msg.name;
  joint_state_msg.position = feedback_msg.position;
  joint_state_msg.velocity = feedback_msg.velocity;
  joint_state_msg.effort = feedback_msg.effort;

  group->feedback_msg = feedback_msg;
  group->joint_state_msg = joint_state_msg;

  HebirosNode::publishers_gazebo.feedback(feedback_msg, group_name);
  HebirosNode::publishers_gazebo.feedbackJointState(joint_state_msg, group_name);
}




