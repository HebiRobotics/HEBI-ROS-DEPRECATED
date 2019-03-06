#include "hebiros_publishers.h"
#include "hebiros_group_registry.h"

#include "hebiros.h"

using namespace hebiros;


std::map<std::string, ros::Publisher> HebirosPublishers::publishers;

void HebirosPublishers::registerGroupPublishers(std::string group_name) {

  publishers["hebiros/"+group_name+"/feedback"] =
    HebirosNode::n_ptr->advertise<FeedbackMsg>("hebiros/"+group_name+"/feedback", 100);

  publishers["hebiros/"+group_name+"/feedback/joint_state"] =
    HebirosNode::n_ptr->advertise<sensor_msgs::JointState>(
    "hebiros/"+group_name+"/feedback/joint_state", 100);

  publishers["hebiros/"+group_name+"/feedback/joint_state_urdf"] =
    HebirosNode::n_ptr->advertise<sensor_msgs::JointState>(
    "hebiros/"+group_name+"/feedback/joint_state_urdf", 100);

  publishers["hebiros/"+group_name+"/command/joint_state"] =
    HebirosNode::n_ptr->advertise<sensor_msgs::JointState>(
    "hebiros/"+group_name+"/command/joint_state", 100);
}


void HebirosPublishers::feedback(FeedbackMsg feedback_msg, std::string group_name) {
  publishers["hebiros/"+group_name+"/feedback"].publish(feedback_msg);
}

void HebirosPublishers::feedbackJointState(sensor_msgs::JointState joint_state_msg,
  std::string group_name) {
  publishers["hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);

  feedbackJointStateUrdf(joint_state_msg, group_name);
}

void HebirosPublishers::feedbackJointStateUrdf(sensor_msgs::JointState joint_state_msg,
  std::string group_name) {

  HebirosGroup* group = HebirosGroupRegistry::Instance().getGroup(group_name);

  if (joint_state_msg.name.size() == group->joint_full_names.size()) {
    for (int i = 0; i < group->joint_full_names.size(); i++) {
      joint_state_msg.name[i] = group->joint_full_names[joint_state_msg.name[i]];
    }

    publishers["hebiros/"+group_name+"/feedback/joint_state_urdf"].publish(joint_state_msg);
  }
}

void HebirosPublishers::commandJointState(sensor_msgs::JointState joint_state_msg,
  std::string group_name) {
  publishers["hebiros/"+group_name+"/command/joint_state"].publish(joint_state_msg);
}
