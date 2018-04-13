#include "hebiros_publishers.h"

#include "hebiros.h"

using namespace hebiros;


std::map<std::string, ros::Publisher> HebirosPublishers::publishers;

void HebirosPublishers::registerGroupPublishers(std::string group_name) {

  publishers["/hebiros/"+group_name+"/feedback"] =
    HebirosNode::n_ptr->advertise<FeedbackMsg>("/hebiros/"+group_name+"/feedback", 100);

  publishers["/hebiros/"+group_name+"/feedback/joint_state"] =
    HebirosNode::n_ptr->advertise<sensor_msgs::JointState>(
    "/hebiros/"+group_name+"/feedback/joint_state", 100);

  publishers["/hebiros/"+group_name+"/command/joint_state"] =   
    HebirosNode::n_ptr->advertise<sensor_msgs::JointState>(
    "/hebiros/"+group_name+"/command/joint_state", 100);
}


void HebirosPublishers::feedback(std::string group_name, FeedbackMsg feedback_msg) {
  publishers["/hebiros/"+group_name+"/feedback"].publish(feedback_msg);
}

void HebirosPublishers::feedbackJointState(std::string group_name,
  sensor_msgs::JointState joint_state_msg) {
  publishers["/hebiros/"+group_name+"/feedback/joint_state"].publish(joint_state_msg);
}

void HebirosPublishers::commandJointState(std::string group_name,
  sensor_msgs::JointState joint_state_msg) {
  publishers["/hebiros/"+group_name+"/command/joint_state"].publish(joint_state_msg);
}

