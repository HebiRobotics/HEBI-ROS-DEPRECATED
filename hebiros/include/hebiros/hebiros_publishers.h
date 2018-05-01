#ifndef HEBIROS_PUBLISHERS_H
#define HEBIROS_PUBLISHERS_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "hebiros/FeedbackMsg.h"


class HebirosPublishers {

  public:

    static std::map<std::string, ros::Publisher> publishers;

    void registerGroupPublishers(std::string group_name);
    void feedback(hebiros::FeedbackMsg feedback_msg, std::string group_name);
    void feedbackJointState(sensor_msgs::JointState joint_state_msg, std::string group_name);
    void feedbackJointStateUrdf(sensor_msgs::JointState joint_state_msg, std::string group_name);
    void commandJointState(sensor_msgs::JointState joint_state_msg, std::string group_name);

};

#endif
