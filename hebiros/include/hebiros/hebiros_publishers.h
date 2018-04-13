#ifndef HEBIROS_PUBLISHERS_H
#define HEBIROS_PUBLISHERS_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "hebiros/FeedbackMsg.h"


class HebirosPublishers {

  public:

    static std::map<std::string, ros::Publisher> publishers;

    void registerGroupPublishers(std::string group_name);
    void feedback(std::string group_name, hebiros::FeedbackMsg feedback_msg);
    void feedbackJointState(std::string group_name, sensor_msgs::JointState joint_state_msg);
    void commandJointState(std::string group_name, sensor_msgs::JointState joint_state_msg);

};

#endif
