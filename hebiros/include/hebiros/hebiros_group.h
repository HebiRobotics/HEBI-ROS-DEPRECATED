#ifndef HEBIROS_GROUP_H
#define HEBIROS_GROUP_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "hebiros/FeedbackMsg.h"


class HebirosGroup {

  public:

    static std::map<std::string, std::shared_ptr<HebirosGroup>> groups;

    std::string name;
    int size;
    std::map<std::string, std::string> joint_full_names;
    std::map<std::string, int> joints;
    sensor_msgs::JointState joint_state_msg;
    hebiros::FeedbackMsg feedback_msg;

    HebirosGroup(std::string name);
    static std::shared_ptr<HebirosGroup> getGroup(std::string name);
    static void removeGroup(std::string name);
    static bool findGroup(std::string name);

};

#endif
