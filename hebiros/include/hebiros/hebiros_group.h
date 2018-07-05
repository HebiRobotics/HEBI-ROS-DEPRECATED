#pragma once

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "hebiros/FeedbackMsg.h"

// Base class for physical and simulated groups of modules.
class HebirosGroup {

  public:

    HebirosGroup();
    virtual ~HebirosGroup() {};

    int size;
    std::map<std::string, std::string> joint_full_names;
    std::map<std::string, int> joints;
    sensor_msgs::JointState joint_state_msg;
    hebiros::FeedbackMsg feedback_msg;

    virtual void setFeedbackFrequencyHz(float frequency);
    virtual void setCommandLifetimeMs(float lifetime);
};
