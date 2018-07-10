#ifndef HEBIROS_SUBSCRIBERS_PHYSICAL_H
#define HEBIROS_SUBSCRIBERS_PHYSICAL_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hebiros/CommandMsg.h"

#include "hebiros_subscribers.h"
#include "group_feedback.hpp"
#include "group_command.hpp"


class HebirosSubscribersPhysical : public HebirosSubscribers {

  public:

    void registerGroupSubscribers(std::string group_name);
    void command(const boost::shared_ptr<hebiros::CommandMsg const> data,
      std::string group_name);
    void jointCommand(const boost::shared_ptr<sensor_msgs::JointState const> data,
      std::string group_name);
    void feedback(std::string group_name, const hebi::GroupFeedback& group_fbk);
    static void addJointCommand(hebi::GroupCommand* group_command,
      sensor_msgs::JointState data, std::string group_name);
    static void addSettingsCommand(hebi::GroupCommand* group_command,
      hebiros::SettingsMsg data, std::string group_name);
    static void addPositionGainsCommand(hebi::GroupCommand* group_command,
      hebiros::PidGainsMsg data, std::string group_name);
    static void addVelocityGainsCommand(hebi::GroupCommand* group_command,
      hebiros::PidGainsMsg data, std::string group_name);
    static void addEffortGainsCommand(hebi::GroupCommand* group_command,
      hebiros::PidGainsMsg data, std::string group_name);

};

#endif
