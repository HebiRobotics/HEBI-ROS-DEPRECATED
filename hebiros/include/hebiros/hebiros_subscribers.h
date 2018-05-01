#ifndef HEBIROS_SUBSCRIBERS_H
#define HEBIROS_SUBSCRIBERS_H

#include "ros/ros.h"


class HebirosSubscribers {

  public:

    static std::map<std::string, ros::Subscriber> subscribers;

    virtual void registerGroupSubscribers(std::string group_name) {}
    static bool jointFound(std::string group_name, std::string joint_name);
    static void jointNotFound(std::string joint_name);

};

#endif
