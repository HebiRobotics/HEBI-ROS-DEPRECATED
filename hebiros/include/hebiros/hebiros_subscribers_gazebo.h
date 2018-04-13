#ifndef HEBIROS_SUBSCRIBERS_GAZEBO_H
#define HEBIROS_SUBSCRIBERS_GAZEBO_H

#include "ros/ros.h"

#include "hebiros_subscribers.h"


class HebirosSubscribersGazebo : public HebirosSubscribers {

  public:

    void registerGroupSubscribers(std::string group_name);

};

#endif
