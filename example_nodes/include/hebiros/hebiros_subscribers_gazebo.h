#ifndef HEBIROS_SUBSCRIBERS_GAZEBO_H
#define HEBIROS_SUBSCRIBERS_GAZEBO_H

#include "sensor_msgs/JointState.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"

#include "hebiros_subscribers.h"


class HebirosSubscribersGazebo : public HebirosSubscribers {

  public:

    void registerGroupSubscribers(std::string group_name);
    void command(const boost::shared_ptr<hebiros::CommandMsg const> data,
      std::string group_name);
    void jointCommand(const boost::shared_ptr<sensor_msgs::JointState const> data,
      std::string group_name);
    void feedback(const boost::shared_ptr<hebiros::FeedbackMsg const> data,
      std::string group_name);

};

#endif
