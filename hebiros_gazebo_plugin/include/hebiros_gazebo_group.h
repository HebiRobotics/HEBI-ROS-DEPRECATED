
#ifndef _HEBIROS_GAZEBO_GROUP_HH_
#define _HEBIROS_GAZEBO_GROUP_HH_

#include "ros/ros.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros_gazebo_joint.h"


using namespace hebiros;


class HebirosGazeboGroup {

  public:

    std::string name;
    std::map<std::string, std::shared_ptr<HebirosGazeboJoint>> joints;
    FeedbackMsg feedback_msg;
    CommandMsg command_target;

    ros::Subscriber command_sub;
    ros::Publisher feedback_pub;

    HebirosGazeboGroup(std::string name);
    ~HebirosGazeboGroup();

};


#endif
