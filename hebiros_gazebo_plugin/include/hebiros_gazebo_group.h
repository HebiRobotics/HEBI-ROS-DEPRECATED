
#ifndef _HEBIROS_GAZEBO_GROUP_HH_
#define _HEBIROS_GAZEBO_GROUP_HH_

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/SettingsMsg.h"
#include "hebiros_gazebo_joint.h"

using namespace hebiros;


class HebirosGazeboGroup : public std::enable_shared_from_this<HebirosGazeboGroup> {

  public:

    std::string name;
    std::map<std::string, std::shared_ptr<HebirosGazeboJoint>> joints;
    FeedbackMsg feedback;
    CommandMsg command_target;
    SettingsMsg settings;
    bool check_acknowledgement = false;
    bool acknowledgement = false;
    bool command_received = false;
    bool group_added = false;

    ros::Time start_time;
    ros::Time prev_time;
    ros::Time prev_feedback_time;

    ros::Subscriber command_sub;
    ros::Publisher feedback_pub;
    ros::ServiceServer acknowledge_srv;

    HebirosGazeboGroup(std::string name, std::shared_ptr<ros::NodeHandle> n);
    ~HebirosGazeboGroup();
    void SubCommand(const boost::shared_ptr<CommandMsg const> data);
    bool SrvAcknowledge(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

};


#endif
