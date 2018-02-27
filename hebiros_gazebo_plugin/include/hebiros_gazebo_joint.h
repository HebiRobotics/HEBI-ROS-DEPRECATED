
#ifndef _HEBIROS_GAZEBO_JOINT_HH_
#define _HEBIROS_GAZEBO_JOINT_HH_

#include "ros/ros.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/SettingsMsg.h"

using namespace hebiros;

class HebirosGazeboJoint {

  public:

    std::string name;
    std::string model_name;
    SettingsMsg settings;
    CommandMsg command_target;
    int command_index;
    bool command_received = false;

    ros::Time start_time;
    ros::Time prev_time;
    double prev_force {};
    double low_pass_alpha {};
    double gear_ratio {};
    double position_prev_error {};
    double position_elapsed_error {};
    double velocity_prev_error {};
    double velocity_elapsed_error {};
    double effort_prev_error {};
    double effort_elapsed_error {};

    ros::Publisher publisher;

    HebirosGazeboJoint(std::string name);
    ~HebirosGazeboJoint();
    void Reset(int i, CommandMsg command_msg);

};


#endif
