
#ifndef _HEBIROS_GAZEBO_JOINT_HH_
#define _HEBIROS_GAZEBO_JOINT_HH_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "hebiros/FeedbackMsg.h"
#include "hebiros/CommandMsg.h"
#include "hebiros/SettingsMsg.h"

using namespace hebiros;

class HebirosGazeboJoint {

  public:

    std::string name;
    std::string model_name;
    SettingsMsg settings;
    CommandMsg command_target;
    FeedbackMsg feedback;
    int command_index;
    bool command_received = false;

    ros::Time start_time;
    ros::Time prev_time;
    ros::Time prev_feedback_time;
    double prev_force {};
    double low_pass_alpha {};
    double gear_ratio {};
    double position_prev_error {};
    double position_elapsed_error {};
    double velocity_prev_error {};
    double velocity_elapsed_error {};
    double effort_prev_error {};
    double effort_elapsed_error {};

    ros::Publisher feedback_publisher;
    ros::Subscriber imu_subscriber;

    HebirosGazeboJoint(std::string name, std::shared_ptr<ros::NodeHandle> n);
    ~HebirosGazeboJoint();
    void SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data);
    void Reset(int i, CommandMsg command_msg);

};


#endif
