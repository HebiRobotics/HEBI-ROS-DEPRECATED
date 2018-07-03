
#ifndef _HEBIROS_GAZEBO_JOINT_HH_
#define _HEBIROS_GAZEBO_JOINT_HH_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "hebiros_temperature_model.h"

class HebirosGazeboJoint : public std::enable_shared_from_this<HebirosGazeboJoint> {

  public:

    std::string name;
    std::string model_name;
    geometry_msgs::Vector3 accelerometer;
    geometry_msgs::Vector3 gyro;

    int feedback_index;
    int command_index;

    hebiros::sim::TemperatureModel temperature;

    double prev_force {};
    double low_pass_alpha {};
    double gear_ratio {};
    double position_prev_error {};
    double position_elapsed_error {};
    double velocity_prev_error {};
    double velocity_elapsed_error {};
    double effort_prev_error {};
    double effort_elapsed_error {};

    ros::Subscriber imu_subscriber;

    HebirosGazeboJoint(std::string name, std::shared_ptr<ros::NodeHandle> n);
    ~HebirosGazeboJoint();
    void SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data);

};


#endif
