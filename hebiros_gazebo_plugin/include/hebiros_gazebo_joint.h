#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "hebiros_temperature_model.h"
#include "temperature_safety_controller.h"

class HebirosGazeboJoint : public std::enable_shared_from_this<HebirosGazeboJoint> {

public:

  // TODO: Make these private.
  std::string name;
  std::string model_name;
  geometry_msgs::Vector3 accelerometer;
  geometry_msgs::Vector3 gyro;

  int feedback_index;
  int command_index;

  hebiros::sim::TemperatureModel temperature;
  hebiros::sim::TemperatureSafetyController temperature_safety{155};

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

  HebirosGazeboJoint(const std::string& name, const std::string& model_name, bool is_x8, std::shared_ptr<ros::NodeHandle> n);

  void SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data);
  bool isX8() const;

};
