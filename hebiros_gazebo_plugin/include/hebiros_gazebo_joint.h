#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "temperature_model.h"
#include "pid.h"
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

  hebi::sim::TemperatureModel temperature;
  hebi::sim::TemperatureSafetyController temperature_safety{155};

  double gear_ratio {};


  hebi::sim::PidController position_pid;
  hebi::sim::PidController velocity_pid;
  hebi::sim::PidController effort_pid;

  double prev_force {};
  double low_pass_alpha {};

  ros::Subscriber imu_subscriber;

  HebirosGazeboJoint(const std::string& name, const std::string& model_name, bool is_x8, std::shared_ptr<ros::NodeHandle> n);

  void SubIMU(const boost::shared_ptr<sensor_msgs::Imu const> data);
  bool isX8() const;

  // TODO: think about resetting gains on a control strategy switch, as with the modules; potentially
  // add a "controller" child object for each joint at end of refactor
  void setControlStrategy(uint8_t strategy) { control_strategy = strategy; }
  uint8_t getControlStrategy() const { return control_strategy; }

private:
  // TODO: in refactor, set this during construction; right now, this
  // is tangled up in HebirosGazeboController::SetSettings and the plugin
  // add modules to group logic; this will be changed.
  uint8_t control_strategy{};
};
