#pragma once

#include <memory>

#include "Eigen/Dense"

#include "pid_controller.h"
#include "temperature_model.h"
#include "temperature_safety_controller.h"

namespace hebi {
namespace sim {

class Joint : public std::enable_shared_from_this<Joint> {

public:

  Joint(const std::string& name, const std::string& model_name, bool is_x8);

  // TODO: Make these private.
  std::string name;
  std::string model_name;

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

  void updateImu(const Eigen::Vector3f& accelerometer, const Eigen::Vector3f& gyro);
  const Eigen::Vector3f getAccelerometer() { return accelerometer_; }
  const Eigen::Vector3f getGyro() { return gyro_; }
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

  // Note -- the accelerometer and gyro feedback must be updated from an external source.
  // TODO: we should store the position / etc feedback alongside this...
  Eigen::Vector3f accelerometer_;
  Eigen::Vector3f gyro_;
};

} // namespace sim
} // namespace hebi
