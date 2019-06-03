#pragma once

#include <memory>

#include "Eigen/Dense"

#include "pid_controller.h"
#include "temperature_model.h"
#include "temperature_safety_controller.h"

namespace hebi {
namespace sim {

// Generally, each "OnUpdate" call of the main loop, the joint should:
// (1) have any commands set w/ `setCommand` (this can be called multiple times per cycle; only
// one will end up taking effect, depending on which senders are locked out)
// (2) have the `update` function run, which generates its PWM command
// (3) have its feedback set
class Joint : public std::enable_shared_from_this<Joint> {

public:
  enum class JointType {
    X5_1, X5_4, X5_9,
    X8_3, X8_9, X8_16
  };

  enum class ControlStrategy {
    Off = 0,
    DirectPWM = 1,
    Strategy2 = 2,
    Strategy3 = 3,
    Strategy4 = 4
  };

  using SimTime = double;

  static std::unique_ptr<Joint> tryCreate(const std::string& family, const std::string& name, const std::string& type);

  void updateImu(const Eigen::Vector3f& accelerometer, const Eigen::Vector3f& gyro);
  const Eigen::Vector3f getAccelerometer() { return accelerometer_; }
  const Eigen::Vector3f getGyro() { return gyro_; }
  const hebi::sim::TemperatureModel& getTemperature() { return temperature_; }

  hebi::sim::PidController& getPositionPid() { return position_pid_; }
  hebi::sim::PidController& getVelocityPid() { return velocity_pid_; }
  hebi::sim::PidController& getEffortPid() { return effort_pid_; }

  std::string getName() { return name_; }
  void setName(const std::string& name) { name_ = name; }

  // TODO: think about resetting gains on a control strategy switch, as with the modules; potentially
  // add a "controller" child object for each joint at end of refactor
  void setControlStrategy(ControlStrategy strategy) { control_strategy_ = strategy; }
  ControlStrategy getControlStrategy() const { return control_strategy_; }

  double getPositionCmd() { return position_cmd_; }
  double getVelocityCmd() { return velocity_cmd_; }
  double getEffortCmd() { return effort_cmd_; }
  double getPwmCmd() { return pwm_cmd_; }

  double getPositionFbk() { return position_fbk_; }
  double getVelocityFbk() { return velocity_fbk_; }
  double getEffortFbk() { return effort_fbk_; }

  // Try to set this command on the joint.  Return "false" if it was ignored (e.g., this module was
  // locked out).
  bool setCommand(double pos, double vel, double eff, uint64_t sender_id, double lifetime_s, SimTime t);

  // Should be called each sim cycle.  Updates command lockouts and generates feedback.
  void update(SimTime t, double pos_fbk, double vel_fbk, double eff_fbk);

  // TODO: consider calling "computePwm" and "generateForce" from update(), and making them private.

  // The position/velocity/effort feedback should be set before this is called; this generates
  // PWM command to send to the "motor"
  void computePwm(double dt);

  // Generate an effort command for the virtual motor output, based on the commanded PWM signal.
  // Also updates temperature model with this information.
  double generateForce(double dt);

private:
  Joint(const std::string& name, JointType joint_type);

  const JointType joint_type_;
  const double gear_ratio_;

  //////////////////////////////////////////////////////////////////////////////
  // Internals/Controllers/Settings:
  //////////////////////////////////////////////////////////////////////////////

  hebi::sim::TemperatureModel temperature_;
  hebi::sim::TemperatureSafetyController temperature_safety_{155};
  
  // Module name -- note that this should be kept consistent with the physics' joint
  std::string name_;

  hebi::sim::PidController position_pid_;
  hebi::sim::PidController velocity_pid_;
  hebi::sim::PidController effort_pid_;

  ControlStrategy control_strategy_{ControlStrategy::Strategy3};

  //////////////////////////////////////////////////////////////////////////////
  // Commands:
  //////////////////////////////////////////////////////////////////////////////

  SimTime command_end_time_ {};
  // If non-zero, this is the sender corresponding to the timeout above. If command end
  // time is zero, this is ignored; otherwise, this is the only sender that can send
  // commands to the module.
  uint64_t command_sender_id_ {};

  // Currently commanded position, velocity, and effort.
  double position_cmd_ { std::numeric_limits<double>::quiet_NaN() };
  double velocity_cmd_ { std::numeric_limits<double>::quiet_NaN() };
  double effort_cmd_ { std::numeric_limits<double>::quiet_NaN() };

  // Internal PWM command; set from PID loops, and computed during update().
  double pwm_cmd_ {};

  //////////////////////////////////////////////////////////////////////////////
  // Feedback:
  //////////////////////////////////////////////////////////////////////////////

  // Note -- the accelerometer and gyro feedback must be updated from an external source.
  Eigen::Vector3f accelerometer_;
  Eigen::Vector3f gyro_;

  // Feedback; set immediately after "update" call.
  double position_fbk_ {};
  double velocity_fbk_ {};
  double effort_fbk_ {};

};

} // namespace sim
} // namespace hebi
