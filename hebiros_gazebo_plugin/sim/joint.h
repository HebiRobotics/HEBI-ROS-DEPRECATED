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

  enum class ControlStrategy {
    Off = 0,
    DirectPWM = 1,
    Strategy2 = 2,
    Strategy3 = 3,
    Strategy4 = 4
  };

  using SimTime = double;

  Joint(const std::string& name, const std::string& model_name, bool is_x8);

  // TODO: Make these private.
  std::string name;
  std::string model_name;

  // TODO: REMOVE!
  int feedback_index;

  hebi::sim::TemperatureModel temperature;
  hebi::sim::TemperatureSafetyController temperature_safety{155};

  double gear_ratio {};

  // TODO: make private; update settings through "setSettings"...but then we need a lot of
  // "optional" logic...perhaps better to handle this in the wrapper classes and expose more here.
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
  void setControlStrategy(ControlStrategy strategy) { control_strategy = strategy; }
  ControlStrategy getControlStrategy() const { return control_strategy; }

  // Try to set this command on the joint.  Return "false" if it was ignored (e.g., this module was
  // locked out).
  bool setCommand(double pos, double vel, double eff, uint64_t sender_id, double lifetime_s, SimTime t);

  // Should be called each sim cycle.  Updates command lockouts and generates feedback.
  void update(SimTime t);

  // TODO: make all this private when refactor is done
  SimTime command_end_time {};
  // If non-zero, this is the sender corresponding to the timeout above. If command end
  // time is zero, this is ignored; otherwise, this is the only sender that can send
  // commands to the module.
  uint64_t command_sender_id {};

  double position_cmd { std::numeric_limits<double>::quiet_NaN() };
  double velocity_cmd { std::numeric_limits<double>::quiet_NaN() };
  double effort_cmd { std::numeric_limits<double>::quiet_NaN() };

  // Internal PWM command; set from PID loops, and computed during update().
  double pwm_cmd {};

  // Feedback; set immediately after "update" call.
  double position_fbk {};
  double velocity_fbk {};
  double effort_fbk {};

private:
  ControlStrategy control_strategy{ControlStrategy::Strategy3};

  // Note -- the accelerometer and gyro feedback must be updated from an external source.
  // TODO: we should store the position / etc feedback alongside this...
  Eigen::Vector3f accelerometer_;
  Eigen::Vector3f gyro_;
};

} // namespace sim
} // namespace hebi
