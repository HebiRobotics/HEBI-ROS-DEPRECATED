#include "joint.h"

#include <map>

namespace hebi {
namespace sim {

/////////////////////////////////
// TODO: refactor this to be a "Joint Info" structure, owned by the joint, and set
// on construction...
/////////////////////////////////

static constexpr double LOW_PASS_ALPHA = 0.1;

static constexpr double DEFAULT_POSITION_KP = 0.5;
static constexpr double DEFAULT_POSITION_KI = 0.0;
static constexpr double DEFAULT_POSITION_KD = 0.0;
static constexpr double DEFAULT_POSITION_FF = 0.0;
static constexpr double DEFAULT_VELOCITY_KP = 0.05;
static constexpr double DEFAULT_VELOCITY_KI = 0.0;
static constexpr double DEFAULT_VELOCITY_KD = 0.0;
static constexpr double DEFAULT_VELOCITY_FF = 1.0;
static constexpr double DEFAULT_EFFORT_KP = 0.25;
static constexpr double DEFAULT_EFFORT_KI = 0.0;
static constexpr double DEFAULT_EFFORT_KD = 0.001;
static constexpr double DEFAULT_EFFORT_FF = 1.0;

static constexpr double GEAR_RATIO_X5_1 = 272.22;
static constexpr double GEAR_RATIO_X8_3 = 272.22;
static constexpr double GEAR_RATIO_X5_4 = 762.22;
static constexpr double GEAR_RATIO_X8_9 = 762.22;
static constexpr double GEAR_RATIO_X5_9 = 1742.22;
static constexpr double GEAR_RATIO_X8_16 = 1462.222;

static constexpr double DEFAULT_GEAR_RATIO = 272.22;

static std::map<std::string, double> gear_ratios = {
  {"X5_1", GEAR_RATIO_X5_1},
  {"X5_4", GEAR_RATIO_X5_4},
  {"X5_9", GEAR_RATIO_X5_9},
  {"X8_3", GEAR_RATIO_X8_3},
  {"X8_9", GEAR_RATIO_X8_9},
  {"X8_16", GEAR_RATIO_X8_16}
};

double getGearRatio(const std::string& model_name) {
  if (gear_ratios.find(model_name) != gear_ratios.end()) {
    return gear_ratios[model_name];
  }
  return DEFAULT_GEAR_RATIO;
}

double getVelocityFF(double gear_ratio, bool is_x8) {
  double voltage = 48;
  double speed_constant = is_x8 ? 1360 : 1530;
  return gear_ratio / (speed_constant * (2.0 * M_PI / 60.0) * voltage);
}

double getEffortFF(double gear_ratio, bool is_x8) {
  double voltage = 48;
  double term_resistance = is_x8 ? 1360 : 1530;
  double torque_constant = is_x8 ? (9.32f / 1000.0f) : (6.26f / 1000.0f);
  return term_resistance / (gear_ratio * voltage * torque_constant * 0.65);
}

/////////////////////////////////
// END REFACTOR TODO
/////////////////////////////////

Joint::Joint(const std::string& name_,
  const std::string& model_name_,
  bool is_x8) // TODO: do this better...
  : name(name_),
    temperature(is_x8 ?
      hebi::sim::TemperatureModel::createX8() :
      hebi::sim::TemperatureModel::createX5()),
    gear_ratio(getGearRatio(model_name_)),
    position_pid(1),
    velocity_pid(getVelocityFF(gear_ratio, is_x8)),
    effort_pid(getEffortFF(gear_ratio, is_x8)),
    low_pass_alpha(LOW_PASS_ALPHA),
    model_name(model_name_) {

  // Set joint type:
  if (model_name == "X5_1")
    joint_type = JointType::X5_1;
  else if (model_name == "X5_4")
    joint_type = JointType::X5_4;
  else if (model_name == "X5_9")
    joint_type = JointType::X5_9;
  else if (model_name == "X8_3")
    joint_type = JointType::X8_3;
  else if (model_name == "X8_9")
    joint_type = JointType::X8_9;
  else if (model_name == "X8_16")
    joint_type = JointType::X8_16;
  else
    joint_type = JointType::Unknown;

  // Set default gains:
  // TODO: cleaner way to do this? Map structure?
  // Each of these PID gains initializations is { kp, ki, kd, feed forward }
  
  // TODO: move all this logic into joint-specific code, and potentially load files from .xml
  // files

  if (model_name == "X5_1" && control_strategy == ControlStrategy::Strategy2) {
    position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_1" && control_strategy == ControlStrategy::Strategy3) {
    position_pid.setGains({ 0.5f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_1" && control_strategy == ControlStrategy::Strategy4) {
    position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == ControlStrategy::Strategy2) {
    position_pid.setGains({ 10.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.2f, 0.f, 0.f, 0.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid.setGains({ 1.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid.setGains({ 10.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid.setGains({ 15.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.5f, 0.f, 0.f, 0.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid.setGains({ 1.5f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid.setGains({ 15.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid.setGains({ 1.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid.setGains({ 2.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else {
    // Go ahead and set default gains first
    position_pid.setGains({
      DEFAULT_POSITION_KP, DEFAULT_POSITION_KI, DEFAULT_POSITION_KD, DEFAULT_POSITION_FF });
    velocity_pid.setGains({
      DEFAULT_VELOCITY_KP, DEFAULT_VELOCITY_KI, DEFAULT_VELOCITY_KD, DEFAULT_VELOCITY_FF });
    effort_pid.setGains({
      DEFAULT_EFFORT_KP, DEFAULT_EFFORT_KI, DEFAULT_EFFORT_KD, DEFAULT_EFFORT_FF });
  }
}

//Update IMU feedback from external source
void Joint::updateImu(const Eigen::Vector3f& accelerometer, const Eigen::Vector3f& gyro) {
  accelerometer_ = accelerometer;
  gyro_ = gyro;
}

bool Joint::isX8() const {
  return (model_name == "X8_3" || 
          model_name == "X8_9" ||
          model_name == "X8_16");
}

bool Joint::setCommand(double pos, double vel, double eff, uint64_t sender_id, double lifetime_s, SimTime t) {
  if (command_end_time == 0 || t > command_end_time || command_sender_id == 0 || sender_id == command_sender_id)
  {
    command_end_time = (lifetime_s == 0) ? 0 : (t + lifetime_s);
    command_sender_id = sender_id;
    position_cmd = pos;
    velocity_cmd = vel;
    effort_cmd = eff;
    return true;
  }
  return false;
}

void Joint::update(SimTime t) {
  if (command_end_time == 0) {
    // Command that has no lifetime -- do nothing
  }
  else if (t > command_end_time) {
    // Cancel the command if we are past its expiration
    command_sender_id = 0;
    command_end_time = 0;
    position_cmd = std::numeric_limits<double>::quiet_NaN();
    velocity_cmd = std::numeric_limits<double>::quiet_NaN();
    effort_cmd = std::numeric_limits<double>::quiet_NaN();
  }

  // Otherwise, continue with the current command.

  // TODO: calculate feedback here...
}

} // namespace sim
} // namespace hebi
