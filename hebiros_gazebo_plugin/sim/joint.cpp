#include "joint.h"

#include <map>

namespace hebi {
namespace sim {

/////////////////////////////////
// TODO: refactor this to be a "Joint Info" structure, owned by the joint, and set
// on construction...
/////////////////////////////////

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
  : name(name_), model_name(model_name_),
    temperature(is_x8 ?
      hebi::sim::TemperatureModel::createX8() :
      hebi::sim::TemperatureModel::createX5()),
    gear_ratio(getGearRatio(model_name)),
    position_pid(1),
    velocity_pid(getVelocityFF(gear_ratio, is_x8)),
    effort_pid(getEffortFF(gear_ratio, is_x8)) {
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
  }
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
