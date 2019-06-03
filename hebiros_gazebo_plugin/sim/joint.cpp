#include "joint.h"

#include <map>

namespace hebi {
namespace sim {

/////////////////////////////////
// TODO: refactor this to be a "Joint Info" structure, owned by the joint, and set
// on construction...
/////////////////////////////////

static constexpr double MAX_PWM = 1.0;
static constexpr double MIN_PWM = -1.0;

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

static std::map<Joint::JointType, double> gear_ratios = {
  {Joint::JointType::X5_1, 272.22},
  {Joint::JointType::X5_4, 762.22},
  {Joint::JointType::X5_9, 1742.22},
  {Joint::JointType::X8_3, 272.22},
  {Joint::JointType::X8_9, 762.22},
  {Joint::JointType::X8_16, 1462.222}
};

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
  
std::unique_ptr<Joint> Joint::tryCreate(const std::string& family, const std::string& name, const std::string& type)
{
  // Set joint type:
  JointType joint_type;
  if (type == "X5_1")
    joint_type = JointType::X5_1;
  else if (type == "X5_4")
    joint_type = JointType::X5_4;
  else if (type == "X5_9")
    joint_type = JointType::X5_9;
  else if (type == "X8_3")
    joint_type = JointType::X8_3;
  else if (type == "X8_9")
    joint_type = JointType::X8_9;
  else if (type == "X8_16")
    joint_type = JointType::X8_16;
  else
    return nullptr;

  // Create module if it is a valid type
  return std::unique_ptr<Joint>(new Joint(family + "/" + name, joint_type));
}

bool isX8(Joint::JointType jt)
{
  using JT = Joint::JointType;
  return jt == JT::X8_3 || jt == JT::X8_9 || jt == JT::X8_16;
}

Joint::Joint(const std::string& name,
  JointType joint_type)
  : joint_type_(joint_type),
    gear_ratio_(gear_ratios.at(joint_type)),
    temperature_(isX8(joint_type) ?
      hebi::sim::TemperatureModel::createX8() :
      hebi::sim::TemperatureModel::createX5()),
    name_(name),
    position_pid_(1),
    velocity_pid_(getVelocityFF(gear_ratio_, isX8(joint_type_))),
    effort_pid_(getEffortFF(gear_ratio_, isX8(joint_type_))) {

  // Set default gains:
  // TODO: cleaner way to do this? Map structure?
  // Each of these PID gains initializations is { kp, ki, kd, feed forward }
  
  // TODO: potentially load files from .xml files

  if (joint_type_ == JointType::X5_1 && control_strategy_ == ControlStrategy::Strategy2) {
    position_pid_.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_1 && control_strategy_ == ControlStrategy::Strategy3) {
    position_pid_.setGains({ 0.5f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_1 && control_strategy_ == ControlStrategy::Strategy4) {
    position_pid_.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_4 && control_strategy_ == ControlStrategy::Strategy2) {
    position_pid_.setGains({ 10.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.2f, 0.f, 0.f, 0.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_4 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid_.setGains({ 1.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_4 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid_.setGains({ 10.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_9 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid_.setGains({ 15.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.5f, 0.f, 0.f, 0.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_9 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid_.setGains({ 1.5f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X5_9 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid_.setGains({ 15.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.05f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (joint_type_ == JointType::X8_3 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid_.setGains({ 3.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_3 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid_.setGains({ 1.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_3 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid_.setGains({ 3.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_9 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid_.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_9 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid_.setGains({ 2.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_9 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid_.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_16 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy2) {
    position_pid_.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.1f, 0.f, 0.f, 0.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_16 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy3) {
    position_pid_.setGains({ 3.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (joint_type_ == JointType::X8_16 && control_strategy_ == hebi::sim::Joint::ControlStrategy::Strategy4) {
    position_pid_.setGains({ 5.f, 0.f, 0.f, 0.f });
    velocity_pid_.setGains({ 0.03f, 0.f, 0.f, 1.f });
    effort_pid_.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else {
    // Go ahead and set default gains first
    position_pid_.setGains({
      DEFAULT_POSITION_KP, DEFAULT_POSITION_KI, DEFAULT_POSITION_KD, DEFAULT_POSITION_FF });
    velocity_pid_.setGains({
      DEFAULT_VELOCITY_KP, DEFAULT_VELOCITY_KI, DEFAULT_VELOCITY_KD, DEFAULT_VELOCITY_FF });
    effort_pid_.setGains({
      DEFAULT_EFFORT_KP, DEFAULT_EFFORT_KI, DEFAULT_EFFORT_KD, DEFAULT_EFFORT_FF });
  }
}

//Update IMU feedback from external source
void Joint::updateImu(const Eigen::Vector3f& accelerometer, const Eigen::Vector3f& gyro) {
  accelerometer_ = accelerometer;
  gyro_ = gyro;
}

bool Joint::setCommand(double pos, double vel, double eff, uint64_t sender_id, double lifetime_s, SimTime t) {
  if (command_end_time_ == 0 || t > command_end_time_ || command_sender_id_ == 0 || sender_id == command_sender_id_)
  {
    command_end_time_ = (lifetime_s == 0) ? 0 : (t + lifetime_s);
    command_sender_id_ = sender_id;
    position_cmd_ = pos;
    velocity_cmd_ = vel;
    effort_cmd_ = eff;
    return true;
  }
  return false;
}

void Joint::update(SimTime t, double pos_fbk, double vel_fbk, double eff_fbk) {
  position_fbk_ = pos_fbk;
  velocity_fbk_ = vel_fbk;
  effort_fbk_ = eff_fbk;

  if (command_end_time_ == 0) {
    // Command that has no lifetime -- do nothing
  }
  else if (t > command_end_time_) {
    // Cancel the command if we are past its expiration
    command_sender_id_ = 0;
    command_end_time_ = 0;
    position_cmd_ = std::numeric_limits<double>::quiet_NaN();
    velocity_cmd_ = std::numeric_limits<double>::quiet_NaN();
    effort_cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  // Otherwise, continue with the current command.

  // TODO: calculate feedback here...
}

//Limit x to a value from low to high
static double Clip(double x, double low, double high) {
  return std::min(std::max(x, low), high);
}

void Joint::computePwm(double dt) {
  // Compute PWM command using selected strategy
  double position_pid_out, velocity_pid_out, effort_pid_out;
  double intermediate_effort;
  switch (control_strategy_) {
    case ControlStrategy::Off:
      pwm_cmd_ = 0;
      break;

    case ControlStrategy::DirectPWM:
      pwm_cmd_ = Clip(effort_cmd_, MIN_PWM, MAX_PWM);
      break;

    case ControlStrategy::Strategy2:
      position_pid_out =
        position_pid_.update(position_cmd_, position_fbk_, dt);
      velocity_pid_out =
        velocity_pid_.update(velocity_cmd_, velocity_fbk_, dt);
      intermediate_effort = effort_cmd_ + position_pid_out + velocity_pid_out;
      pwm_cmd_ = Clip(
        effort_pid_.update(intermediate_effort, effort_fbk_, dt),
        MIN_PWM, MAX_PWM);
      break;

    case ControlStrategy::Strategy3:
      position_pid_out = Clip(
        position_pid_.update(position_cmd_, position_fbk_, dt),
        MIN_PWM, MAX_PWM);
      velocity_pid_out = Clip(
        velocity_pid_.update(velocity_cmd_, velocity_fbk_, dt),
        MIN_PWM, MAX_PWM);
      effort_pid_out = Clip(
        effort_pid_.update(effort_cmd_, effort_fbk_, dt),
        MIN_PWM, MAX_PWM);
      pwm_cmd_ = Clip(position_pid_out + velocity_pid_out + effort_pid_out, MIN_PWM, MAX_PWM);
      break;

    case ControlStrategy::Strategy4:
      position_pid_out = position_pid_.update(position_cmd_, position_fbk_, dt);
      intermediate_effort = effort_cmd_ + position_pid_out;
      effort_pid_out = Clip(
        effort_pid_.update(intermediate_effort, effort_fbk_, dt),
        MIN_PWM, MAX_PWM);
      velocity_pid_out = Clip(
        velocity_pid_.update(velocity_cmd_, velocity_fbk_, dt),
        MIN_PWM, MAX_PWM);
      pwm_cmd_ = Clip(velocity_pid_out + effort_pid_out, MIN_PWM, MAX_PWM);
      break;

    default:
      pwm_cmd_ = 0;
  }
 
  // Apply safety limits
  pwm_cmd_ = temperature_safety_.limit(pwm_cmd_);
}
  
double Joint::generateForce(double dt) {
  //Set target positions
  double force, alpha;

  float voltage = 48.0f;
  float motor_velocity = velocity_fbk_ * gear_ratio_;
  float speed_constant = 1530.0f;
  float term_resist = 9.99f;
  if (isX8(joint_type_)) {
    speed_constant = 1360.0f;
    term_resist = 3.19f;
  }

  if (pwm_cmd_ == 0) {
    force = 0;
  }
  else {
    // TODO: use temp compensation here, too?
    force = ((pwm_cmd_ * voltage - (motor_velocity / speed_constant)) / term_resist) * 0.00626 * gear_ratio_ * 0.65;
  }

  float prev_winding_temp = temperature_.getMotorWindingTemperature();

  // Get components of power into the motor

  // Temperature compensated speed constant
  float comp_speed_constant = speed_constant * 1.05f * // Experimental tuning factor                           
    (1.f + .001f * (prev_winding_temp - 20.f)); // .001 is speed constant change per temperature change 
  float winding_resistance = term_resist * 
    (1.f + .004f * (prev_winding_temp - 20.f)); // .004 is resistance change per temperature change for copper 
  float back_emf = (motor_velocity * 30.f / M_PI) / comp_speed_constant;
  float winding_voltage = pwm_cmd_ * voltage - back_emf;

  // TODO: could add ripple current estimate here, too

  // Update temperature:
  // Power = I^2R, but I = V/R so I^2R = V^2/R:

  double power_in = winding_voltage * winding_voltage / winding_resistance;
  temperature_.update(power_in, dt);
  temperature_safety_.update(temperature_.getMotorWindingTemperature());

  // Low pass?

  //alpha = 0.1;
  //force = (force * alpha) + prev_force * (1 - alpha);
  //prev_force = force;

  return force;
}

} // namespace sim
} // namespace hebi
