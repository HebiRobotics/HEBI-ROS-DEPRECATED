#include <hebiros_gazebo_controller.h>
#include "pid_controller.h"

namespace controller {

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

}

using namespace controller;

//Initialize gains with default values based on model and control strategy
void HebirosGazeboController::SetDefaultGains(
  hebi::sim::Joint* hebiros_joint) {
  
  std::string model_name = hebiros_joint->model_name;
  auto control_strategy = hebiros_joint->getControlStrategy();

  // TODO: move all this logic into joint-specific code, and potentially load files from .xml
  // files

  // Each of these PID gains initializations is { kp, ki, kd, feed forward }
  if (model_name == "X5_1" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_1" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    hebiros_joint->position_pid.setGains({ 0.5f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_1" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    hebiros_joint->position_pid.setGains({ 10.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.2f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    hebiros_joint->position_pid.setGains({ 1.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    hebiros_joint->position_pid.setGains({ 10.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    hebiros_joint->position_pid.setGains({ 15.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.5f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    hebiros_joint->position_pid.setGains({ 1.5f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    hebiros_joint->position_pid.setGains({ 15.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    hebiros_joint->position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    hebiros_joint->position_pid.setGains({ 1.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    hebiros_joint->position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    hebiros_joint->position_pid.setGains({ 2.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy2) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy3) {
    hebiros_joint->position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == hebi::sim::Joint::ControlStrategy::Strategy4) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else {
    // Go ahead and set default gains first
    hebiros_joint->position_pid.setGains({
      DEFAULT_POSITION_KP, DEFAULT_POSITION_KI, DEFAULT_POSITION_KD, DEFAULT_POSITION_FF });
    hebiros_joint->velocity_pid.setGains({
      DEFAULT_VELOCITY_KP, DEFAULT_VELOCITY_KI, DEFAULT_VELOCITY_KD, DEFAULT_VELOCITY_FF });
    hebiros_joint->effort_pid.setGains({
      DEFAULT_EFFORT_KP, DEFAULT_EFFORT_KI, DEFAULT_EFFORT_KD, DEFAULT_EFFORT_FF });
  }
}

// TODO: the conversion helper functions will be moved at later point during refactoring
hebi::sim::PidGains convertToSimGains(const hebiros::PidGainsMsg& msg, size_t index) {
  return hebi::sim::PidGains {
    static_cast<float>(msg.kp[index]),
    static_cast<float>(msg.ki[index]),
    static_cast<float>(msg.kd[index]),
    static_cast<float>(msg.feed_forward[index])
  };
}

//Compute output force to the joint based on PID and control strategy
double HebirosGazeboController::ComputeForce(
  hebi::sim::Joint* hebiros_joint,
  double position, double velocity, double effort, double iteration_time) {

  auto dt = iteration_time;

  //Set target positions
  double target_position = hebiros_joint->position_cmd;
  double target_velocity = hebiros_joint->velocity_cmd;
  double target_effort = hebiros_joint->effort_cmd;

  double position_pid, velocity_pid, effort_pid;
  double position_pwm, velocity_pwm, effort_pwm;
  double intermediate_effort;
  double pwm, force, alpha;

  //Combine forces using selected strategy
  auto control_strategy = hebiros_joint->getControlStrategy();
  using CS = hebi::sim::Joint::ControlStrategy;
  switch (control_strategy) {
    case CS::Off:
      pwm = 0;
      break;

    case CS::DirectPWM:
      pwm = Clip(target_effort, MIN_PWM, MAX_PWM);
      break;

    case CS::Strategy2:
      position_pid =
        hebiros_joint->position_pid.update(target_position, position, dt);
      velocity_pid =
        hebiros_joint->velocity_pid.update(target_velocity, velocity, dt);
      intermediate_effort = target_effort + position_pid + velocity_pid;
      effort_pwm = Clip(
        hebiros_joint->effort_pid.update(intermediate_effort, effort, dt),
        MIN_PWM, MAX_PWM);
      pwm = effort_pwm;
      break;

    case CS::Strategy3:
      position_pwm = Clip(
        hebiros_joint->position_pid.update(target_position, position, dt),
        MIN_PWM, MAX_PWM);
      velocity_pwm = Clip(
        hebiros_joint->velocity_pid.update(target_velocity, velocity, dt),
        MIN_PWM, MAX_PWM);
      effort_pwm = Clip(
        hebiros_joint->effort_pid.update(target_effort, effort, dt),
        MIN_PWM, MAX_PWM);
      pwm = Clip(position_pwm + velocity_pwm + effort_pwm, MIN_PWM, MAX_PWM);
      break;

    case CS::Strategy4:
      position_pid = hebiros_joint->position_pid.update(target_position, position, dt);
      intermediate_effort = target_effort + position_pid;
      effort_pwm = Clip(
        hebiros_joint->effort_pid.update(intermediate_effort, effort, dt),
        MIN_PWM, MAX_PWM);
      velocity_pwm = Clip(
        hebiros_joint->velocity_pid.update(target_velocity, velocity, dt),
        MIN_PWM, MAX_PWM);
      pwm = Clip(velocity_pwm + effort_pwm, MIN_PWM, MAX_PWM);
      break;

    default:
      pwm = 0;
  }

  double gear_ratio = hebiros_joint->gear_ratio;

  float voltage = 48.0f;
  float motor_velocity = velocity * gear_ratio;
  float speed_constant = 1530.0f;
  float term_resist = 9.99f;
  if (hebiros_joint->isX8()) {
    speed_constant = 1360.0f;
    term_resist = 3.19f;
  }

  pwm = hebiros_joint->temperature_safety.limit(pwm);

  if (pwm == 0) {
    force = 0;
  }
  else {
    // TODO: use temp compensation here, too?
    force = ((pwm*voltage - (motor_velocity/speed_constant)) / term_resist) * 0.00626 * gear_ratio * 0.65;
  }

  float prev_winding_temp = hebiros_joint->temperature.getMotorWindingTemperature();

  // Get components of power into the motor
  
  // Temperature compensated speed constant
  float comp_speed_constant = speed_constant * 1.05f * // Experimental tuning factor                           
    (1.f + .001f * (prev_winding_temp - 20.f)); // .001 is speed constant change per temperature change 
  float winding_resistance = term_resist * 
    (1.f + .004f * (prev_winding_temp - 20.f)); // .004 is resistance change per temperature change for copper 
  float back_emf = (motor_velocity * 30.f / M_PI) / comp_speed_constant;
  float winding_voltage = pwm * voltage - back_emf;

  // TODO: could add ripple current estimate here, too

  // Update temperature:
  // Power = I^2R, but I = V/R so I^2R = V^2/R:
  double power_in = winding_voltage * winding_voltage / winding_resistance;
  hebiros_joint->temperature.update(power_in, dt);
  hebiros_joint->temperature_safety.update(hebiros_joint->temperature.getMotorWindingTemperature());

  //alpha = hebiros_joint->low_pass_alpha;
  //force = (force * alpha) + hebiros_joint->prev_force * (1 - alpha);
  //hebiros_joint->prev_force = force;

  return force;
}

//Limit x to a value from low to high
double HebirosGazeboController::Clip(double x, double low, double high) {
  return std::min(std::max(x, low), high);
}

