#include <hebiros_gazebo_controller.h>
#include "pid_controller.h"

namespace controller {

static constexpr double MAX_PWM = 1.0;
static constexpr double MIN_PWM = -1.0;

}

using namespace controller;

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

