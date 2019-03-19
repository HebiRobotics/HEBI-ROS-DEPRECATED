#include <hebiros_gazebo_controller.h>
#include "pid_controller.h"

namespace controller {

enum class control_strategies {
  CONTROL_STRATEGY_OFF = 0,
  CONTROL_STRATEGY_DIRECT_PWM = 1,
  CONTROL_STRATEGY_2 = 2,
  CONTROL_STRATEGY_3 = 3,
  CONTROL_STRATEGY_4 = 4
};

static constexpr control_strategies DEFAULT_CONTROL_STRATEGY = control_strategies::CONTROL_STRATEGY_3;

static constexpr double MAX_PWM = 1.0;
static constexpr double MIN_PWM = -1.0;

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

}

using namespace controller;


//Set defaults settings for a joint once
void HebirosGazeboController::SetSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  hebi::sim::Joint* hebiros_joint) {

  hebiros_joint->low_pass_alpha = LOW_PASS_ALPHA;
  int i = hebiros_joint->command_index;

  //Set gear ratio
  // TODO: check previous call to SetSettings -- potentially eliminate?

  //Set control strategy
  hebiros_joint->setControlStrategy(static_cast<uint8_t>(DEFAULT_CONTROL_STRATEGY));

  SetDefaultGains(hebiros_group, hebiros_joint);
}

//Initialize gains with default values based on model and control strategy
void HebirosGazeboController::SetDefaultGains(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  hebi::sim::Joint* hebiros_joint) {
  
  std::string model_name = hebiros_joint->model_name;
  int i = hebiros_joint->command_index;
  auto control_strategy = hebiros_joint->getControlStrategy();

  // TODO: move all this logic into joint-specific code, and potentially load files from .xml
  // files

  // Each of these PID gains initializations is { kp, ki, kd, feed forward }
  if (model_name == "X5_1" && control_strategy == 2) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_1" && control_strategy == 3) {
    hebiros_joint->position_pid.setGains({ 0.5f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_1" && control_strategy == 4) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == 2) {
    hebiros_joint->position_pid.setGains({ 10.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.2f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == 3) {
    hebiros_joint->position_pid.setGains({ 1.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_4" && control_strategy == 4) {
    hebiros_joint->position_pid.setGains({ 10.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == 2) {
    hebiros_joint->position_pid.setGains({ 15.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.5f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == 3) {
    hebiros_joint->position_pid.setGains({ 1.5f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X5_9" && control_strategy == 4) {
    hebiros_joint->position_pid.setGains({ 15.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.05f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.25f, 0.f, 0.001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == 2) {
    hebiros_joint->position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == 3) {
    hebiros_joint->position_pid.setGains({ 1.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_3" && control_strategy == 4) {
    hebiros_joint->position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == 2) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == 3) {
    hebiros_joint->position_pid.setGains({ 2.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_9" && control_strategy == 4) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == 2) {
    hebiros_joint->position_pid.setGains({ 5.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.1f, 0.f, 0.f, 0.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == 3) {
    hebiros_joint->position_pid.setGains({ 3.f, 0.f, 0.f, 0.f });
    hebiros_joint->velocity_pid.setGains({ 0.03f, 0.f, 0.f, 1.f });
    hebiros_joint->effort_pid.setGains({ 0.1f, 0.f, 0.0001f, 1.f });
  } else if (model_name == "X8_16" && control_strategy == 4) {
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

// Update the gains for the fields which have information; leave the others unchanged.
// \return `true` if gains changed, `false` otherwise.
bool updateGains(hebi::sim::PidGains& gains, const hebiros::PidGainsMsg& msg, size_t i) {
  bool changed = false;
  if (i < msg.kp.size()) {
    gains.kp_ = msg.kp[i];
    changed = true;
  }
  if (i < msg.ki.size()) {
    gains.ki_ = msg.ki[i];
    changed = true;
  }
  if (i < msg.kd.size()) {
    gains.kd_ = msg.kd[i];
    changed = true;
  }
  if (i < msg.feed_forward.size()) {
    gains.feed_forward_ = msg.feed_forward[i];
    changed = true;
  }
  return changed;
}

//Change settings for a joint if specifically commanded
void HebirosGazeboController::ChangeSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  hebi::sim::Joint* hebiros_joint) {

  CommandMsg target = hebiros_group->command_target;
  int i = hebiros_joint->command_index;

  //Set name
  if (i < target.settings.name.size()) {
    hebiros_joint->name = target.settings.name[i];
  }

  //Change control strategy
  if (i < target.settings.control_strategy.size()) {
    hebiros_joint->setControlStrategy(target.settings.control_strategy[i]);
  }

  //Change gains:
  auto current_pos_gains = hebiros_joint->position_pid.getGains();
  if (updateGains(current_pos_gains, target.settings.position_gains, i))
    hebiros_joint->position_pid.setGains(current_pos_gains);

  auto current_vel_gains = hebiros_joint->velocity_pid.getGains();
  if (updateGains(current_vel_gains, target.settings.velocity_gains, i))
    hebiros_joint->velocity_pid.setGains(current_vel_gains);

  auto current_eff_gains = hebiros_joint->effort_pid.getGains();
  if (updateGains(current_eff_gains, target.settings.effort_gains, i))
    hebiros_joint->effort_pid.setGains(current_eff_gains);
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
double HebirosGazeboController::ComputeForce(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  hebi::sim::Joint* hebiros_joint,
  double position, double velocity, double effort, const ros::Duration& iteration_time) {

  auto dt = iteration_time.toSec();

  CommandMsg target = hebiros_group->command_target;
  int i = hebiros_joint->command_index;

  double target_position = std::numeric_limits<float>::quiet_NaN();
  double target_velocity = std::numeric_limits<float>::quiet_NaN();
  double target_effort = std::numeric_limits<float>::quiet_NaN();
  double position_pid, velocity_pid, effort_pid;
  double position_pwm, velocity_pwm, effort_pwm;
  double intermediate_effort;
  double pwm, force, alpha;

  //Set target positions
  if (i < target.position.size()) {
    target_position = target.position[i];
  }
  if (i < target.velocity.size()) {
    target_velocity = target.velocity[i];
  }
  if (i < target.effort.size()) {
    target_effort = target.effort[i];
  }

  //Combine forces using selected strategy
  int control_strategy = hebiros_joint->getControlStrategy();

  switch (control_strategy) {
    case 0:
      pwm = 0;
      break;

    case 1:
      pwm = Clip(target_effort, MIN_PWM, MAX_PWM);
      break;

    case 2:
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

    case 3:
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

    case 4:
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

