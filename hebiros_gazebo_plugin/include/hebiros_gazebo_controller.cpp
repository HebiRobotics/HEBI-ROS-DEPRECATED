#include <hebiros_gazebo_controller.h>

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
static constexpr double DEFAULT_VELOCITY_KP = 0.05;
static constexpr double DEFAULT_VELOCITY_KI = 0.0;
static constexpr double DEFAULT_VELOCITY_KD = 0.0;
static constexpr double DEFAULT_EFFORT_KP = 0.25;
static constexpr double DEFAULT_EFFORT_KI = 0.0;
static constexpr double DEFAULT_EFFORT_KD = 0.001;

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
  {"X8_16", GEAR_RATIO_X8_16}};
}

using namespace controller;


//Set defaults settings for a joint once
void HebirosGazeboController::SetSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {

  hebiros_group->settings.name.push_back(hebiros_joint->name);
  hebiros_joint->low_pass_alpha = LOW_PASS_ALPHA;
  int i = hebiros_joint->command_index;

  //Set gear ratio
  if (gear_ratios.find(hebiros_joint->model_name) != gear_ratios.end()) {
    hebiros_joint->gear_ratio = gear_ratios[hebiros_joint->model_name];
  }
  else {
    hebiros_joint->gear_ratio = DEFAULT_GEAR_RATIO;
  }

  //Set control strategy
  hebiros_group->settings.control_strategy.push_back(
    static_cast<char>(DEFAULT_CONTROL_STRATEGY));

  SetDefaultGains(hebiros_group, hebiros_joint);
}

//Initialize gains with default values based on model and control strategy
void HebirosGazeboController::SetDefaultGains(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {
  
  std::string model_name = hebiros_joint->model_name;
  int i = hebiros_joint->command_index;
  int control_strategy = hebiros_group->settings.control_strategy[i];

  if (model_name == "X5_1" && control_strategy == 2) {
    hebiros_group->settings.position_gains.kp.push_back(5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.1);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_1" && control_strategy == 3) {
    hebiros_group->settings.position_gains.kp.push_back(0.5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.05);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_1" && control_strategy == 4) {
    hebiros_group->settings.position_gains.kp.push_back(5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.05);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_4" && control_strategy == 2) {
    hebiros_group->settings.position_gains.kp.push_back(10);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.2);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_4" && control_strategy == 3) {
    hebiros_group->settings.position_gains.kp.push_back(1);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.05);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_4" && control_strategy == 4) {
    hebiros_group->settings.position_gains.kp.push_back(10);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.05);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_9" && control_strategy == 2) {
    hebiros_group->settings.position_gains.kp.push_back(15);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.5);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_9" && control_strategy == 3) {
    hebiros_group->settings.position_gains.kp.push_back(1.5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.05);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  else if (model_name == "X5_9" && control_strategy == 4) {
    hebiros_group->settings.position_gains.kp.push_back(15);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.05);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.25);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.001);
  }
  if (model_name == "X8_3" && control_strategy == 2) {
    hebiros_group->settings.position_gains.kp.push_back(3);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.1);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_3" && control_strategy == 3) {
    hebiros_group->settings.position_gains.kp.push_back(1);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.03);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_3" && control_strategy == 4) {
    hebiros_group->settings.position_gains.kp.push_back(3);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.03);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_9" && control_strategy == 2) {
    hebiros_group->settings.position_gains.kp.push_back(5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.1);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_9" && control_strategy == 3) {
    hebiros_group->settings.position_gains.kp.push_back(2);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.03);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_9" && control_strategy == 4) {
    hebiros_group->settings.position_gains.kp.push_back(5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.03);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_16" && control_strategy == 2) {
    hebiros_group->settings.position_gains.kp.push_back(5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.1);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_16" && control_strategy == 3) {
    hebiros_group->settings.position_gains.kp.push_back(3);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.03);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else if (model_name == "X8_16" && control_strategy == 4) {
    hebiros_group->settings.position_gains.kp.push_back(5);
    hebiros_group->settings.position_gains.ki.push_back(0);
    hebiros_group->settings.position_gains.kd.push_back(0);
    hebiros_group->settings.velocity_gains.kp.push_back(0.03);
    hebiros_group->settings.velocity_gains.ki.push_back(0);
    hebiros_group->settings.velocity_gains.kd.push_back(0);
    hebiros_group->settings.effort_gains.kp.push_back(0.1);
    hebiros_group->settings.effort_gains.ki.push_back(0);
    hebiros_group->settings.effort_gains.kd.push_back(0.0001);
  }
  else {
    hebiros_group->settings.position_gains.kp.push_back(DEFAULT_POSITION_KP);
    hebiros_group->settings.position_gains.ki.push_back(DEFAULT_POSITION_KI);
    hebiros_group->settings.position_gains.kd.push_back(DEFAULT_POSITION_KD);
    hebiros_group->settings.velocity_gains.kp.push_back(DEFAULT_VELOCITY_KP);
    hebiros_group->settings.velocity_gains.ki.push_back(DEFAULT_VELOCITY_KI);
    hebiros_group->settings.velocity_gains.kd.push_back(DEFAULT_VELOCITY_KD);
    hebiros_group->settings.effort_gains.kp.push_back(DEFAULT_EFFORT_KP);
    hebiros_group->settings.effort_gains.ki.push_back(DEFAULT_EFFORT_KI);
    hebiros_group->settings.effort_gains.kd.push_back(DEFAULT_EFFORT_KD);
  }
}

//Change settings for a joint if specifically commanded
void HebirosGazeboController::ChangeSettings(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {

  CommandMsg target = hebiros_group->command_target;
  int i = hebiros_joint->command_index;

  //Set name
  if (i < target.settings.name.size()) {
    hebiros_group->settings.name[i] = target.settings.name[i];
  }

  //Change control strategy
  if (i < target.settings.control_strategy.size()) {
    hebiros_group->settings.control_strategy[i] = target.settings.control_strategy[i];
  }

  //Change position gains
  if (i < target.settings.position_gains.kp.size()) {
    hebiros_group->settings.position_gains.kp[i] = target.settings.position_gains.kp[i];
  }
  if (i < target.settings.position_gains.ki.size()) {
    hebiros_group->settings.position_gains.ki[i] = target.settings.position_gains.ki[i];
  }
  if (i < target.settings.position_gains.kd.size()) {
    hebiros_group->settings.position_gains.kd[i] = target.settings.position_gains.kd[i];
  }

  //Change velocity gains
  if (i < target.settings.velocity_gains.kp.size()) {
    hebiros_group->settings.velocity_gains.kp[i] = target.settings.velocity_gains.kp[i];
  }
  if (i < target.settings.velocity_gains.ki.size()) {
    hebiros_group->settings.velocity_gains.ki[i] = target.settings.velocity_gains.ki[i];
  }
  if (i < target.settings.velocity_gains.kd.size()) {
    hebiros_group->settings.velocity_gains.kd[i] = target.settings.velocity_gains.kd[i];
  }

  //Change effort gains
  if (i < target.settings.effort_gains.kp.size()) {
    hebiros_group->settings.effort_gains.kp[i] = target.settings.effort_gains.kp[i];
  }
  if (i < target.settings.effort_gains.ki.size()) {
    hebiros_group->settings.effort_gains.ki[i] = target.settings.effort_gains.ki[i];
  }
  if (i < target.settings.effort_gains.kd.size()) {
    hebiros_group->settings.effort_gains.kd[i] = target.settings.effort_gains.kd[i];
  }
}

//Compute output force to the joint based on PID and control strategy
double HebirosGazeboController::ComputeForce(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double position, double velocity, double effort, const ros::Duration& iteration_time) {

  CommandMsg target = hebiros_group->command_target;
  int i = hebiros_joint->command_index;

  double target_position, target_velocity, target_effort;
  double position_pid, velocity_pid, effort_pid;
  double position_pwm, velocity_pwm, effort_pwm;
  double intermediate_effort;
  double gear_ratio, pwm, force, alpha;

  //Set target positions
  if (i < target.position.size()) {
    target_position = target.position[i];
  }
  else {
    target_position = position;
  }
  if (i < target.velocity.size()) {
    target_velocity = target.velocity[i];
  }
  else {
    target_velocity = velocity;
  }
  if (i < target.effort.size()) {
    target_effort = target.effort[i];
  }
  else {
    target_effort = effort;
  }

  //Combine forces using selected strategy
  int control_strategy = hebiros_group->settings.control_strategy[i];
  switch (control_strategy) {
    case 0:
      pwm = 0;
      break;

    case 1:
      pwm = Clip(target_effort, MIN_PWM, MAX_PWM);
      break;

    case 2:
      position_pid =
        ComputePositionPID(hebiros_group, hebiros_joint, target_position, position, iteration_time);
      velocity_pid =
        ComputeVelocityPID(hebiros_group, hebiros_joint, target_velocity, velocity, iteration_time);
      intermediate_effort = target_effort + position_pid + velocity_pid;
      effort_pwm = Clip(
        ComputeEffortPID(hebiros_group, hebiros_joint, intermediate_effort, effort, iteration_time),
        MIN_PWM, MAX_PWM);
      pwm = effort_pwm;
      break;

    case 3:
      position_pwm = Clip(
        ComputePositionPID(hebiros_group, hebiros_joint, target_position, position, iteration_time),
        MIN_PWM, MAX_PWM);
      velocity_pwm = Clip(
        ComputeVelocityPID(hebiros_group, hebiros_joint, target_velocity, velocity, iteration_time),
        MIN_PWM, MAX_PWM);
      effort_pwm = Clip(
        ComputeEffortPID(hebiros_group, hebiros_joint, target_effort, effort, iteration_time),
        MIN_PWM, MAX_PWM);
      pwm = Clip(position_pwm + velocity_pwm + effort_pwm, MIN_PWM, MAX_PWM);
      break;

    case 4:
      position_pid =
        ComputePositionPID(hebiros_group, hebiros_joint, target_position, position, iteration_time);
      intermediate_effort = target_effort + position_pid;
      effort_pwm = Clip(
        ComputeEffortPID(hebiros_group, hebiros_joint, intermediate_effort, effort, iteration_time),
        MIN_PWM, MAX_PWM);
      velocity_pwm = Clip(
        ComputeVelocityPID(hebiros_group, hebiros_joint, target_velocity, velocity, iteration_time),
        MIN_PWM, MAX_PWM);
      pwm = Clip(velocity_pwm + effort_pwm, MIN_PWM, MAX_PWM);
      break;

    default:
      pwm = 0;
  }

  gear_ratio = hebiros_joint->gear_ratio;

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
  hebiros_joint->temperature.update(power_in, iteration_time.toSec());
  hebiros_joint->temperature_safety.update(hebiros_joint->temperature.getMotorWindingTemperature());

  //alpha = hebiros_joint->low_pass_alpha;
  //force = (force * alpha) + hebiros_joint->prev_force * (1 - alpha);
  //hebiros_joint->prev_force = force;

  return force;
}

//Compute the PID error for positions
double HebirosGazeboController::ComputePositionPID(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double target_position, double position, const ros::Duration& iteration_time) {

  double position_error_p, position_error_i, position_error_d;

  position_error_p = target_position - position;
  position_error_i = hebiros_joint->position_elapsed_error + position_error_p;
  position_error_d = (position_error_p - hebiros_joint->position_prev_error) /
    iteration_time.toSec();
  hebiros_joint->position_prev_error = position_error_p;
  hebiros_joint->position_elapsed_error = position_error_i;

  if (iteration_time.toSec() <= 0) {
    position_error_d = 0;
  }

  int i = hebiros_joint->command_index;

  return (hebiros_group->settings.position_gains.kp[i] * position_error_p) +
    (hebiros_group->settings.position_gains.ki[i] * position_error_i) +
    (hebiros_group->settings.position_gains.kd[i] * position_error_d);
}

//Compute the PID error for velocities
double HebirosGazeboController::ComputeVelocityPID(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double target_velocity, double velocity, const ros::Duration& iteration_time) {

  double velocity_error_p, velocity_error_i, velocity_error_d;

  velocity_error_p = target_velocity - velocity;
  velocity_error_i = hebiros_joint->velocity_elapsed_error + velocity_error_p;
  velocity_error_d = (velocity_error_p - hebiros_joint->velocity_prev_error) /
    iteration_time.toSec();
  hebiros_joint->velocity_prev_error = velocity_error_p;
  hebiros_joint->velocity_elapsed_error = velocity_error_i;

  if (iteration_time.toSec() <= 0) {
    velocity_error_d = 0;
  }

  int i = hebiros_joint->command_index;

  return (hebiros_group->settings.velocity_gains.kp[i] * velocity_error_p) +
    (hebiros_group->settings.velocity_gains.ki[i] * velocity_error_i) +
    (hebiros_group->settings.velocity_gains.kd[i] * velocity_error_d);
}

//Compute the PID error for efforts
double HebirosGazeboController::ComputeEffortPID(std::shared_ptr<HebirosGazeboGroup> hebiros_group,
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double target_effort, double effort, const ros::Duration& iteration_time) {

  double effort_error_p, effort_error_i, effort_error_d;

  effort_error_p = target_effort - effort;
  effort_error_i = hebiros_joint->effort_elapsed_error + effort_error_p;
  effort_error_d = (effort_error_p - hebiros_joint->effort_prev_error) /
    iteration_time.toSec();
  hebiros_joint->effort_prev_error = effort_error_p;
  hebiros_joint->effort_elapsed_error = effort_error_i;

  if (iteration_time.toSec() <= 0) {
    effort_error_d = 0;
  }

  int i = hebiros_joint->command_index;

  return (hebiros_group->settings.effort_gains.kp[i] * effort_error_p) +
    (hebiros_group->settings.effort_gains.ki[i] * effort_error_i) +
    (hebiros_group->settings.effort_gains.kd[i] * effort_error_d);
}

//Limit x to a value from low to high
double HebirosGazeboController::Clip(double x, double low, double high) {
  return std::min(std::max(x, low), high);
}

