
#include <hebiros_gazebo_controller.h>


HebirosGazeboController::HebirosGazeboController() {}

HebirosGazeboController::~HebirosGazeboController() {}

//Set defaults settings for a joint once
void HebirosGazeboController::SetSettings(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {

  CommandMsg target = hebiros_joint->command_target;
  int i = hebiros_joint->command_index;

  hebiros_joint->low_pass_alpha = LOW_PASS_ALPHA;

  //Set gear ratio
  if (gear_ratios.find(hebiros_joint->model_name) != gear_ratios.end()) {
    hebiros_joint->gear_ratio = gear_ratios[hebiros_joint->model_name];
  }
  else {
    hebiros_joint->gear_ratio = DEFAULT_GEAR_RATIO;
  }

  //Set control strategy
  if (i < target.settings.control_strategy.size()) {
    hebiros_joint->settings.control_strategy = {target.settings.control_strategy[i]};
  }
  else {
    hebiros_joint->settings.control_strategy = {
      static_cast<char>(DEFAULT_CONTROL_STRATEGY)};
  }

  SetDefaultGains(hebiros_joint);
}

//Initialize gains with default values based on model and control strategy
void HebirosGazeboController::SetDefaultGains(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {
  
  std::string model_name = hebiros_joint->model_name;
  int control_strategy = hebiros_joint->settings.control_strategy[0];

  if (model_name == "X5_1" && control_strategy == 2) {
    hebiros_joint->settings.position_gains.kp = {5};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.1};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_1" && control_strategy == 3) {
    hebiros_joint->settings.position_gains.kp = {0.5};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.05};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_1" && control_strategy == 4) {
    hebiros_joint->settings.position_gains.kp = {5};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.05};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_4" && control_strategy == 2) {
    hebiros_joint->settings.position_gains.kp = {10};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.2};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_4" && control_strategy == 3) {
    hebiros_joint->settings.position_gains.kp = {1};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.05};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_4" && control_strategy == 4) {
    hebiros_joint->settings.position_gains.kp = {10};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.05};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_9" && control_strategy == 2) {
    hebiros_joint->settings.position_gains.kp = {15};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.5};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_9" && control_strategy == 3) {
    hebiros_joint->settings.position_gains.kp = {1.5};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.05};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else if (model_name == "X5_9" && control_strategy == 4) {
    hebiros_joint->settings.position_gains.kp = {15};
    hebiros_joint->settings.position_gains.ki = {0};
    hebiros_joint->settings.position_gains.kd = {0};
    hebiros_joint->settings.velocity_gains.kp = {0.05};
    hebiros_joint->settings.velocity_gains.ki = {0};
    hebiros_joint->settings.velocity_gains.kd = {0};
    hebiros_joint->settings.effort_gains.kp = {0.25};
    hebiros_joint->settings.effort_gains.ki = {0};
    hebiros_joint->settings.effort_gains.kd = {0.001};
  }
  else {
    hebiros_joint->settings.position_gains.kp = {DEFAULT_POSITION_KP};
    hebiros_joint->settings.position_gains.ki = {DEFAULT_POSITION_KI};
    hebiros_joint->settings.position_gains.kd = {DEFAULT_POSITION_KD};
    hebiros_joint->settings.velocity_gains.kp = {DEFAULT_VELOCITY_KP};
    hebiros_joint->settings.velocity_gains.ki = {DEFAULT_VELOCITY_KI};
    hebiros_joint->settings.velocity_gains.kd = {DEFAULT_VELOCITY_KD};
    hebiros_joint->settings.effort_gains.kp = {DEFAULT_EFFORT_KP};
    hebiros_joint->settings.effort_gains.ki = {DEFAULT_EFFORT_KI};
    hebiros_joint->settings.effort_gains.kd = {DEFAULT_EFFORT_KD};
  }
}

//Change settings for a joint if specifically commanded
void HebirosGazeboController::ChangeSettings(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {

  CommandMsg target = hebiros_joint->command_target;
  int i = hebiros_joint->command_index;

  //Change control strategy
  if (i < target.settings.control_strategy.size()) {
    hebiros_joint->settings.control_strategy = {target.settings.control_strategy[i]};
  }

  //Change position gains
  if (i < target.settings.position_gains.kp.size()) {
    hebiros_joint->settings.position_gains.kp = {target.settings.position_gains.kp[i]};
  }
  if (i < target.settings.position_gains.ki.size()) {
    hebiros_joint->settings.position_gains.ki = {target.settings.position_gains.ki[i]};
  }
  if (i < target.settings.position_gains.kd.size()) {
    hebiros_joint->settings.position_gains.kd = {target.settings.position_gains.kd[i]};
  }

  //Change velocity gains
  if (i < target.settings.velocity_gains.kp.size()) {
    hebiros_joint->settings.velocity_gains.kp = {target.settings.velocity_gains.kp[i]};
  }
  if (i < target.settings.velocity_gains.ki.size()) {
    hebiros_joint->settings.velocity_gains.ki = {target.settings.velocity_gains.ki[i]};
  }
  if (i < target.settings.velocity_gains.kd.size()) {
    hebiros_joint->settings.velocity_gains.kd = {target.settings.velocity_gains.kd[i]};
  }

  //Change effort gains
  if (i < target.settings.effort_gains.kp.size()) {
    hebiros_joint->settings.effort_gains.kp = {target.settings.effort_gains.kp[i]};
  }
  if (i < target.settings.effort_gains.ki.size()) {
    hebiros_joint->settings.effort_gains.ki = {target.settings.effort_gains.ki[i]};
  }
  if (i < target.settings.effort_gains.kd.size()) {
    hebiros_joint->settings.effort_gains.kd = {target.settings.effort_gains.kd[i]};
  }
}

//Compute output force to the joint based on PID and control strategy
double HebirosGazeboController::ComputeForce(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double position, double velocity, double effort) {

  ros::Time current_time = ros::Time::now();
  ros::Duration iteration_time = current_time - hebiros_joint->prev_time;
  hebiros_joint->prev_time = current_time;

  CommandMsg target = hebiros_joint->command_target;
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
  int control_strategy = hebiros_joint->settings.control_strategy[0];
  switch (control_strategy) {
    case 0:
      pwm = 0;
      break;

    case 1:
      pwm = Clip(target_effort, MIN_PWM, MAX_PWM);
      break;

    case 2:
      position_pid =
        ComputePositionPID(hebiros_joint, target_position, position, iteration_time);
      velocity_pid =
        ComputeVelocityPID(hebiros_joint, target_velocity, velocity, iteration_time);
      intermediate_effort = target_effort + position_pid + velocity_pid;
      effort_pwm = Clip(
        ComputeEffortPID(hebiros_joint, intermediate_effort, effort, iteration_time),
        MIN_PWM, MAX_PWM);
      pwm = effort_pwm;
      break;

    case 3:
      position_pwm = Clip(
        ComputePositionPID(hebiros_joint, target_position, position, iteration_time),
        MIN_PWM, MAX_PWM);
      velocity_pwm = Clip(
        ComputeVelocityPID(hebiros_joint, target_velocity, velocity, iteration_time),
        MIN_PWM, MAX_PWM);
      effort_pwm = Clip(
        ComputeEffortPID(hebiros_joint, target_effort, effort, iteration_time),
        MIN_PWM, MAX_PWM);
      pwm = Clip(position_pwm + velocity_pwm + effort_pwm, MIN_PWM, MAX_PWM);
      break;

    case 4:
      position_pid =
        ComputePositionPID(hebiros_joint, target_position, position, iteration_time);
      intermediate_effort = target_effort + position_pid;
      effort_pwm = Clip(
        ComputeEffortPID(hebiros_joint, intermediate_effort, effort, iteration_time),
        MIN_PWM, MAX_PWM);
      velocity_pwm = Clip(
        ComputeVelocityPID(hebiros_joint, target_velocity, velocity, iteration_time),
        MIN_PWM, MAX_PWM);
      pwm = Clip(velocity_pwm + effort_pwm, MIN_PWM, MAX_PWM);
      break;

    default:
      pwm = 0;
  }

  gear_ratio = hebiros_joint->gear_ratio;

  if (pwm == 0) {
    force = 0;
  }
  else {
    force = ((pwm*48.0 - ((velocity*gear_ratio)/1530)) / 9.99) * 0.00626 * gear_ratio;
  }

  //alpha = hebiros_joint->low_pass_alpha;
  //force = (force * alpha) + hebiros_joint->prev_force * (1 - alpha);
  //hebiros_joint->prev_force = force;

  return force;
}

//Compute the PID error for positions
double HebirosGazeboController::ComputePositionPID(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double target_position, double position, ros::Duration iteration_time) {

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

  return (hebiros_joint->settings.position_gains.kp[0] * position_error_p) +
    (hebiros_joint->settings.position_gains.ki[0] * position_error_i) +
    (hebiros_joint->settings.position_gains.kd[0] * position_error_d);
}

//Compute the PID error for velocities
double HebirosGazeboController::ComputeVelocityPID(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double target_velocity, double velocity, ros::Duration iteration_time) {

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

  return (hebiros_joint->settings.velocity_gains.kp[0] * velocity_error_p) +
    (hebiros_joint->settings.velocity_gains.ki[0] * velocity_error_i) +
    (hebiros_joint->settings.velocity_gains.kd[0] * velocity_error_d);
}

//Compute the PID error for efforts
double HebirosGazeboController::ComputeEffortPID(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double target_effort, double effort, ros::Duration iteration_time) {

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

  return (hebiros_joint->settings.effort_gains.kp[0] * effort_error_p) +
    (hebiros_joint->settings.effort_gains.ki[0] * effort_error_i) +
    (hebiros_joint->settings.effort_gains.kd[0] * effort_error_d);
}

//Limit x to a value from low to high
double HebirosGazeboController::Clip(double x, double low, double high) {
  return std::min(std::max(x, low), high);
}

