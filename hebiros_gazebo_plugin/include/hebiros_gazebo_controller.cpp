
#include <hebiros_gazebo_controller.h>


HebirosGazeboController::HebirosGazeboController() {}

HebirosGazeboController::~HebirosGazeboController() {}

void HebirosGazeboController::SetSettings(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint) {

  CommandMsg target = hebiros_joint->command_target;
  int i = hebiros_joint->command_index;

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

  //Set position gains
  if (i < target.settings.position_gains.kp.size()) {
    hebiros_joint->settings.position_gains.kp = {target.settings.position_gains.kp[i]};
  }
  else {
    hebiros_joint->settings.position_gains.kp = {DEFAULT_POSITION_KP};
  }
  if (i < target.settings.position_gains.ki.size()) {
    hebiros_joint->settings.position_gains.ki = {target.settings.position_gains.ki[i]};
  }
  else {
    hebiros_joint->settings.position_gains.ki = {DEFAULT_POSITION_KI};
  }
  if (i < target.settings.position_gains.kd.size()) {
    hebiros_joint->settings.position_gains.kd = {target.settings.position_gains.kd[i]};
  }
  else {
    hebiros_joint->settings.position_gains.kd = {DEFAULT_POSITION_KD};
  }

  //Set velocity gains
  if (i < target.settings.velocity_gains.kp.size()) {
    hebiros_joint->settings.velocity_gains.kp = {target.settings.velocity_gains.kp[i]};
  }
  else {
    hebiros_joint->settings.velocity_gains.kp = {DEFAULT_VELOCITY_KP};
  }
  if (i < target.settings.velocity_gains.ki.size()) {
    hebiros_joint->settings.velocity_gains.ki = {target.settings.velocity_gains.ki[i]};
  }
  else {
    hebiros_joint->settings.velocity_gains.ki = {DEFAULT_VELOCITY_KI};
  }
  if (i < target.settings.velocity_gains.kd.size()) {
    hebiros_joint->settings.velocity_gains.kd = {target.settings.velocity_gains.kd[i]};
  }
  else {
    hebiros_joint->settings.velocity_gains.kd = {DEFAULT_VELOCITY_KD};
  }

  //Set effort gains
  if (i < target.settings.effort_gains.kp.size()) {
    hebiros_joint->settings.effort_gains.kp = {target.settings.effort_gains.kp[i]};
  }
  else {
    hebiros_joint->settings.effort_gains.kp = {DEFAULT_EFFORT_KP};
  }
  if (i < target.settings.effort_gains.ki.size()) {
    hebiros_joint->settings.effort_gains.ki = {target.settings.effort_gains.ki[i]};
  }
  else {
    hebiros_joint->settings.effort_gains.ki = {DEFAULT_EFFORT_KI};
  }
  if (i < target.settings.effort_gains.kd.size()) {
    hebiros_joint->settings.effort_gains.kd = {target.settings.effort_gains.kd[i]};
  }
  else {
    hebiros_joint->settings.effort_gains.kd = {DEFAULT_EFFORT_KD};
  }
}

double HebirosGazeboController::ComputeForce(
  std::shared_ptr<HebirosGazeboJoint> hebiros_joint,
  double position, double velocity, double effort) {

  ros::Time current_time = ros::Time::now();
  ros::Duration elapsed_time = current_time - hebiros_joint->start_time;
  ros::Duration iteration_time = current_time - hebiros_joint->prev_time;
  hebiros_joint->prev_time = current_time;

  CommandMsg target = hebiros_joint->command_target;
  int i = hebiros_joint->command_index;

  double target_position, target_velocity, target_effort;
  double position_error_p, position_error_i, position_error_d;
  double velocity_error_p, velocity_error_i, velocity_error_d;
  double effort_error_p, effort_error_i, effort_error_d;
  double position_force, velocity_force, effort_force;
  double gear_ratio, pwm, force;

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

  //Compute pid errors
  position_error_p = target_position - position;
  position_error_i = hebiros_joint->position_elapsed_error + position_error_p;
  position_error_d = (position_error_p - hebiros_joint->position_prev_error) /
    iteration_time.toSec();
  hebiros_joint->position_prev_error = position_error_p;
  hebiros_joint->position_elapsed_error = position_error_i;

  velocity_error_p = target_velocity - velocity;
  velocity_error_i = hebiros_joint->velocity_elapsed_error + velocity_error_p;
  velocity_error_d = (velocity_error_p - hebiros_joint->velocity_prev_error) /
    iteration_time.toSec();
  hebiros_joint->velocity_prev_error = velocity_error_p;
  hebiros_joint->velocity_elapsed_error = velocity_error_i;

  effort_error_p = target_effort - effort;
  effort_error_i = hebiros_joint->effort_elapsed_error + effort_error_p;
  effort_error_d = (effort_error_p - hebiros_joint->effort_prev_error) /
    iteration_time.toSec();
  hebiros_joint->effort_prev_error = effort_error_p;
  hebiros_joint->effort_elapsed_error = effort_error_i;

  if (iteration_time.toSec() <= 0) {
    position_error_d = 0;
    velocity_error_d = 0;
    effort_error_d = 0;
  }

  //Compute force components
  position_force = (hebiros_joint->settings.position_gains.kp[0] * position_error_p) +
    (hebiros_joint->settings.position_gains.ki[0] * position_error_i) +
    (hebiros_joint->settings.position_gains.kd[0] * position_error_d);

  velocity_force = (hebiros_joint->settings.velocity_gains.kp[0] * velocity_error_p) +
    (hebiros_joint->settings.velocity_gains.ki[0] * velocity_error_i) +
    (hebiros_joint->settings.velocity_gains.kd[0] * velocity_error_d);

  effort_force = (hebiros_joint->settings.effort_gains.kp[0] * effort_error_p) +
    (hebiros_joint->settings.effort_gains.ki[0] * effort_error_i) +
    (hebiros_joint->settings.effort_gains.kd[0] * effort_error_d);

  //Combine forces using selected strategy
  int control_strategy = hebiros_joint->settings.control_strategy[0];
  switch (control_strategy) {
    case 0:
      pwm = 0;
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      position_force = Clip(position_force, MIN_PWM, MAX_PWM);
      velocity_force = Clip(velocity_force, MIN_PWM, MAX_PWM);
      effort_force = Clip(effort_force, MIN_PWM, MAX_PWM);
      pwm = Clip(position_force + velocity_force + effort_force, MIN_PWM, MAX_PWM);
      break;
    case 4:
      break;
    default:
      pwm = 0;
  }

  gear_ratio = hebiros_joint->gear_ratio;

  force = ((pwm*48.0 - ((velocity*gear_ratio)/1530)) / 9.99) * 0.00626 * gear_ratio;

  return force;
}

double HebirosGazeboController::Clip(double x, double low, double high) {
  return std::min(std::max(x, low), high);
}

