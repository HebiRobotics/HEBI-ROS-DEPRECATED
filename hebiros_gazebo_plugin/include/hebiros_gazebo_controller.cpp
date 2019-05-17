#include <hebiros_gazebo_controller.h>
#include "pid_controller.h"

//Compute output force to the joint based on PID and control strategy
double HebirosGazeboController::ComputeForce(
  hebi::sim::Joint* hebiros_joint, double iteration_time) {

  auto dt = iteration_time;

  //Set target positions
  double force, alpha;

  double gear_ratio = hebiros_joint->gear_ratio;

  float voltage = 48.0f;
  float motor_velocity = hebiros_joint->velocity_fbk * gear_ratio;
  float speed_constant = 1530.0f;
  float term_resist = 9.99f;
  if (hebiros_joint->isX8()) {
    speed_constant = 1360.0f;
    term_resist = 3.19f;
  }

  if (hebiros_joint->pwm_cmd == 0) {
    force = 0;
  }
  else {
    // TODO: use temp compensation here, too?
    force = ((hebiros_joint->pwm_cmd*voltage - (motor_velocity/speed_constant)) / term_resist) * 0.00626 * gear_ratio * 0.65;
  }

  float prev_winding_temp = hebiros_joint->temperature.getMotorWindingTemperature();

  // Get components of power into the motor
  
  // Temperature compensated speed constant
  float comp_speed_constant = speed_constant * 1.05f * // Experimental tuning factor                           
    (1.f + .001f * (prev_winding_temp - 20.f)); // .001 is speed constant change per temperature change 
  float winding_resistance = term_resist * 
    (1.f + .004f * (prev_winding_temp - 20.f)); // .004 is resistance change per temperature change for copper 
  float back_emf = (motor_velocity * 30.f / M_PI) / comp_speed_constant;
  float winding_voltage = hebiros_joint->pwm_cmd * voltage - back_emf;

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
