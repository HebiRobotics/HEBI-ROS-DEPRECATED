#include "temperature_safety_controller.h"

namespace hebiros {
namespace sim {

TemperatureSafetyController::TemperatureSafetyController(double max_temp) 
  : max_temp_(max_temp) {
}

void TemperatureSafetyController::update(double measured_temp) {
  if (measured_temp >= max_temp_) {
    max_pwm_ = 0;
    return;
  }
  float d_val = max_temp_ - measured_temp;
  float d_pwm = lambda_ / d_val;
  if (d_pwm > 1.0)    
    max_pwm_ = 0;
  else
    max_pwm_ = 1.0 - d_pwm;
}

double TemperatureSafetyController::limit(double raw_value) {
  if (raw_value > max_pwm_)
    return max_pwm_;
  else if (raw_value < -max_pwm_)
    return -max_pwm_;
  return raw_value;
}

}
}
