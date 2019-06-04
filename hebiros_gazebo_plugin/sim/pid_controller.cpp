#include "pid_controller.h"
#include <cmath>

namespace hebi {
namespace sim {

// Update and return the new output command
double PidController::update(double target, double feedback, double dt) {
  // "Disable" the controller if commands are nan
  if (std::isnan(target)) {
    return 0;
  }
  double error_p, error_i, error_d;
  error_p = target - feedback;
  error_i = elapsed_error_ + error_p;
  if (dt <= 0)
    error_d = 0;
  else
    error_d = (error_p - prev_error_) / dt;
  prev_error_ = error_p;
  elapsed_error_ = error_i;

  return
    gains_.kp_ * error_p +
    gains_.ki_ * error_i +
    gains_.kd_ * error_d +
    gains_.feed_forward_ * ff_scale_ * target;
}

} // namespace simulation
} // namespace hebi
