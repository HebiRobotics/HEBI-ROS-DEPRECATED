#include "hebiros_gazebo_pid.h"

namespace hebiros {
namespace sim {

// Update and return the new output command
double PidController::update(double target, double feedback, double dt, const PidGainsMsg& pid_gains, size_t gain_idx)
{
  // "Disable" the controller if commands are nan
  if (std::isnan(target)) {
    return 0;
  }
  double error_p, error_i, error_d;
  error_p = target - feedback;
  error_i = _elapsed_error + error_p;
  error_d = (error_p - _prev_error) / dt;
  _prev_error = error_p;
  _elapsed_error = error_i;

  if (dt <= 0)
    error_d = 0;

  // TODO: store gains instead of looking them up
  // here...
  return
    pid_gains.kp[gain_idx] * error_p +
    pid_gains.ki[gain_idx] * error_i +
    pid_gains.kd[gain_idx] * error_d +
    pid_gains.feed_forward[gain_idx] * _ff_scale * target;
}

} // namespace simulation
} // namespace hebiros
