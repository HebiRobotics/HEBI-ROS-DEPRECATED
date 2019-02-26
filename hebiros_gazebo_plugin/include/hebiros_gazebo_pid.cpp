#include "hebiros_gazebo_pid.h"
#include "hebiros/PidGainsMsg.h"

namespace hebiros {
namespace sim {

// Update and return the new output command
double PidController::update(double target, double feedback, double dt, const PidGainsMsg& pid_gains, size_t gain_idx)
{
  double error_p, error_i, error_d;
  error_p = target - feedback;
  error_i = elapsed_error + error_p;
  error_d = (error_p - prev_error) / dt;
  prev_error = error_p;
  elapsed_error = error_i;

  if (dt <= 0)
    error_d = 0;

  // TODO: store gains instead of looking them up
  // here...
  return
    pid_gains.kp[i] * error_p + 
    pid_gains.ki[i] * error_i +
    pid_gains.ki[i] * error_d;
}

} // namespace simulation
} // namespace hebiros
