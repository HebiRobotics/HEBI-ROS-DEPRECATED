#include "hebiros/PidGainsMsg.h"
#include <cstdint>

namespace hebiros {
namespace sim {

// A simple PID controller class
class PidController {

public:
  PidController(double ff_scale)
    : _ff_scale(ff_scale)
  {}

  double update(double target, double feedback, double dt, const PidGainsMsg& pid_gains, size_t gain_idx);

private:
  double _elapsed_error{};
  double _prev_error{};
  // TODO: scale here or when setting value?
  double _ff_scale{};
};

} // namespace simulation
} // namespace hebiros
