#include "hebiros/PidGainsMsg.h"
#include <cstdint>

namespace hebiros {
namespace sim {

// A simple PID controller class
class PidController {

public:
  explicit PidController(double ff_scale)
    : ff_scale_(ff_scale)
  {}

  double update(double target, double feedback, double dt, const PidGainsMsg& pid_gains, size_t gain_idx);

private:
  double elapsed_error_{};
  double prev_error_{};
  // TODO: scale here or when setting value?
  double ff_scale_{};
};

} // namespace simulation
} // namespace hebiros
