#pragma once

#include <cstdint>

namespace hebi {
namespace sim {

// Pid gains structure for a single simulated module
struct PidGains {
  float kp_{};
  float ki_{};
  float kd_{};
  float feed_forward_{};
  // TODO: support remaining items in the future
};

// A simple PID controller class
class PidController {

public:
  explicit PidController(double ff_scale)
    : ff_scale_(ff_scale)
  {}

  void setGains(const PidGains& gains) { gains_ = gains; }
  PidGains getGains() const { return gains_; }

  double update(double target, double feedback, double dt);

private:
  double elapsed_error_{};
  double prev_error_{};
  // TODO: scale here or when setting value?
  double ff_scale_{};
  PidGains gains_{};
};

} // namespace simulation
} // namespace hebi
