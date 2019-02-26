
namespace hebiros {

class PidGainsMsg;

namespace sim {

// A simple PID controller class
class PidController {

public:
  PidController() = default;

  double update(double target, double feedback, double dt, const PidGainsMsg& pid_gains, size_t gain_idx);

private:
  double elapsed_error{};
  double prev_error{};
};

} // namespace simulation
} // namespace hebiros
