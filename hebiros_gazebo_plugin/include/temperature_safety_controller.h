#pragma once

namespace hebiros {
namespace sim {

// A simple multi-stage actuator temperature model.
class TemperatureSafetyController {
public:
  TemperatureSafetyController(double max_temp);

  void update(double measured_temp);
  double limit(double raw_value);

private:
  double max_temp_;
  double lambda_{1};
  double max_pwm_{1};

};

}
}
