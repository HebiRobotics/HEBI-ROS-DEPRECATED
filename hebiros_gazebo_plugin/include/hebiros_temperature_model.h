#pragma once

namespace hebiros {
namespace sim {

// A simple multi-stage actuator temperature model.
class TemperatureModel {
public:
  static TemperatureModel createX5();
  static TemperatureModel createX8();

  void update(double power_in, double dt);

  // Winding temperature
  double getMotorWindingTemperature() { return t_w_; }
  double getMotorHousingTemperature() { return t_h_; }
  double getActuatorBodyTemperature() { return t_b_; }

private:
  TemperatureModel(
    double r_wh, double r_hb, double r_be,
    double c_w, double c_h, double c_b);

  // Thermal resistances: winding-housing, housing-body, body-environment
  const double r_wh_;
  const double r_hb_;
  const double r_be_;

  // Thermal capacitances: winding, housing, body
  const double c_w_;
  const double c_h_;
  const double c_b_;

  // State variables - these are the bodies for which we
  // model temperature
  double t_w_{34.0}; // Motor winding
  double t_h_{34.0}; // Motor housing
  double t_b_{32.0}; // Actuator body

  // Environment temperature (in *C) ; assume room temperature + fudge factor
  static constexpr double t_e_{32.0};
};

}
}
