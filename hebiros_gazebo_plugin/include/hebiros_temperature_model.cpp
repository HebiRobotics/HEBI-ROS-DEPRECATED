#include "hebiros_temperature_model.h"

namespace hebiros {
namespace sim {

TemperatureModel TemperatureModel::createX5() {
  double r_be = 8.0; // (found experimentally from steady-state experiments)
  return TemperatureModel(
    2.57 * 1.8,     // r_wh (datasheet)
    1.0,            // r_hb (Open air from data sheet divided by empirical factor)
    r_be,              
    0.943 / 2.57,   // c_w (winding, datasheet; tc_winding / rth_winding)
    390.0 / 23.5,   // c_h (housing, datasheet) (TC / R)
    10000.0 / r_be  // c_b actuator body; from experimentally measured time constant
  );
}

TemperatureModel TemperatureModel::createX8() {
  double r_be = 4.0; // (found experimentally from steady-state experiments)
  return TemperatureModel(
    1.41 * 1.6,     // r_wh (datasheet)
    1.0,            // r_hb (Open air from data sheet divided by empirical factor)
    r_be,              
    0.9 / 1.41,     // c_w (winding, datasheet; tc_winding / rth_winding)
    427.0 / 17.7,   // c_h (housing, datasheet) (TC / R)
    10000.0 / r_be  // c_b actuator body; from experimentally measured time constant
  );
}

// Perform an update of all of the stages of the model,
// based on the estimated power going into the system
void TemperatureModel::update(double power_in, double dt) {
  // The heat flux between each stage; we calculate
  // these all _before_ updating the temperatures
  double q_wh = (t_w_ - t_h_) / r_wh_;
  double q_hb = (t_h_ - t_b_) / r_hb_;
  double q_be = (t_b_ - t_e_) / r_be_;
  
  // Update the temperature of each phase
  t_w_ += (power_in - q_wh) * dt / c_w_;
  t_h_ += (q_wh - q_hb) * dt / c_h_;
  t_b_ += (q_hb - q_be) * dt / c_b_;
}

TemperatureModel::TemperatureModel(
  double r_wh, double r_hb, double r_be,
  double c_w, double c_h, double c_b)
  : r_wh_(r_wh), r_hb_(r_hb), r_be_(r_be), 
    c_w_(c_w), c_h_(c_h), c_b_(c_b) {
}

}
}
