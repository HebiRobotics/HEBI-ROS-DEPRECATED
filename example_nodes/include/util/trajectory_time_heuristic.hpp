#pragma once

#include "Eigen/Dense"

namespace hebi {
namespace util {

class TrajectoryTimeHeuristic
{
public:
  /**
   * Get suggested times to reach each waypoint through a simple heuristic.
   */
  static Eigen::VectorXd getTimes(
    const Eigen::MatrixXd& positions,
    const Eigen::MatrixXd& velocities,
    const Eigen::MatrixXd& accelerations)
  {
    Eigen::VectorXd times(positions.cols());
    // TODO: add better heuristic, using joint velocity limits for each module and
    // amount of rotation.
    for (int i = 0; i < positions.cols(); ++i)
      times(i) = i * 2;
    return times;
  }
private:
  TrajectoryTimeHeuristic() = delete;
};

} // namespace util
} // namespace hebi

