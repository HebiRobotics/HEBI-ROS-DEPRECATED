#pragma once

#include "hebi.h"
#include "Eigen/Eigen"
#include "util.hpp"
#include <vector>
#include <memory>

using namespace Eigen;

namespace hebi {
namespace trajectory {

/**
 * \brief Represents a smooth trajectory through a set of waypoints.
 */
class Trajectory final
{
  private:
    /**
     * C-style trajectory objects (one for each module)
     */
    std::vector<HebiTrajectoryPtr> trajectories_;

    /**
     * The number of modules controlled by this trajectory.
     */
    const int number_of_joints_;

    /**
     * The number of waypoints in this trajectory.
     */
    const int number_of_waypoints_;

    /**
     * The time at which the trajectory starts (seconds).
     */
    const double start_time_;

    /**
     * The time at which the trajectory ends (seconds).
     */
    const double end_time_;
 
    /**
     * Creates a Trajectory from a list of the underlying C-style objects.
     */
    Trajectory(std::vector<HebiTrajectoryPtr> trajectories, int number_of_waypoints, double start_time, double end_time);

  public:

    /**
     * \brief Creates a smooth trajectory through a set of waypoints (position
     * velocity and accelerations defined at particular times). This trajectory
     * wrapper object can create multi-dimensional trajectories (i.e., multiple
     * joints moving together using the same time reference).
     *
     * \param time_vector A vector of desired times at which to reach each
     * waypoint; this must be defined (and not NAN for any element).
     * \param positions A matrix of waypoint joint positions (in SI units). The
     * number of rows should be equal to the number of joints, and the number of
     * columns equal to the number of waypoints.  Any elements that are NAN will
     * be considered free parameters when solving for a trajectory. Values of
     * +/-infinity are not allowed.
     * \param velocities An optional matrix of velocity constraints at the
     * corresponding waypoints; should either be nullptr or matching the size of
     * the positions matrix. Any elements that are NAN will be considered free
     * parameters when solving for a trajectory. Values of +/-infinity are not
     * allowed.
     * \param accelerations An optional matrix of acceleration constraints at
     * the corresponding waypoints; should either be nullptr or matching the
     * size of the positions matrix. Any elements that are NAN will be
     * considered free parameters when solving for a trajectory. Values of
     * +/-infinity are not allowed.
     *
     * \returns A HebiTrajectory object if there were no errors, and the
     * trajectory has been created. An empty shared_ptr indicates that there was
     * an error in creating the trajectory.
     */
    static std::shared_ptr<Trajectory> createUnconstrainedQp(
      const VectorXd& time_vector,
      const MatrixXd& positions,
      const MatrixXd* velocities = nullptr,
      const MatrixXd* accelerations = nullptr);

    /**
     * \brief Destructor cleans up resources for trajectory.
     */
    virtual ~Trajectory() noexcept;

    /**
     * \brief The number of independent position trajectories over the same time
     * domain that are managed by this object.
     */
    int getJointCount() { return number_of_joints_; }

    /**
     * \brief The number of fixed waypoints that each joint is moving through.
     */
    int getWaypointCount() { return number_of_waypoints_; }

    /**
     * \brief Get the time (in seconds) at which the defined trajectory begins.
     */
    double getStartTime() { return start_time_; }

    /**
     * \brief Get the time (in seconds) at which the defined trajectory ends.
     */
    double getEndTime() { return end_time_; }

    /**
     * \brief The time (in seconds) between the start and end of this
     * trajectory.
     */
    double getDuration();

    /**
     * \brief Returns the position, velocity, and acceleration for a given
     * point in time along the trajectory.
     *
     * \param time The time for which the trajectory state is being queried.
     * This should be between the start and end of the trajectory.
     * \param position If not nullptr, this vector is filled in with the
     * position along the trajectory for each joint at the given time.
     * \param velocity If not nullptr, this vector is filled in with the
     * velocity along the trajectory for each joint at the given time.
     * \param acceleration If not nullptr, this vector is filled in with the
     * acceleration along the trajectory for each joint at the given time.
     */
    bool getState(double time, VectorXd* position, VectorXd* velocity, VectorXd* acceleration);

  private:
    /**
     * Disable copy and move constructors and assignment operators
     */
    HEBI_DISABLE_COPY_MOVE(Trajectory)
};

} // namespace trajectory
} // namespace hebi
