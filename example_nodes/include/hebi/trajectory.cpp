#include "trajectory.hpp"

namespace hebi {
namespace trajectory {

Trajectory::Trajectory(std::vector<HebiTrajectoryPtr> trajectories, size_t number_of_waypoints, double start_time, double end_time)
  : trajectories_(trajectories),
    number_of_joints_ (trajectories.size()),
    number_of_waypoints_(number_of_waypoints),
    start_time_(start_time),
    end_time_(end_time)
{
}

std::shared_ptr<Trajectory> Trajectory::createUnconstrainedQp(
  const VectorXd& time_vector,
  const MatrixXd& positions,
  const MatrixXd* velocities,
  const MatrixXd* accelerations)
{
  std::shared_ptr<Trajectory> res;

  // Check argument validity
  size_t num_joints = positions.rows();
  size_t num_waypoints = positions.cols();
  if (time_vector.size() != num_waypoints)
    return res;
  if (velocities != nullptr && (velocities->rows() != num_joints && velocities->cols() != num_waypoints))
    return res;
  if (accelerations != nullptr && (accelerations->rows() != num_joints && accelerations->cols() != num_waypoints))
    return res;
  if (num_waypoints < 2)
    return res;

  // Put data into C-style arrays:
  double* time_vector_c = nullptr;
  double* positions_c = nullptr;
  double* velocities_c = nullptr;
  double* accelerations_c = nullptr;
  time_vector_c = new double[num_joints * num_waypoints];
  {
    Map<Matrix<double, Dynamic, Dynamic, RowMajor> > tmp(time_vector_c, num_waypoints, 1);
    tmp = time_vector;
  } 
  positions_c = new double[num_joints * num_waypoints];
  {
    Map<Matrix<double, Dynamic, Dynamic, RowMajor> > tmp(positions_c, num_joints, num_waypoints);
    tmp = positions;
  } 
  if (velocities != nullptr)
  {
    velocities_c = new double[num_joints * num_waypoints];
    Map<Matrix<double, Dynamic, Dynamic, RowMajor> > tmp(velocities_c, num_joints, num_waypoints);
    tmp = *velocities;
  }
  else // Default to [0 nan .... nan 0] (can remove when C API is updated)
  {
    velocities_c = new double[num_joints * num_waypoints];
    for (size_t joint = 0; joint < num_joints; ++joint)
    {
      velocities_c[joint * num_waypoints + 0] = 0.0f;
      velocities_c[joint * num_waypoints + num_waypoints - 1] = 0.0f;
      for (size_t waypoint = 1; waypoint < num_waypoints - 1; ++waypoint)
        velocities_c[joint * num_waypoints + waypoint] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  if (accelerations != nullptr)
  {
    accelerations_c = new double[num_joints * num_waypoints];
    Map<Matrix<double, Dynamic, Dynamic, RowMajor> > tmp(accelerations_c, num_joints, num_waypoints);
    tmp = *accelerations;
  }
  else // Default to [0 nan .... nan 0] (can remove when C API is updated)
  {
    accelerations_c = new double[num_joints * num_waypoints];
    for (size_t joint = 0; joint < num_joints; ++joint)
    {
      accelerations_c[joint * num_waypoints + 0] = 0.0f;
      accelerations_c[joint * num_waypoints + num_waypoints - 1] = 0.0f;
      for (size_t waypoint = 1; waypoint < num_waypoints - 1; ++waypoint)
        accelerations_c[joint * num_waypoints + waypoint] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  // Build C trajectory objects
  std::vector<HebiTrajectoryPtr> trajectories(num_joints, nullptr);
  for (size_t i = 0; i < num_joints; ++i)
  {
    HebiTrajectoryPtr trajectory = hebiTrajectoryCreateUnconstrainedQp(num_waypoints,
      (positions_c + i * num_waypoints),
      velocities_c == nullptr ? nullptr : (velocities_c + i * num_waypoints),
      accelerations_c == nullptr ? nullptr : (accelerations_c + i * num_waypoints),
      time_vector_c);
    // Failure? cleanup previous trajectories
    if (trajectory == nullptr)
    {
      for (size_t j = 0; j < i; ++j)
      {
        hebiTrajectoryRelease(trajectories[j]);
      }
      return res;
    }
    trajectories[i] = trajectory;
  }

  delete[] positions_c;
  if (velocities_c != nullptr)
    delete[] velocities_c;
  if (accelerations_c != nullptr)
    delete[] accelerations_c;

  // Create C++ wrapper
  return std::shared_ptr<Trajectory>(new Trajectory(trajectories, num_waypoints, time_vector[0], time_vector[time_vector.size() - 1]));
}

Trajectory::~Trajectory() noexcept
{
  for (HebiTrajectoryPtr traj : trajectories_)
    hebiTrajectoryRelease(traj);
}

double Trajectory::getDuration() const
{
  // Note -- could use any joint here, as they all have the same time vector
  return hebiTrajectoryGetDuration(trajectories_[0]);
}

bool Trajectory::getState(double time, VectorXd* position, VectorXd* velocity, VectorXd* acceleration) const
{
  double tmp_p, tmp_v, tmp_a;
  bool success = true;
  for (size_t i = 0; i < trajectories_.size(); ++i)
  {
    success = (hebiTrajectoryGetState(
      trajectories_[i],
      time,
      position == nullptr ? &tmp_p : &(*position)[i],
      velocity == nullptr ? &tmp_v : &(*velocity)[i],
      acceleration == nullptr ? &tmp_a : &(*acceleration)[i]) == 0) && success;
  }
  return success;
}

} // namespace trajectory
} // namespace hebi

