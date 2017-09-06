#pragma once

#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_info.h"
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// Types
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief The C-style's API representation of a trajectory.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * Used to generate position, velocity, and acceleration for different joints.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 2 Feb 2017
 */
typedef struct _HebiTrajectory* HebiTrajectoryPtr;

////////////////////////////////////////////////////////////////////////////////
// Trajectories
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates a HebiTrajectory object for a single joint using the given
 * parameters; this must be released with hebiTrajectoryRelease after use.
 *
 * \param num_waypoints The number of waypoints.
 * \param positions A vector of waypoints for this joint; should be
 * num_waypoints in length. Any elements that are NAN will be considered free
 * parameters, and will be set by the function.  Values of +/-infinity are not
 * allowed.
 * \param velocities An optional vector of velocity constraints at the
 * corresponding waypoints; should either be NULL or num_waypoints in length.
 * Any elements that are NAN will be considered free parameters, and will be set
 * by the function.  Values of +/-infinity are not allowed.
 * \param accelerations An optional vector of acceleration constraints at the
 * corresponding waypoints; should either be NULL or num_waypoints in length.
 * Any elements that are NAN will be considered free parameters, and will be set
 * by the function.  Values of +/-infinity are not allowed.
 * \param time_vector A vector of times to reach each waypoint; this must
 * be defined (not NULL, and not NAN for any element).  The first element must
 * be zero.
 *
 * \returns A HebiTrajectory object if there were no errors, and the trajectory
 * has been created. A NULL value indicates that there was an error, but does
 * not specify any details about the error at this time.
 */
HebiTrajectoryPtr hebiTrajectoryCreateUnconstrainedQp(int num_waypoints, double* positions, double* velocities, double* accelerations, double* time_vector);

/**
 * \brief Frees resources created by this trajectory.
 *
 * Trajectory should no longer be used after this function is called!
 */
void hebiTrajectoryRelease(HebiTrajectoryPtr trajectory);

/**
 * \brief Returns the length of this trajectory (in seconds).
 */
double hebiTrajectoryGetDuration(HebiTrajectoryPtr trajectory);

/**
 * \brief Gets the value of the trajectory at a given time.
 *
 * \param trajectory A HebiTrajectory object
 * \param time The time within the trajectory (from 0 to the duration of the
 * trajectory) at which to query.
 * \param position The position at the given time, as defined by this
 * trajectory.
 * \param velocity The velocity at the given time, as defined by this
 * trajectory.
 * \param acceleration The acceleration at the given time, as defined by this
 * trajectory.
 *
 * \returns HebiStatusSuccess on success, otherwise HebiStatusFailure
 */
HebiStatusCode hebiTrajectoryGetState(HebiTrajectoryPtr trajectory, double time, double* position, double* velocity, double* acceleration);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
