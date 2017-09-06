#pragma once

#include "hebi_kinematics.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * An inverse kinematics object which uses a kinematics object to search for
 * joint angles that optimize any of several objectives while respecting defined
 * constraints.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 21 Jul 2017
 */
typedef struct _HebiIK* HebiIKPtr;

/**
 * \brief Creates an IK (inverse kinematics) object that allows for solving for
 * joint angles/positions given objectives and constraints.
 *
 * This optimization is completed using a specified forward kinematics object.
 *
 * The objectives and constraints are stored with this object.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 21 Jul 2017
 */
HebiIKPtr hebiIKCreate();

/**
 * \brief Add an objective that optimizes for the end effector output frame
 * origin to be at the given (x, y, z) point.
 *
 * If an end effector position objective already exists, this will replace it.
 *
 * \param weight The weight of this objective relative to any other objective
 * functions (this objective is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param x The desired x position of the end effector frame. 'NaN' can be
 * passed to ignore this variable.
 * \param y The desired y position of the end effector frame. 'NaN' can be
 * passed to ignore this variable.
 * \param z The desired z position of the end effector frame. 'NaN' can be
 * passed to ignore this variable.
 *
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g., incompatible with
 * existing objectives, or all components are set to 'NaN')
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorPosition(HebiIKPtr,
  float weight, float x, float y, float z);

/**
 * \brief Add an objective that optimizes for the end effector output frame
 * orientation to be given by the 3x3 (row major) rotation matrix. Note that
 * this is incompatible with the end effector tip axis objective.
 *
 * If an end effector orientation objective already exists, this will replace
 * it.
 *
 * \param weight The weight of this objective relative to any other objective
 * functions (this objective is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param matrix The desired orientation of the end effector frame, as a 3x3
 * rotation matrix in row major order.
 *
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g., incompatible with
 * existing objectives, or rotation matrix is invalid.)
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorSO3(HebiIKPtr,
  float weight, float* matrix);

/**
 * \brief Add an objective that points the end effector's z axis in a given
 * direction.  Note that this is incompatible with the end effector S03
 * orientation objective.
 *
 * If an end effector orientation objective already exists, this will replace
 * it.
 *
 * \param weight The weight of this objective relative to any other objective
 * functions (this objective is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param x The desired end effector z axis, projected onto the x axis.
 * \param y The desired end effector z axis, projected onto the y axis.
 * \param z The desired end effector z axis, projected onto the z axis.
 *
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g., incompatible with
 * existing objectives, or rotation matrix is invalid.)
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorTipAxis(HebiIKPtr,
  float weight, float x, float y, float z);

/**
 * \brief Define joint angle constraints.
 *
 * NaN or +/- infinity can be set on particular joints to ignore joint limits.
 * Currently, the joint limit constraints are two-sided, which means that any
 * joint must either have both min/max set to NaN/inf, or have neither.
 *
 * \param weight The weight of this constraint relative to any other objective
 * functions (this constraint is multiplied by this weight before passing to the
 * optimizer). Defaults to 1.0.
 * \param num_joints The number of elements in the min_positions and
 * max_positions arrays.
 * \param min_positions An array with the minimum joint limit for each joint, or
 * NaN or inf if unlimited. Must have num_joints elements.
 * \param max_positions An array with the maximum joint limit for each joint, or
 * NaN or inf if unlimited. Must have num_joints elements.
 *
 * \return HebiStatusSuccess on success; otherwise a failure code
 */
HebiStatusCode hebiIKAddConstraintsJointAngles(HebiIKPtr,
  float weight, int num_joints, double* min_positions, double* max_positions);

/**
 * \brief Clears the objectives and constraints from this IK object, along
 * with any modifications to the default algorithm parameters.
 */
void hebiIKClearAll(HebiIKPtr);

/**
 * \brief Solves for an inverse kinematics solution that moves the end effector
 * to a given point.
 *
 * Note: multiple "hebiIKSolve" calls can be made using the same IK object. 
 *
 * \param ik A valid HEBI IK object.
 * \param kin A valid HEBI Forward Kinematics object.
 * \param initial_positions The seed positions/angles (in SI units of meters or
 * radians) to start the IK search from; equal in length to the number of DoFs
 * of the kinematic tree.
 * \param ik_solution Allocated array of doubles equal in length to the
 * number of DoFs of the kinematic tree; the function will will in this array
 * with the IK solution (in SI units of meters or radians).
 * \param result_info Reserved for future use (will enable more information
 * about output of optimization such as success/failure, function error, etc).
 * This can currently be set to NULL.
 *
 * \return HebiStatusSuccess on success, other values on failure (e.g., no objectives given or
 * dimension mismatch between kinematics object and stored objectives).
 */
HebiStatusCode hebiIKSolve(HebiIKPtr ik, HebiKinematicsPtr kin,
  const double* initial_positions, double* ik_solution, void* result_info);

/**
 * \brief Frees resources created by this inverse kinematics object.
 *
 * IK object should no longer be used after this function is called!
 *
 * \param ik A valid HEBI inverse kinematics object.
 */
void hebiIKRelease(HebiIKPtr ik);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
