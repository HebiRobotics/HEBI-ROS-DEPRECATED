#pragma once

#include "hebi_status_codes.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// Types
////////////////////////////////////////////////////////////////////////////////

/**
 * A kinematics object which stores a tree of connected modules, and allows for
 * computation of forward kinematics and jacobians.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
typedef struct _HebiKinematics* HebiKinematicsPtr;

/**
 * Contains a kinematic body, which represents a transform from an input to one
 * or more outputs.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
typedef struct _HebiBody* HebiBodyPtr;

/**
 * Which frame to report results in (e.g., for getForwardKinematics and other
 * functions.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
typedef enum HebiFrameType
{
  FrameTypeCenterOfMass,
  FrameTypeOutput
} HebiFrameType;

////////////////////////////////////////////////////////////////////////////////
// Bodies
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates a one-dof actuator with the specified
 * properties.
 *
 * \param com Vector (length 3) of the center of mass location.
 * \param input_to_joint Matrix (4x4, row major order) of the transform from the
 * input of the actuator to the center of rotation about the degree of freedom.
 * \param joint_rotation_axis vector (length 3) of the axis of rotation for the
 * module.  The relative coordinate frame is the output of the "input_to_joint"
 * transform.
 * \param joint_to_output Matrix (4x4, row major order) of the transform from
 * the output of the rotation about the joint axis to frame of the output of the
 * actuator.
 *
 * \returns NULL if the actuator body could not be created; otherwise, pointer
 * to newly allocated body.  Must either be added to a kinematics object or
 * released via hebiBodyRelease(HebiBodyPtr).
 */
HebiBodyPtr hebiBodyCreateActuator(const float* com, const float* input_to_joint, const float* joint_rotation_axis, const float* joint_to_output);
/**
 * \brief Creates a kinematic body with a static transform to the output.
 *
 * \param com Vector (length 3) of the center of mass location.
 * \param num_outputs The number of available outputs.
 * \param outputs Matrices (list of n 4x4 transforms, row major order) of the
 * transforms from input to the available outputs of the body.
 * NOTE: currently, only a single output is supported.
 *
 * \returns NULL if the static body could not be created; otherwise, pointer to
 * newly allocated body.  Must either be added to a kinematics object or
 * released via 'hebiBodyRelease'.
 */
HebiBodyPtr hebiBodyCreateStatic(const float* com, int num_outputs, const float* outputs);

/**
 * \brief Frees resources created by this body.
 *
 * Note: Only do this if body has not been added to a kinematics object!  Once
 * added, the kinematics object manages the body's resources.
 *
 * The body should no longer be used after this function is called.
 *
 * \param A valid kinematic body object which has not been added to a kinematics
 * object.
 */
void hebiBodyRelease(HebiBodyPtr);

////////////////////////////////////////////////////////////////////////////////
// Kinematics
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates an object to hold a kinematic tree. This structure has a
 * single output available at the origin.
 *
 * Kinematics object created by this function must be released with
 * 'hebiKinematicsRelease' when no longer needed.
 */
HebiKinematicsPtr hebiKinematicsCreate();

/**
 * \brief Sets the fixed transform from the origin to the first added body.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param transform A 4x4 homogeneous transform, in row major order.
 */
void hebiKinematicsSetBaseFrame(HebiKinematicsPtr kinematics, const float* transform);

/**
 * \brief Retreives the fixed transform from the origin to the first added body.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param transform A allocated 16 element array of floats; this is filled
 * in by the function with the 4x4 homogeneous transform in row major order.
 */
void hebiKinematicsGetBaseFrame(HebiKinematicsPtr kinematics, float* transform);

/**
 * \brief Return the number of frames in the forward kinematics.
 *
 * Note that this depends on the type of frame requested -- for center of mass
 * frames, there is one per added body; for output frames, there is one per
 * output per body.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 */
int hebiKinematicsGetNumberOfFrames(HebiKinematicsPtr kinematics, HebiFrameType frame_type);

/**
 * \brief Returns the number of settable degrees of freedom in the kinematic
 * tree. (This is equal to the number of actuators added).
 *
 * \param kinematics A valid HEBI Kinematics object.
 */
int hebiKinematicsGetNumberOfDoFs(HebiKinematicsPtr kinematics);

/**
 * \brief Add a body to a parent body connected to a kinematic tree object.
 *
 * After the addition, the kinematics object manages the resources of the added
 * body.
 *
 * The added body is assumed to connect to an available output on a body that
 * has already been attached to the kinematic tree.  That body should be passed
 * in as 'existing_body', and the index of the requested output on that body
 * should be given as 'output_index'.
 *
 * To attach the initial body to the kinematics object, use NULL for the
 * 'existing_body' argument.
*
 * NOTE: currently, only a single output is supported for each body (e.g., a
 * kinematic chain), and so the 'existing_body' and 'output_index' parameters
 * are not checked.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param existing_body The parent body which the body is added to (or null to
 * add the initial body to the tree).
 * \param output_index The index of the requested output on the parent body on
 * which to attach this body.
 * \param new_body The kinematic body which is added to the tree.
 *
 * \returns HebiStatusSuccess on success, otherwise HebiStatusFailure (e.g., the parent body's
 * requested output is invalid or already occupied).
 */
HebiStatusCode hebiKinematicsAddBody(HebiKinematicsPtr kinematics, HebiBodyPtr existing_body, int output_index, HebiBodyPtr new_body);

/**
 * \brief Generates the forward kinematics for the given kinematic tree.
 *
 * The order of the returned frames is in a depth-first tree. As an example,
 * assume a body A has one output, to which body B is connected to. Body B has
 * two outputs; actuator C is attached to the first output and actuator E is
 * attached to the second output.  Body D is attached to the only output of
 * actuator C:
 *
 * (BASE) A - B(1) - C - D
 *           (2)
 *            |
 *            E
 *
 * For center of mass frames, the returned frames would be A-B-C-D-E.
 *
 * For output frames, the returned frames would be A-B(1)-C-D-B(2)-E.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 * \param positions A vector of joint positions/angles (in SI units of meters or
 * radians) equal in length to the number of DoFs of the kinematic tree.
 * \param frames An allocated (16 x number of frames) array of floats; this is
 * filled in by the function with the 4x4 homogeneous transform of each frame,
 * each given in row major order. Note that the number of frames depends on the
 * frame type!
 */
void hebiKinematicsGetForwardKinematics(HebiKinematicsPtr kinematics, HebiFrameType frame_type, const double* positions, float* frames);

/**
 * \brief Generates the jacobian for each frame in the given kinematic tree.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 * \param positions A vector of joint positions/angles (in SI units of meters or
 * radians) equal in length to the number of DoFs of the kinematic tree.
 * \param jacobians An allocated (6 x number of dofs x number of frames) array
 * of floats; this is filled in by the function with the 6 x number of dofs
 * jacobian for each frame, each given in row major order.  Note that the number
 * of frames depends on the frame type!
 */
void hebiKinematicsGetJacobians(HebiKinematicsPtr kinematics, HebiFrameType frame_type, const double* positions, float* jacobians);

/**
 * \brief Generates the forward kinematics to the end effector (leaf node)
 * frame(s).
 *
 * Note -- for center of mass frames, this is one per leaf node; for output
 * frames, this is one per output per leaf node, in depth first order.
 *
 * \param kinematics A valid HEBI Kinematics object.
 * \param frame_type Which type of frame to consider -- see HebiFrameType enum.
 * \param positions A vector of joint positions/angles (in SI units of meters or
 * radians) equal in length to the number of DoFs of the kinematic tree.
 * \param transforms An allocated (16 x number of frames) array of floats; this is
 * filled in by the function with the 4x4 homogeneous transform of each frame,
 * each given in row major order. Note that the number of frames depends on the
 * frame type!
 */
void hebiKinematicsGetEndEffector(HebiKinematicsPtr kinematics, HebiFrameType frame_type, const double* positions, float* transforms);

/**
 * \brief Frees resources created by this kinematics object.
 *
 * Kinematics object should no longer be used after this function is called!
 *
 * \param kinematics A valid HEBI Kinematics object.
 */
void hebiKinematicsRelease(HebiKinematicsPtr kinematics);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
