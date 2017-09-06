#pragma once

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

#include <math.h>

#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif
  
////////////////////////////////////////////////////////////////////////////////
// Data types
////////////////////////////////////////////////////////////////////////////////

/**
 * A structure which stores the parameters for a 1-DOF rotary actuator
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
typedef struct HebiKinematicParametersActuator
{
  float com[3];
  float input_to_joint[16];
  float joint_rotation_axis[3];
  float joint_to_output[16];
} HebiKinematicParametersActuator;

/**
 * A structure which stores the parameters for a 1 output static body.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
typedef struct HebiKinematicParametersStaticBody
{
  float com[3];
  float output[16];
} HebiKinematicParametersStaticBody;

////////////////////////////////////////////////////////////////////////////////
// Parameter types for helper functions below.
////////////////////////////////////////////////////////////////////////////////

/**
 * How a particular body is mounted on the previous body, when this value is
 * quantized. Not all mounting types are supported for each body type.
 */
typedef enum HebiMountingType
{
  MountingTypeLeft,
  MountingTypeRight,
  MountingTypeLeftInside,
  MountingTypeRightInside,
  MountingTypeLeftOutside,
  MountingTypeRightOutside,
} HebiMountingType;

////////////////////////////////////////////////////////////////////////////////
// Some geometric/transform related helper functions.
////////////////////////////////////////////////////////////////////////////////

/*
 * Internal helper function to set values in a vector.
 */
static void hebiKinematicsSetVector3f(float* vector, float x, float y, float z)
{
  vector[0] = x;
  vector[1] = y;
  vector[2] = z;
}
/*
 * Internal helper function to set values in a homogenous transform.
 */
static void hebiKinematicsSetMatrix4fIdentity(float* matrix)
{
  matrix[0] = 1;
  matrix[1] = 0;
  matrix[2] = 0;
  matrix[3] = 0;
  matrix[4] = 0;
  matrix[5] = 1;
  matrix[6] = 0;
  matrix[7] = 0;
  matrix[8] = 0;
  matrix[9] = 0;
  matrix[10] = 1;
  matrix[11] = 0;
  matrix[12] = 0;
  matrix[13] = 0;
  matrix[14] = 0;
  matrix[15] = 1;
}

/*
 * Internal helper function to set values in a homogenous transform.
 */
static void hebiKinematicsSetMatrix4fRotX(float* matrix, float radians)
{
  matrix[0] = 1;
  matrix[1] = 0;
  matrix[2] = 0;
  matrix[4] = 0;
  matrix[5] = cos(radians);
  matrix[6] = -sin(radians);
  matrix[8] = 0;
  matrix[9] = sin(radians);
  matrix[10] = cos(radians);
}

/*
 * Internal helper function to set values in a homogenous transform.
 */
static void hebiKinematicsSetMatrix4fTranslate(float* matrix, float x, float y, float z)
{
  matrix[3] = x;
  matrix[7] = y;
  matrix[11] = z;
}

////////////////////////////////////////////////////////////////////////////////
// Helper functions for returning parameters for specific kinematic bodies.
////////////////////////////////////////////////////////////////////////////////

/**
 * Sets the input parameter to the kinematic parameters for an X5-series
 * actuator.
 *
 * @returns HebiStatusSuccess on success, or HebiStatusInvalidArgument (e.g., NULL pointer or
 * other invalid arguments).
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static HebiStatusCode hebiKinematicParametersX5(HebiKinematicParametersActuator* params)
{
  // Ignore null pointers
  if (!params)
    return HebiStatusInvalidArgument;

  hebiKinematicsSetVector3f(params->com, 0, 0, 0.0155f); 
  hebiKinematicsSetMatrix4fIdentity(params->input_to_joint);
  hebiKinematicsSetMatrix4fTranslate(params->input_to_joint, 0, 0, 0.03105f);
  // z axis rotation
  hebiKinematicsSetVector3f(params->joint_rotation_axis, 0, 0, 1);
  hebiKinematicsSetMatrix4fIdentity(params->joint_to_output);
  return HebiStatusSuccess;
}

/**
 * Sets the input parameter to the kinematic parameters for an X8-series
 * actuator.
 *
 * @returns HebiStatusSuccess on success, or HebiStatusInvalidArgument (e.g., NULL pointer or
 * other invalid arguments).
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static HebiStatusCode hebiKinematicParametersX8(HebiKinematicParametersActuator* params)
{
  // Ignore null pointers
  if (!params)
    return HebiStatusInvalidArgument;

  hebiKinematicsSetVector3f(params->com, 0, 0, 0.0226f); 
  hebiKinematicsSetMatrix4fIdentity(params->input_to_joint);
  hebiKinematicsSetMatrix4fTranslate(params->input_to_joint, 0, 0, 0.0451f);
  // z axis rotation
  hebiKinematicsSetVector3f(params->joint_rotation_axis, 0, 0, 1);
  hebiKinematicsSetMatrix4fIdentity(params->joint_to_output);
  return HebiStatusSuccess;
}

/**
 * Sets the input parameter to the kinematic parameters for an X5-series
 * light bracket.
 *
 * The "mounting" parameter can be set to "Left" or "Right".
 *
 * @returns HebiStatusSuccess on success, or HebiStatusInvalidArgument (e.g., NULL pointer or
 * other invalid arguments).
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static HebiStatusCode hebiKinematicParametersX5LightBracket(
  HebiKinematicParametersStaticBody* params, HebiMountingType mounting)
{
  // Ignore null pointers and invalid parameters
  if (!params)
    return HebiStatusInvalidArgument;
  if (mounting != MountingTypeLeft && mounting != MountingTypeRight)
    return HebiStatusInvalidArgument;

  float mult = 1;
  if (mounting == MountingTypeRight)
    mult = -1; 
 
  hebiKinematicsSetVector3f(params->com, 0, mult * 0.0215f, 0.02f);
  hebiKinematicsSetMatrix4fIdentity(params->output);
  hebiKinematicsSetMatrix4fRotX(params->output, mult * (-M_PI / 2.0f));
  hebiKinematicsSetMatrix4fTranslate(params->output, 0, mult * .043f, 0.04f);
  return HebiStatusSuccess;
}

/**
 * Sets the input parameter to the kinematic parameters for an X5-series
 * heavy bracket.
 *
 * The "mounting" parameter can be set to "LeftInside", "LeftOutside",
 * "RightInside", or "RightOutside".
 *
 * @returns HebiStatusSuccess on success, or HebiStatusInvalidArgument (e.g., NULL pointer or
 * other invalid arguments).
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static HebiStatusCode hebiKinematicParametersX5HeavyBracket(
  HebiKinematicParametersStaticBody* params, HebiMountingType mounting)
{
  // Ignore null pointers and invalid parameters
  if (!params)
    return HebiStatusInvalidArgument;
  if (mounting != MountingTypeLeftInside &&
      mounting != MountingTypeLeftOutside &&
      mounting != MountingTypeRightInside &&
      mounting != MountingTypeRightOutside)
    return HebiStatusInvalidArgument;

  float lr_mult = 1;
  if (mounting == MountingTypeRightInside || mounting == MountingTypeRightOutside)
    lr_mult = -1; 

  float y_dist = -0.0225f; // Inside
  if (mounting == MountingTypeLeftOutside || mounting == MountingTypeRightOutside)
    y_dist = 0.0375f; // Outside
 
  hebiKinematicsSetVector3f(params->com, 0, lr_mult * 0.5f * y_dist, 0.0275f);
  hebiKinematicsSetMatrix4fIdentity(params->output);
  hebiKinematicsSetMatrix4fRotX(params->output, lr_mult * (-M_PI / 2.0f));
  hebiKinematicsSetMatrix4fTranslate(params->output, 0, lr_mult * y_dist, 0.055f);
  return HebiStatusSuccess;
}

/**
 * The kinematic parameters for an X5-series tube link.
 *
 * \param extension The length, from center of the previous actuator's output to
 * the center of the subsequent actuator's input, in meters.
 * \param twist The twist about the central axis of the tube.  A twist of zero
 * refers to an input and output frame that are aligned in rotation, but offset
 * in the z-direction.
 * 
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 20 Apr 2017
 */
static HebiStatusCode hebiKinematicParametersX5Link(
  HebiKinematicParametersStaticBody* params, float extension, float twist)
{
  // Ignore null pointers and invalid parameters
  if (!params)
    return HebiStatusInvalidArgument;

  // Edge of bracket to center of pipe.
  float edge_to_center = .0175f;

  // Note that this ignores the effect of the end brackets on moving the com
  // slightly off center.
  hebiKinematicsSetVector3f(params->com, extension * 0.5f, 0, edge_to_center);
  hebiKinematicsSetMatrix4fIdentity(params->output);
  hebiKinematicsSetMatrix4fRotX(params->output, twist);
  hebiKinematicsSetMatrix4fTranslate(params->output, extension,
                                     -edge_to_center * sin(twist),
                                     edge_to_center * (1 + cos(twist)));
  return HebiStatusSuccess;
}

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif
