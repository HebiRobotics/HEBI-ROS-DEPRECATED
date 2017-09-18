#pragma once

#include <math.h>
#include <stdint.h>
#include <stddef.h>


#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

////////////////////////////////////////////////////////////////////////////////
// Enum Types
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiStatusCode {
  HebiStatusSuccess = 0, ///Success; no failures occurred
  HebiStatusInvalidArgument = 1, ///An invalid argument was supplied to the routine (e.g. null pointer)
  HebiStatusBufferTooSmall = 2, ///A buffer supplied to the routine was too small (normally determined by a size parameter)
  HebiStatusValueNotSet = 3, ///Returned when an accessor function attempts to retrieve a field which is not set
  HebiStatusFailure = 4, ///Generic code for failure; this is generally used for an internal or unknown failure
  HebiStatusArgumentOutOfRange = 5 ///Failure caused by an argument supplied to the routine that is out of range (e.g. a negative integer when only a positive integer is valid)
} HebiStatusCode;

////////////////////////////////////////////////////////////////////////////////
// Command Enums
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiCommandFloatField {
  HebiCommandFloatVelocity, ///Velocity of the module output (post-spring), in radians/second.
  HebiCommandFloatEffort, ///Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
  HebiCommandFloatPositionKp, ///Proportional PID gain for position
  HebiCommandFloatPositionKi, ///Integral PID gain for position
  HebiCommandFloatPositionKd, ///Derivative PID gain for position
  HebiCommandFloatPositionFeedForward, ///Feed forward term for position (this term is multiplied by the target and added to the output).
  HebiCommandFloatPositionDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiCommandFloatPositionIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiCommandFloatPositionPunch, ///Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiCommandFloatPositionMinTarget, ///Minimum allowed value for input to the PID controller
  HebiCommandFloatPositionMaxTarget, ///Maximum allowed value for input to the PID controller
  HebiCommandFloatPositionTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatPositionMinOutput, ///Output from the PID controller is limited to a minimum of this value.
  HebiCommandFloatPositionMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
  HebiCommandFloatPositionOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatVelocityKp, ///Proportional PID gain for velocity
  HebiCommandFloatVelocityKi, ///Integral PID gain for velocity
  HebiCommandFloatVelocityKd, ///Derivative PID gain for velocity
  HebiCommandFloatVelocityFeedForward, ///Feed forward term for velocity (this term is multiplied by the target and added to the output).
  HebiCommandFloatVelocityDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiCommandFloatVelocityIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiCommandFloatVelocityPunch, ///Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiCommandFloatVelocityMinTarget, ///Minimum allowed value for input to the PID controller
  HebiCommandFloatVelocityMaxTarget, ///Maximum allowed value for input to the PID controller
  HebiCommandFloatVelocityTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatVelocityMinOutput, ///Output from the PID controller is limited to a minimum of this value.
  HebiCommandFloatVelocityMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
  HebiCommandFloatVelocityOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatEffortKp, ///Proportional PID gain for effort
  HebiCommandFloatEffortKi, ///Integral PID gain for effort
  HebiCommandFloatEffortKd, ///Derivative PID gain for effort
  HebiCommandFloatEffortFeedForward, ///Feed forward term for effort (this term is multiplied by the target and added to the output).
  HebiCommandFloatEffortDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiCommandFloatEffortIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiCommandFloatEffortPunch, ///Constant offset to the effort PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiCommandFloatEffortMinTarget, ///Minimum allowed value for input to the PID controller
  HebiCommandFloatEffortMaxTarget, ///Maximum allowed value for input to the PID controller
  HebiCommandFloatEffortTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatEffortMinOutput, ///Output from the PID controller is limited to a minimum of this value.
  HebiCommandFloatEffortMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
  HebiCommandFloatEffortOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiCommandFloatSpringConstant, ///The spring constant of the module.
} HebiCommandFloatField;

typedef enum HebiCommandHighResAngleField {
  HebiCommandHighResAnglePosition, ///Position of the module output (post-spring), in radians.
} HebiCommandHighResAngleField;

typedef enum HebiCommandNumberedFloatField {
  HebiCommandNumberedFloatDebug, ///Values for internal debug functions (channel 1-9 available).
} HebiCommandNumberedFloatField;

typedef enum HebiCommandBoolField {
  HebiCommandBoolPositionDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiCommandBoolVelocityDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiCommandBoolEffortDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
} HebiCommandBoolField;

typedef enum HebiCommandStringField {
  HebiCommandStringName, ///Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
  HebiCommandStringFamily, ///Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
} HebiCommandStringField;

typedef enum HebiCommandFlagField {
  HebiCommandFlagSaveCurrentSettings, ///Indicates if the module should save the current values of all of its settings.
} HebiCommandFlagField;

typedef enum HebiCommandEnumField {
  HebiCommandEnumControlStrategy, ///How the position, velocity, and effort PID loops are connected in order to control motor PWM.
} HebiCommandEnumField;

typedef enum HebiCommandIoPinBank {
  HebiCommandIoBankA, ///I/O pin bank a (pins 1-8 available)
  HebiCommandIoBankB, ///I/O pin bank b (pins 1-8 available)
  HebiCommandIoBankC, ///I/O pin bank c (pins 1-8 available)
  HebiCommandIoBankD, ///I/O pin bank d (pins 1-8 available)
  HebiCommandIoBankE, ///I/O pin bank e (pins 1-8 available)
  HebiCommandIoBankF, ///I/O pin bank f (pins 1-8 available)
} HebiCommandIoPinBank;

typedef enum HebiCommandLedField {
  HebiCommandLedLed, ///The module's LED.
} HebiCommandLedField;

////////////////////////////////////////////////////////////////////////////////
// Feedback Enums
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiFeedbackFloatField {
  HebiFeedbackFloatBoardTemperature, ///Ambient temperature inside the module (measured at the IMU chip), in degrees Celsius.
  HebiFeedbackFloatProcessorTemperature, ///Temperature of the processor chip, in degrees Celsius.
  HebiFeedbackFloatVoltage, ///Bus voltage that the module is running at (in Volts).
  HebiFeedbackFloatVelocity, ///Velocity of the module output (post-spring), in radians/second.
  HebiFeedbackFloatEffort, ///Effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
  HebiFeedbackFloatVelocityCommand, ///Commanded velocity of the module output (post-spring), in radians/second.
  HebiFeedbackFloatEffortCommand, ///Commanded effort at the module output; units vary (e.g., N * m for rotational joints and N for linear stages).
  HebiFeedbackFloatDeflection, ///Difference (in radians) between the pre-spring and post-spring output position.
  HebiFeedbackFloatDeflectionVelocity, ///Velocity (in radians/second) of the difference between the pre-spring and post-spring output position.
  HebiFeedbackFloatMotorVelocity, ///The velocity (in radians/second) of the motor shaft.
  HebiFeedbackFloatMotorCurrent, ///Current supplied to the motor.
  HebiFeedbackFloatMotorSensorTemperature, ///The temperature from a sensor near the motor housing.
  HebiFeedbackFloatMotorWindingCurrent, ///The estimated current in the motor windings.
  HebiFeedbackFloatMotorWindingTemperature, ///The estimated temperature of the motor windings.
  HebiFeedbackFloatMotorHousingTemperature, ///The estimated temperature of the motor housing.
} HebiFeedbackFloatField;

typedef enum HebiFeedbackHighResAngleField {
  HebiFeedbackHighResAnglePosition, ///Position of the module output (post-spring), in radians.
  HebiFeedbackHighResAnglePositionCommand, ///Commanded position of the module output (post-spring), in radians.
} HebiFeedbackHighResAngleField;

typedef enum HebiFeedbackNumberedFloatField {
  HebiFeedbackNumberedFloatDebug, ///Values for internal debug functions (channel 1-9 available).
} HebiFeedbackNumberedFloatField;

typedef enum HebiFeedbackUInt64Field {
  HebiFeedbackUInt64SequenceNumber, ///Sequence number going to module (local)
  HebiFeedbackUInt64ReceiveTime, ///Timestamp of when message was received from module (local)
  HebiFeedbackUInt64TransmitTime, ///Timestamp of when message was transmitted to module (local)
  HebiFeedbackUInt64HardwareReceiveTime, ///Timestamp of when message was received by module (remote)
  HebiFeedbackUInt64HardwareTransmitTime, ///Timestamp of when message was transmitted from module (remote)
} HebiFeedbackUInt64Field;

typedef enum HebiFeedbackVector3fField {
  HebiFeedbackVector3fAccelerometer, ///Accelerometer data, in m/s^2.
  HebiFeedbackVector3fGyro, ///Gyro data, in radians/second.
} HebiFeedbackVector3fField;

typedef enum HebiFeedbackIoPinBank {
  HebiFeedbackIoBankA, ///I/O pin bank a (pins 1-8 available)
  HebiFeedbackIoBankB, ///I/O pin bank b (pins 1-8 available)
  HebiFeedbackIoBankC, ///I/O pin bank c (pins 1-8 available)
  HebiFeedbackIoBankD, ///I/O pin bank d (pins 1-8 available)
  HebiFeedbackIoBankE, ///I/O pin bank e (pins 1-8 available)
  HebiFeedbackIoBankF, ///I/O pin bank f (pins 1-8 available)
} HebiFeedbackIoPinBank;

typedef enum HebiFeedbackLedField {
  HebiFeedbackLedLed, ///The module's LED.
} HebiFeedbackLedField;

////////////////////////////////////////////////////////////////////////////////
// Info Enums
////////////////////////////////////////////////////////////////////////////////

typedef enum HebiInfoFloatField {
  HebiInfoFloatPositionKp, ///Proportional PID gain for position
  HebiInfoFloatPositionKi, ///Integral PID gain for position
  HebiInfoFloatPositionKd, ///Derivative PID gain for position
  HebiInfoFloatPositionFeedForward, ///Feed forward term for position (this term is multiplied by the target and added to the output).
  HebiInfoFloatPositionDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiInfoFloatPositionIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiInfoFloatPositionPunch, ///Constant offset to the position PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiInfoFloatPositionMinTarget, ///Minimum allowed value for input to the PID controller
  HebiInfoFloatPositionMaxTarget, ///Maximum allowed value for input to the PID controller
  HebiInfoFloatPositionTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatPositionMinOutput, ///Output from the PID controller is limited to a minimum of this value.
  HebiInfoFloatPositionMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
  HebiInfoFloatPositionOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatVelocityKp, ///Proportional PID gain for velocity
  HebiInfoFloatVelocityKi, ///Integral PID gain for velocity
  HebiInfoFloatVelocityKd, ///Derivative PID gain for velocity
  HebiInfoFloatVelocityFeedForward, ///Feed forward term for velocity (this term is multiplied by the target and added to the output).
  HebiInfoFloatVelocityDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiInfoFloatVelocityIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiInfoFloatVelocityPunch, ///Constant offset to the velocity PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiInfoFloatVelocityMinTarget, ///Minimum allowed value for input to the PID controller
  HebiInfoFloatVelocityMaxTarget, ///Maximum allowed value for input to the PID controller
  HebiInfoFloatVelocityTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatVelocityMinOutput, ///Output from the PID controller is limited to a minimum of this value.
  HebiInfoFloatVelocityMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
  HebiInfoFloatVelocityOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatEffortKp, ///Proportional PID gain for effort
  HebiInfoFloatEffortKi, ///Integral PID gain for effort
  HebiInfoFloatEffortKd, ///Derivative PID gain for effort
  HebiInfoFloatEffortFeedForward, ///Feed forward term for effort (this term is multiplied by the target and added to the output).
  HebiInfoFloatEffortDeadZone, ///Error values within +/- this value from zero are treated as zero (in terms of computed proportional output, input to numerical derivative, and accumulated integral error).
  HebiInfoFloatEffortIClamp, ///Maximum allowed value for the output of the integral component of the PID loop; the integrated error is not allowed to exceed value that will generate this number.
  HebiInfoFloatEffortPunch, ///Constant offset to the effort PID output outside of the deadzone; it is added when the error is positive and subtracted when it is negative.
  HebiInfoFloatEffortMinTarget, ///Minimum allowed value for input to the PID controller
  HebiInfoFloatEffortMaxTarget, ///Maximum allowed value for input to the PID controller
  HebiInfoFloatEffortTargetLowpass, ///A simple lowpass filter applied to the target set point; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatEffortMinOutput, ///Output from the PID controller is limited to a minimum of this value.
  HebiInfoFloatEffortMaxOutput, ///Output from the PID controller is limited to a maximum of this value.
  HebiInfoFloatEffortOutputLowpass, ///A simple lowpass filter applied to the controller output; needs to be between 0 and 1.  At each timestep: x_t = x_t * a + x_{t-1} * (1 - a).
  HebiInfoFloatSpringConstant, ///The spring constant of the module.
} HebiInfoFloatField;

typedef enum HebiInfoBoolField {
  HebiInfoBoolPositionDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiInfoBoolVelocityDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
  HebiInfoBoolEffortDOnError, ///Controls whether the Kd term uses the "derivative of error" or "derivative of measurement."  When the setpoints have step inputs or are noisy, setting this to @c false can eliminate corresponding spikes or noise in the output.
} HebiInfoBoolField;

typedef enum HebiInfoStringField {
  HebiInfoStringName, ///Sets the name for this module. Name must be null-terminated character string for the name; must be <= 20 characters.
  HebiInfoStringFamily, ///Sets the family for this module. Name must be null-terminated character string for the family; must be <= 20 characters.
} HebiInfoStringField;

typedef enum HebiInfoFlagField {
  HebiInfoFlagSaveCurrentSettings, ///Indicates if the module should save the current values of all of its settings.
} HebiInfoFlagField;

typedef enum HebiInfoEnumField {
  HebiInfoEnumControlStrategy, ///How the position, velocity, and effort PID loops are connected in order to control motor PWM.
} HebiInfoEnumField;

typedef enum HebiInfoLedField {
  HebiInfoLedLed, ///The module's LED.
} HebiInfoLedField;

////////////////////////////////////////////////////////////////////////////////
// Kinematics Enums
////////////////////////////////////////////////////////////////////////////////

/**
 * How a particular body is mounted on the previous body, when this value is
 * quantized. Not all mounting types are supported for each body type.
 */
typedef enum HebiMountingType
{
  HebiMountingTypeLeft,
  HebiMountingTypeRight,
  HebiMountingTypeLeftInside,
  HebiMountingTypeRightInside,
  HebiMountingTypeLeftOutside,
  HebiMountingTypeRightOutside,
} HebiMountingType;

/**
 * Which frame to report results in (e.g., for getForwardKinematics and other
 * functions.
 */
typedef enum HebiFrameType
{
  HebiFrameTypeCenterOfMass,
  HebiFrameTypeOutput
} HebiFrameType;

////////////////////////////////////////////////////////////////////////////////
// Typedefs
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief The C-style's API representation of a command.
 *
 * Encapsulates data to be sent to a module
 */
typedef struct _HebiCommand* HebiCommandPtr;

/**
 * \brief The C-style's API representation of feedback.
 *
 * Encapsulates feedback received from a module
 */
typedef struct _HebiFeedback* HebiFeedbackPtr;

/**
 * \brief The C-style's API representation of a group.
 *
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 */
typedef struct _HebiGroup* HebiGroupPtr;

/**
 * \brief The C-style's API representation of a group command.
 *
 * This is an iterable list of command structures; commands are sent to modules in a group using the
 * fields within this structure.
 */
typedef struct _HebiGroupCommand* HebiGroupCommandPtr;

/**
 * \brief The C-style's API representation of group feedback.
 *
 * This is an iterable list of feedback structures; feedback from modules in a group are retrieved primarily through
 * this structure.
 */
typedef struct _HebiGroupFeedback* HebiGroupFeedbackPtr;

/**
 * \brief The C-style's API representation of group info.
 *
 * This is an iterable list of info structures; info from modules in a group are retrieved primarily through
 * this structure.
 */
typedef struct _HebiGroupInfo* HebiGroupInfoPtr;

/**
 * \brief The C-style's API representation of a group.
 *
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 */
typedef struct _HebiInfo* HebiInfoPtr;

/**
 * An inverse kinematics object which uses a kinematics object to search for
 * joint angles that optimize any of several objectives while respecting defined
 * constraints.
 */
typedef struct _HebiIK* HebiIKPtr;

/**
 * A kinematics object which stores a tree of connected modules, and allows for
 * computation of forward kinematics and jacobians.
 */
typedef struct _HebiKinematics* HebiKinematicsPtr;

/**
 * Contains a kinematic body, which represents a transform from an input to one
 * or more outputs.
 */
typedef struct _HebiBody* HebiBodyPtr;

/**
 * \brief The C-style's API representation of a log file.
 *
 * Represents a log file generated by the API.
 */
typedef struct _HebiLogFile* HebiLogFilePtr;

/**
 * Maintains a registry of network-connected modules and returns Group objects
 * to the user. Only one Lookup object is needed per application.
 *
 */
typedef struct _HebiLookup *HebiLookupPtr;

/**
 * A list of entries that represent a snapshot of the state of the lookup object
 * at some point in time.  These entries include network HEBI devices such
 * as actuators.
 */
typedef struct _HebiLookupEntryList *HebiLookupEntryListPtr;

/**
 * \brief The C-style's API representation of a trajectory.
 *
 * Do not inherit from this; only obtain pointers through the API!
 *
 * Used to generate position, velocity, and acceleration for different joints.
 */
typedef struct _HebiTrajectory* HebiTrajectoryPtr;

/**
 * \brief Group feedback handling function signature
 */
typedef void (*GroupFeedbackHandlerFunction)(HebiGroupFeedbackPtr fbk, void* user_data);

////////////////////////////////////////////////////////////////////////////////
// Structures
////////////////////////////////////////////////////////////////////////////////

typedef struct _HebiMacAddress {
  uint8_t bytes_[6];
} HebiMacAddress;

typedef struct _HebiVector3f {
  float x;
  float y;
  float z;
} HebiVector3f;

////////////////////////////////////////////////////////////////////////////////
// Kinematics Structures
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief A structure which stores the parameters for a 1-DOF rotary actuator
 */
typedef struct HebiKinematicParametersActuator
{
  float com[3];
  float input_to_joint[16];
  float joint_rotation_axis[3];
  float joint_to_output[16];
} HebiKinematicParametersActuator;

/**
 * \brief A structure which stores the parameters for a 1 output static body.
 */
typedef struct HebiKinematicParametersStaticBody
{
  float com[3];
  float output[16];
} HebiKinematicParametersStaticBody;

////////////////////////////////////////////////////////////////////////////////
// Lookup API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Create a Lookup instance.
 *
 * Lookup created by this function must be released with 'hebiLookupRelease'
 * when no longer needed.
 *
 * Note that this call invokes a background thread to query the network for
 * modules at regular intervals.
 */
HebiLookupPtr hebiLookupCreate(void);

/**
 * \brief Frees resources created by the lookup object.
 *
 * Lookup object should no longer be used after this function is called!
 * Note that background query thread is stopped by this function.
 */
void hebiLookupRelease(HebiLookupPtr lookup);

/**
 * \brief Return a snapshot of the contents of the module registry -- i.e.,
 * which modules have been found by the lookup.
 *
 * \param lookup A valid HebiLookup object.
 */
HebiLookupEntryListPtr hebiCreateLookupEntryList(HebiLookupPtr lookup);

/**
 * Gets the number of entries in the lookup entry list.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 */
int hebiLookupEntryListGetSize(HebiLookupEntryListPtr lookup_list);

/**
 * Gets the name of the given entry in the lookup entry list. Must be a valid
 * index.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * Note - assumes ASCII string encoding.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 * \param index The entry index that is being queried.
 * \param buffer An allocated buffer of length 'length'
 * \param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLookupEntryListGetName(HebiLookupEntryListPtr lookup_list, int index,
  char* buffer, size_t* length);

/**
 * Gets the family of the given entry in the lookup entry list. Must be a valid
 * index.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * Note - assumes ASCII string encoding.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 * \param index The entry index that is being queried.
 * \param buffer An allocated buffer of length 'length'.
 * \param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLookupEntryListGetFamily(HebiLookupEntryListPtr lookup_list, int index,
  char* buffer, size_t* length);

/**
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 * \param index The entry index that is being queried.
 */
HebiMacAddress hebiLookupEntryListGetMacAddress(HebiLookupEntryListPtr lookup_list, int index);

/**
 * \brief Release resources for a given lookup entry list; list should not be
 * used after this call.
 *
 * \param lookup_list A valid HebiLookupEntryList object.
 */
void hebiLookupEntryListRelease(HebiLookupEntryListPtr lookup_list);

////////////////////////////////////////////////////////////////////////////////
// Group API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates an "imitation" group with the specified number of modules.
 *
 * The imitation group is useful for testing the API, as it acts like a Group
 * would in most cases, but is not backed by hardware.  Commands that are
 * sent to the imitation group are returned as feedback, using the standard
 * feedback request methods.
 *
 * Note that standard groups are created through the HebiLookup objects.
 *
 * \param size The number of modules in the group.
 *
 * \returns An imitation group that returns commanded values as feedback.
 */
HebiGroupPtr hebiGroupCreateImitation(unsigned int size);

/**
 * \brief Create a group of modules with the given MAC addresses.
 *
 * If any given modules are not found, no group is created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param addresses An array of pointers to physical mac addresses of the given
 * modules. Length of the array must equal num_addresses.
 * \param num_addresses Length of the addresses array of pointers (number of
 * pointers in the array, not cumulative size of objects they point to).
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromMacs(HebiLookupPtr lookup, const HebiMacAddress* addresses,
  int num_addresses, long timeout_ms);

/**
 * \brief Create a group with modules matching the given names and families.
 *
 * If only one family is given, it is used for all modules.  Otherwise, number of
 * names and families must match. If any given modules are not found, no group is
 * created.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param names The given names of the modules, as viewable in the HEBI GUI. Must
 * be a list of pointers to null-terminated strings. The number of pointers must
 * match the num_names parameter.
 * \param num_names The number of pointers to null-terminated strings given
 * by the names parameter.
 * \param families The given families of the modules, as viewable in the HEBI
 * GUI. Must be a list of pointers to null-terminated strings. The number of
 * pointers must match the num_families parameter. Note that a single string
 * (with corresponding value of num_families == 1) will be used with each name in
 * the names list.
 * \param num_families The number of pointers to null-terminated strings given
 * by the families parameter. Note that this must either be 1, or be equal to
 * num_names.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromNames(HebiLookupPtr lookup, const char* const* families, int num_families,
  const char* const* names, int num_names, long timeout_ms);

/**
 * \brief Create a group with all modules known to the lookup with the given family.
 *
 * Group contains all modules with the given family, regardless of name.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param family The given family of the modules, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateFromFamily(HebiLookupPtr lookup, const char* family, long timeout_ms);

/**
 * \brief Create a group with all modules connected to module with the given MAC
 * address.
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param address Physical mac address of the given module (serves as unique id).
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateConnectedFromMac(HebiLookupPtr lookup, const HebiMacAddress* address,
  long timeout_ms);

/**
 * \brief Create a group with all modules connected to module with the given name
 * and family
 *
 * Modules in group will be ordered depth-first, starting with the most proximal
 * module.
 *
 * Blocking call which waits to get a pointer to a Group object with the
 * given parameters.  Times out after timeout_msec milliseconds. Must be
 * released when use is complete via the hebiGroupRelease function.
 *
 * \param lookup A valid HebiLookup object.
 * \param name The given name of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * \param family The given family of the key module, as viewable in the HEBI GUI.
 * Must be a null-terminated string.
 * \param timeout_ms Timeout in milliseconds.  A value of -1 blocks until
 * a module is found, and a value of 0 returns immediately if no module with
 * that address is currently known by the Lookup class.
 * \returns NULL if matching group not found in allotted time; pointer to newly
 * allocated group object otherwise.
 */
HebiGroupPtr hebiGroupCreateConnectedFromName(HebiLookupPtr lookup, const char* family, const char* name,
  long timeout_ms);

/**
 * \brief Returns the number of modules in a group.
 *
 * \param group The group to send this command to.
 *
 * \returns the number of modules in the group.
 */
int hebiGroupGetSize(HebiGroupPtr group);

/**
 * \brief Sends a command to the given group, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * Note: A non-HebiStatusSuccess return does not indicate a specific failure,
 * and may result from an error while sending or simply a timeout/dropped
 * response packet after a successful transmission.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns HebiStatusSuccess if an acknowledgement was successfully received (guaranteeing
 * the group received this command), or a failure code for an error otherwise.
 */
HebiStatusCode hebiGroupSendCommandWithAcknowledgement(HebiGroupPtr group, HebiGroupCommandPtr command,
  int timeout_ms);

/**
 * \brief Sends a command to the given group without requesting an
 * acknowledgement.
 *
 * Appropriate for high-frequency applications.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 *
 * \returns HebiStatusSuccess if the command was successfully sent, otherwise a failure code.
 */
HebiStatusCode hebiGroupSendCommand(HebiGroupPtr group, HebiGroupCommandPtr command);

/**
 * \brief Sets the command lifetime for the group, in milliseconds.
 *
 * The command lifetime is the duration for which a sent command remains active.
 * If the hardware does not receive further commands within the specified time
 * frame, all local controllers get deactivated. This is a safety feature to
 * mitigate the risk of accidents in case programs get interrupted in an unsafe
 * state, e.g., on program exceptions or during a network fault.
 *
 * Additionally, supporting hardware does not accept commands from any other
 * sources during the lifetime of a command. This mitigates the risk of other
 * users accidentally sending conflicting targets from, e.g., the GUI.
 *
 * \param group Which group the command lifetime is being set for.
 * \param lifetime_ms The number of milliseconds which the command 'lives' for.
 * Setting a value less than or equal to '0' disables command lifetime. When
 * disabled, the hardware will continue to execute the last sent command.
 * Setting a value above the accepted maximum will set the lockout to the
 * maximum value.
 *
 * \returns HebiStatusSuccess if command lifetime successfully set, or a failure code if
 * value was outside of accepted range (higher than supported maximum or negative).
 */
HebiStatusCode hebiGroupSetCommandLifetime(HebiGroupPtr group, int32_t lifetime_ms);

/**
 * \brief Returns the current command lifetime, in milliseconds.
 *
 * \param group Which group is being queried.
 *
 * \returns The current command lifetime, in milliseconds. A value of '0' indicates
 * that commands remain active until the next command is received.
 */
int32_t hebiGroupGetCommandLifetime(HebiGroupPtr group);

/**
 * \brief Sets the feedback request loop frequency (in Hz).
 *
 * The group is queried for feedback in a background thread at this frequency,
 * and any added callbacks are called from this background thread.
 *
 * \param group Which group this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns HebiStatusSuccess if feedback frequency successfully set, or a failure code if
 * value was outside of accepted range (higher than supported maximum, NaN or negative).
 */
HebiStatusCode hebiGroupSetFeedbackFrequencyHz(HebiGroupPtr group, float frequency);

/**
 * \brief Returns the current feedback request loop frequency (in Hz).
 *
 * \param group Which group is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiGroupGetFeedbackFrequencyHz(HebiGroupPtr group);

/**
 * \brief Add a function that is called whenever feedback is returned from the
 * group.
 *
 * \param group The group to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the group.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
void hebiGroupRegisterFeedbackHandler(HebiGroupPtr group, GroupFeedbackHandlerFunction handler,
  void* user_data);

/**
 * \brief Removes all feedback handling functions from the queue to be called on
 * receipt of group feedback.
 *
 * \param group The group to which the handlers are attached.
 */
void hebiGroupClearFeedbackHandlers(HebiGroupPtr group);

/**
 * \brief Requests feedback from the group.
 *
 * Sends a background request to the modules in the group; if/when all modules
 * return feedback, any associated handler functions are called. This returned
 * feedback is also stored to be returned by the next call to
 * hebiGroupGetNextFeedback (any previously returned data is discarded).
 *
 * \param group The group to return feedback from.
 *
 * \returns HebiStatusSuccess if request was successfully sent, or a failure code if not
 * (i.e., connection error).
 */
HebiStatusCode hebiGroupSendFeedbackRequest(HebiGroupPtr group);

/**
 * \brief Returns the most recently stored feedback from a sent feedback
 * request, or returns the next one received (up to the requested timeout).
 *
 * Note that a feedback request can be sent either with the
 * hebiGroupSendFeedbackRequest function, or by setting a background feedback
 * frequency with hebiGroupSetFeedbackFrequencyHz.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param group The group to return feedback from.
 * \param feedback On success, the feedback read from the group are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for feedback.
 * For typical networks, '15' ms is a value that can be reasonably expected to
 * allow for a round trip transmission after the last 'send feedback request'
 * call.
 *
 * \returns HebiStatusSuccess if feedback was returned, or a failure code if not
 * (i.e., connection error or timeout waiting for response).
 */
HebiStatusCode hebiGroupGetNextFeedback(HebiGroupPtr group, HebiGroupFeedbackPtr feedback, int timeout_ms);

/**
 * \brief Requests info from the group, and writes it to the provided info
 * object.
 *
 * Warning: other data in the provided 'Info' object is erased!
 *
 * \param group The group to send this command to.
 * \param info On success, the info read from the group is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns HebiStatusSuccess if info was received, or a failure code if not
 * (i.e., connection error or timeout waiting for response).
 */
HebiStatusCode hebiGroupRequestInfo(HebiGroupPtr group, HebiGroupInfoPtr info, int timeout_ms);

/**
 * \brief Starts logging data to a file in the local directory.
 *
 * WARNING: this function interface is not yet stable.
 *
 * \param group The group to log from.
 * \param dir The relative or absolute path to the directory to log in. To use
 * the current directory, pass in a null pointer
 * \param file The optional file name. If this is null, a name will be created using
 * the time at the moment which this function was invoked.
 *
 * \returns HebiStatusSuccess if successfully started a log, a failure code otherwise.
 */

HebiStatusCode hebiGroupStartLog(HebiGroupPtr group, const char* dir, const char* file);

/**
 * \brief Stops logging data to a file in the local directory.
 * Note: This function allocates a log file structure on the heap, so make sure to release the pointer
 * returned by this function by calling @c hebiLogFileRelease(HebiLogFilePtr ptr)
 *
 * \param group The group that is logging.
 *
 * \returns a log file instance on success, otherwise a null pointer.
 */
HebiLogFilePtr hebiGroupStopLog(HebiGroupPtr group);

/**
 * \brief Release resources for a given group; group should not be used after
 * this call.
 *
 * \param group A valid HebiGroup object.
 */
void hebiGroupRelease(HebiGroupPtr group);

/**
 * \brief Creates a GroupCommand for a group with the specified number of
 * modules.
 *
 * \param size The number of modules in the group.
 *
 * \returns A pointer to a new GroupCommand object. This must be released
 * with hebiGroupCommandRelease(HebiGroupCommandPtr).
 */
HebiGroupCommandPtr hebiGroupCommandCreate(int size);

/**
 * \brief Return the number of modules in this group Command.
 *
 * \returns The number of module commands in this group command.
 */
int hebiGroupCommandGetSize(HebiGroupCommandPtr cmd);

/**
 * \brief Import gains from a file into a GroupCommand object.
 */
HebiStatusCode hebiGroupCommandReadGains(HebiGroupCommandPtr cmd, const char* file);

/**
 * \brief Export gains from a GroupCommand object into a file.
 */
HebiStatusCode hebiGroupCommandWriteGains(HebiGroupCommandPtr cmd, const char* file);

/**
 * \brief Get an individual command for a particular module at index
 * @c module_index.
 *
 * \param module_index The index to retrieve the module command; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * \returns The command corresponding to the module at the specified index
 */
HebiCommandPtr hebiGroupCommandGetModuleCommand(HebiGroupCommandPtr cmd, int module_index);

/**
 * \brief Frees resources created by the GroupCommand object.
 *
 * The GroupCommandPtr must not be used after this function is called.
 */
void hebiGroupCommandRelease(HebiGroupCommandPtr cmd);

/**
 * \brief Creates a GroupFeedback for a group with the specified number of
 * modules.
 *
 * \param size The number of modules in the group.
 *
 * \returns A pointer to a new GroupFeedback object. This must be released
 * with hebiGroupFeedbackRelease(HebiGroupFeedbackPtr).
 */
HebiGroupFeedbackPtr hebiGroupFeedbackCreate(int size);

/**
 * \brief Return the number of modules in this group Feedback.
 *
 * \returns The number of module feedbacks in this group feedback.
 */
int hebiGroupFeedbackGetSize(HebiGroupFeedbackPtr fbk);

/**
 * \brief Get an individual feedback for a particular module at index
 * @c module_index.
 *
 * \param module_index The index to retrieve the module feedback; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * \returns The feedback corresponding to the module at the specified index.
 */
HebiFeedbackPtr hebiGroupFeedbackGetModuleFeedback(HebiGroupFeedbackPtr fbk, int module_index);

/**
 * \brief Frees resources created by the GroupFeedback object.
 *
 * The GroupFeedbackPtr should must not be used after this function is called.
 */
void hebiGroupFeedbackRelease(HebiGroupFeedbackPtr fbk);

/**
 * \brief Creates a GroupInfo for a group with the specified number of
 * modules.
 *
 * \param size The number of modules in the group.
 *
 * \returns A pointer to a new GroupInfo object. This must be released
 * with hebiGroupInfoRelease(HebiGroupInfoPtr).
 */
HebiGroupInfoPtr hebiGroupInfoCreate(int size);

/**
 * \brief Return the number of modules in this group Info.
 *
 * \returns The number of module infos in this group info.
 */
int hebiGroupInfoGetSize(HebiGroupInfoPtr info);

/**
 * \brief Export gains from a GroupInfo object into a file.
 */
HebiStatusCode hebiGroupInfoWriteGains(HebiGroupInfoPtr info, const char* file);

/**
 * \brief Get an individual info for a particular module at index
 * @c module_index.
 *
 * \param module_index The index to retrieve the module info; must be
 * from 0 to the size - 1, inclusive, or results in undefined behavior.
 *
 * \returns The info corresponding to the module at the specified index.
 */
HebiInfoPtr hebiGroupInfoGetModuleInfo(HebiGroupInfoPtr info, int module_index);

/**
 * \brief Frees resources created by the GroupInfo object.
 *
 * The GroupInfoPtr must not be used after this function is called.
 */
void hebiGroupInfoRelease(HebiGroupInfoPtr info);

////////////////////////////////////////////////////////////////////////////////
// Command API
////////////////////////////////////////////////////////////////////////////////

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetFloat(HebiCommandPtr cmd, HebiCommandFloatField field, float* value);

/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetFloat(HebiCommandPtr cmd, HebiCommandFloatField field, float* value);

/**
 * If the specified value is set, writes the value of the field to the pointers
 * (if both are not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetHighResAngle(HebiCommandPtr cmd, HebiCommandHighResAngleField field,
  int64_t* int_part, float* dec_part);

/**
 * Sets the given field. If any of the provided pointers are null, the field is cleared.
 */
void hebiCommandSetHighResAngle(HebiCommandPtr cmd, HebiCommandHighResAngleField field,
  int64_t* int_part, float* dec_part);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetNumberedFloat(HebiCommandPtr cmd, HebiCommandNumberedFloatField field,
  int number, float* value);

/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetNumberedFloat(HebiCommandPtr cmd, HebiCommandNumberedFloatField field,
  int number, float* value);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet. 
 * 
 * The value written to the pointer will be 1 for true
 * and 0 for false.
 */
HebiStatusCode hebiCommandGetBool(HebiCommandPtr cmd, HebiCommandBoolField field, int* value);

/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetBool(HebiCommandPtr cmd, HebiCommandBoolField field, int* value);

/**
 * Retrieves the value and/or the length of the string field.
 *
 * If this string is not set in the Command, HebiStatusValueNotSet is
 * returned (regardless of any other arguments) and any pointer values are
 * unchanged.
 *
 * If 'length' is NULL, returns HebiStatusValueSuccess if this string value is
 * set (the value of 'buffer' is ignored in this case).
 *
 * If 'buffer' is not NULL, then 'length' should be non-NULL and set to the
 * number of characters that can be written to 'buffer'. If the specified string
 * is set, and it can fit into the provided buffer (including a null terminating
 * character), then the string is copied to 'buffer' and HebiStatusSuccess is
 * returned.  If the buffer is not large enough to contain the string, then
 * HebiStatusBufferTooSmall is returned.
 *
 * If 'buffer' is NULL, returns HebiStatusValueSuccess if this string value is
 * set.
 *
 * If 'length' is not NULL (regardless of the state of 'buffer'), it is set to
 * the necessary size to hold the specified string value (including the null
 * terminating character).
 *
 * Note - assumes ASCII string encoding.
 */
HebiStatusCode hebiCommandGetString(HebiCommandPtr cmd, HebiCommandStringField field,
  char* buffer, size_t* length);

/**
 * Sets the given string to the value given in the buffer (if given). If any of
 * the provided pointers are null, the field is cleared.
 *
 * 'length' should be set to the length of the c-style string in 'buffer',
 * without including the terminating null character. The data in 'buffer' does
 * not need to be null terminated.
 *
 * Note - assumes ASCII string encoding.
 */
void hebiCommandSetString(HebiCommandPtr cmd, HebiCommandStringField field,
  const char* buffer, size_t* length);

/**
 * Checks whether this flag is set. Returns '1' for yes, '0' for no.
 */
int hebiCommandGetFlag(HebiCommandPtr cmd, HebiCommandFlagField field);

/**
 * Sets or clears a flag value. A nonzero value sets this flag and a value of zero clears this flag.
 */
void hebiCommandSetFlag(HebiCommandPtr cmd, HebiCommandFlagField field, int value);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetEnum(HebiCommandPtr cmd, HebiCommandEnumField field, int* value);

/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetEnum(HebiCommandPtr cmd, HebiCommandEnumField field, int* value);

/**
 * If the indicated pin has an integer value, writes it to the pointer (if not
 * NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetIoPinInt(HebiCommandPtr cmd, HebiCommandIoPinBank field,
  unsigned int pin_number, int64_t* value);

/**
 * If the indicated pin has an floating point value, writes it to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetIoPinFloat(HebiCommandPtr cmd, HebiCommandIoPinBank field,
  unsigned int pin_number, float* value);

/**
 * Sets the indicated pin to this integer value. If the provided pointer is NULL
 * the field is cleared (of values of any type).
 */
void hebiCommandSetIoPinInt(HebiCommandPtr cmd, HebiCommandIoPinBank field,
  unsigned int pin_number, int64_t* value);

/**
 * Sets the indicated pin to this floating point value. If the provided pointer
 * is NULL the field is cleared (of values of any type).
 */
void hebiCommandSetIoPinFloat(HebiCommandPtr cmd, HebiCommandIoPinBank field,
  unsigned int pin_number, float* value);

/**
 * If the led color is set, writes it to the three output integer pointers (if
 * all are not NULL), each 0-255, and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 *
 * For command-style messages, this refers to the color to override the module's
 * default control of the LED.
 */
HebiStatusCode hebiCommandGetLedColor(HebiCommandPtr cmd, HebiCommandLedField field,
  uint8_t* r, uint8_t* g, uint8_t* b);

/**
 * Returns '1' if this message indicates that the module should resume control of the LED.  A '0' can indicate either
 * an override command (if and only if HasLedColor() returns '1'), or no information about the LED (i.e., the module
 * should maintain it's current state regarding the LED).
 */
int hebiCommandHasLedModuleControl(HebiCommandPtr cmd, HebiCommandLedField field);

/**
 * Commands a color that overrides the module's control of the LED.
 */
void hebiCommandSetLedOverrideColor(HebiCommandPtr cmd, HebiCommandLedField field,
  uint8_t r, uint8_t g, uint8_t b);

/**
 * Sets the module to regain control of the LED.
 */
void hebiCommandSetLedModuleControl(HebiCommandPtr cmd, HebiCommandLedField field);

/**
 * Clears the given LED field, so that the module maintains its previous state
 * of LED control/color (i.e., does not have an override color command or an explicit 'module control' command).
 */
void hebiCommandClearLed(HebiCommandPtr cmd, HebiCommandLedField field);

////////////////////////////////////////////////////////////////////////////////
// Feedback API
////////////////////////////////////////////////////////////////////////////////

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetFloat(HebiFeedbackPtr fbk, HebiFeedbackFloatField field, float* value);

/**
 * If the specified value is set, writes the value of the field to the pointers
 * (if both are not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetHighResAngle(HebiFeedbackPtr fbk, HebiFeedbackHighResAngleField field,
  int64_t* int_part, float* dec_part);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetNumberedFloat(HebiFeedbackPtr fbk, HebiFeedbackNumberedFloatField field,
  int number, float* value);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetUInt64(HebiFeedbackPtr fbk, HebiFeedbackUInt64Field field, uint64_t* value);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetVector3f(HebiFeedbackPtr fbk,
  HebiFeedbackVector3fField field, HebiVector3f* value);

/**
 * If the indicated pin has an integer value, writes it to the pointer (if not
 * NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetIoPinInt(HebiFeedbackPtr fbk, HebiFeedbackIoPinBank field,
  unsigned int pin_number, int64_t* value);

/**
 * If the indicated pin has an floating point value, writes it to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetIoPinFloat(HebiFeedbackPtr fbk, HebiFeedbackIoPinBank field,
  unsigned int pin_number, float* value);

/**
 * If the led color is set, writes it to the three output integer pointers (if
 * all are not NULL), each 0-255, and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 *
 * For command-style messages, this refers to the color to override the module's
 * default control of the LED.
 */
HebiStatusCode hebiFeedbackGetLedColor(HebiFeedbackPtr fbk, HebiFeedbackLedField field,
  uint8_t* r, uint8_t* g, uint8_t* b);

////////////////////////////////////////////////////////////////////////////////
// Info API
////////////////////////////////////////////////////////////////////////////////

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiInfoGetFloat(HebiInfoPtr info, HebiInfoFloatField field, float* value);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 *
 * The value written to the pointer will be 1 for true
 * and 0 for false.
 */
HebiStatusCode hebiInfoGetBool(HebiInfoPtr info, HebiInfoBoolField field, int* value);

/**
 * Retrieves the value and/or the length of the string field.
 *
 * If this string is not set in the Info, HebiStatusValueNotSet is
 * returned (regardless of any other arguments) and any pointer values are
 * unchanged.
 *
 * If 'length' is NULL, returns HebiStatusValueSuccess if this string value is
 * set (the value of 'buffer' is ignored in this case).
 *
 * If 'buffer' is not NULL, then 'length' should be non-NULL and set to the
 * number of characters that can be written to 'buffer'. If the specified string
 * is set, and it can fit into the provided buffer (including a null terminating
 * character), then the string is copied to 'buffer' and HebiStatusSuccess is
 * returned.  If the buffer is not large enough to contain the string, then
 * HebiStatusBufferTooSmall is returned.
 *
 * If 'buffer' is NULL, returns HebiStatusValueSuccess if this string value is
 * set.
 *
 * If 'length' is not NULL (regardless of the state of 'buffer'), it is set to
 * the necessary size to hold the specified string value (including the null
 * terminating character).
 *
 * Note - assumes ASCII string encoding.
 */
HebiStatusCode hebiInfoGetString(HebiInfoPtr info, HebiInfoStringField field, char* buffer, size_t* length);

/**
 * Checks whether this flag is set. Returns '1' for yes, '0' for no.
 */
int hebiInfoGetFlag(HebiInfoPtr info, HebiInfoFlagField field);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 */
HebiStatusCode hebiInfoGetEnum(HebiInfoPtr info, HebiInfoEnumField field, int* value);

/**
 * If the led color is set, writes it to the three output integer pointers (if
 * all are not NULL), each 0-255, and returns HebiStatusSuccess. Otherwise, returns HebiStatusValueNotSet.
 *
 * For command-style messages, this refers to the color to override the module's
 * default control of the LED.
 */
HebiStatusCode hebiInfoGetLedColor(HebiInfoPtr info, HebiInfoLedField field,
  uint8_t* r, uint8_t* g, uint8_t* b);

////////////////////////////////////////////////////////////////////////////////
// Kinematics API
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
HebiBodyPtr hebiBodyCreateActuator(const float* com, const float* input_to_joint,
  const float* joint_rotation_axis, const float* joint_to_output);

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
void hebiBodyRelease(HebiBodyPtr body);

/**
 * \brief Creates an object to hold a kinematic tree. This structure has a
 * single output available at the origin.
 *
 * Kinematics object created by this function must be released with
 * 'hebiKinematicsRelease' when no longer needed.
 */
HebiKinematicsPtr hebiKinematicsCreate(void);

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
 * has already been attached to the kinematic tree. That body should be passed
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
HebiStatusCode hebiKinematicsAddBody(HebiKinematicsPtr kinematics, HebiBodyPtr existing_body,
  int output_index, HebiBodyPtr new_body);

/**
 * \brief Generates the forward kinematics for the given kinematic tree.
 *
 * The order of the returned frames is in a depth-first tree. As an example,
 * assume a body A has one output, to which body B is connected to. Body B has
 * two outputs; actuator C is attached to the first output and actuator E is
 * attached to the second output. Body D is attached to the only output of
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
void hebiKinematicsGetForwardKinematics(HebiKinematicsPtr kinematics, HebiFrameType frame_type,
  const double* positions, float* frames);

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
void hebiKinematicsGetJacobians(HebiKinematicsPtr kinematics, HebiFrameType frame_type,
  const double* positions, float* jacobians);

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
void hebiKinematicsGetEndEffector(HebiKinematicsPtr kinematics, HebiFrameType frame_type,
  const double* positions, float* transforms);

/**
 * \brief Frees resources created by this kinematics object.
 *
 * Kinematics object should no longer be used after this function is called!
 *
 * \param kinematics A valid HEBI Kinematics object.
 */
void hebiKinematicsRelease(HebiKinematicsPtr kinematics);

// Some geometric/transform related helper functions for kinematics:

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

// Helper functions for returning parameters for specific kinematic bodies.

/**
 * Sets the input parameter to the kinematic parameters for an X5-series
 * actuator.
 *
 * \returns HebiStatusSuccess on success or HebiStatusInvalidArgument (e.g. null pointer or other invalid arguments).
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
 * \returns HebiStatusSuccess on success or HebiStatusInvalidArgument (e.g. null pointer or other invalid arguments).
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
 * \returns HebiStatusSuccess on success or HebiStatusInvalidArgument (e.g. null pointer or other invalid arguments).
 */
static HebiStatusCode hebiKinematicParametersX5LightBracket(
  HebiKinematicParametersStaticBody* params, HebiMountingType mounting)
{
  // Ignore null pointers and invalid parameters
  if (!params)
    return HebiStatusInvalidArgument;
  if (mounting != HebiMountingTypeLeft && mounting != HebiMountingTypeRight)
    return HebiStatusInvalidArgument;

  float mult = 1;
  if (mounting == HebiMountingTypeRight)
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
 * \returns HebiStatusSuccess on success or HebiStatusInvalidArgument (e.g. null pointer or other invalid arguments).
 */
static HebiStatusCode hebiKinematicParametersX5HeavyBracket(
  HebiKinematicParametersStaticBody* params, HebiMountingType mounting)
{
  // Ignore null pointers and invalid parameters
  if (!params)
    return HebiStatusInvalidArgument;
  if (mounting != HebiMountingTypeLeftInside &&
      mounting != HebiMountingTypeLeftOutside &&
      mounting != HebiMountingTypeRightInside &&
      mounting != HebiMountingTypeRightOutside)
    return HebiStatusInvalidArgument;

  float lr_mult = 1;
  if (mounting == HebiMountingTypeRightInside || mounting == HebiMountingTypeRightOutside)
    lr_mult = -1; 

  float y_dist = -0.0225f; // Inside
  if (mounting == HebiMountingTypeLeftOutside || mounting == HebiMountingTypeRightOutside)
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

////////////////////////////////////////////////////////////////////////////////
// Inverse Kinematics API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates an IK (inverse kinematics) object that allows for solving for
 * joint angles/positions given objectives and constraints.
 *
 * This optimization is completed using a specified forward kinematics object.
 * The objectives and constraints are stored with this object.
 */
HebiIKPtr hebiIKCreate(void);

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
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g. incompatible with
 * existing objectives, or all components are set to 'NaN')
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorPosition(HebiIKPtr ik,
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
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g. incompatible with
 * existing objectives, or rotation matrix is invalid.)
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorSO3(HebiIKPtr ik,
  float weight, const float* matrix);

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
 * \return HebiStatusSuccess on success, otherwise a failure code (e.g. incompatible with
 * existing objectives, or rotation matrix is invalid.)
 */
HebiStatusCode hebiIKAddObjectiveEndEffectorTipAxis(HebiIKPtr ik,
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
HebiStatusCode hebiIKAddConstraintJointAngles(HebiIKPtr ik,
  float weight, int num_joints, const double* min_positions, const double* max_positions);

/**
 * \brief Clears the objectives and constraints from this IK object, along
 * with any modifications to the default algorithm parameters.
 */
void hebiIKClearAll(HebiIKPtr ik);

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

////////////////////////////////////////////////////////////////////////////////
// Trajectory API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Creates a HebiTrajectory object for a single joint using the given
 * parameters; this must be released with hebiTrajectoryRelease after use.
 *
 * \param num_waypoints The number of waypoints.
 * \param positions A vector of waypoints for this joint; should be
 * num_waypoints in length. Any elements that are NAN will be considered free
 * parameters, and will be set by the function. Values of +/-infinity are not
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
 * be defined (not NULL, and not NAN for any element). The first element must
 * be zero.
 *
 * \returns A HebiTrajectory object if there were no errors, and the trajectory
 * has been created. A NULL value indicates that there was an error, but does
 * not specify any details about the error at this time.
 */
HebiTrajectoryPtr hebiTrajectoryCreateUnconstrainedQp(int num_waypoints, double* positions,
  double* velocities, double* accelerations, double* time_vector);

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
HebiStatusCode hebiTrajectoryGetState(HebiTrajectoryPtr trajectory, double time, double* position,
  double* velocity, double* acceleration);

////////////////////////////////////////////////////////////////////////////////
// Logging API
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Releases a log file instance
 */
void hebiLogFileRelease(HebiLogFilePtr log_file);

/**
 * \brief Retrieves the name and path of the log file.
 *
 * To only query the length of the string, provide a null pointer for the buffer parameter.
 * If the provided buffer is not large enough to hold the string (the length determined by the length parameter),
 * the call will fail. Note that the size of this buffer includes the null
 * terminating character.
 *
 * \param buffer buffer into which the string will be copied. This string will be null terminated.
 * \param length the length of the provided buffer. After calling this function, the value dereferenced will be
 * updated with the length of the string plus the null character.
 *
 * \returns HebiStatusSuccess on success, HebiStatusBufferTooSmall if the provided buffer is too small, or
 * HebiStatusInvalidArgument if the length parameter is null
 */
HebiStatusCode hebiLogFileGetFileName(HebiLogFilePtr log_file, char* buffer, size_t* length);

/**
 * \brief Opens an existing log file
 *
 * Note: It is up to the user to check if the returned pointer is null. If the file does not exist,
 * or if the file is not a valid log file, this function returns null.
 *
 * If this function returns a pointer, you must call @c hebiLogFileRelease(HebiLogFilePtr)
 * to release the allocated memory.
 *
 * \param file the directory and path of the file to open
 * \return a pointer to the file;  null if the file could not be opened
 */
HebiLogFilePtr hebiLogFileOpen(const char* file);

/**
 * \brief Retrieve the number of modules in the group represented by an opened log file
 *
 * \return The number of modules in the group
 */
int hebiLogFileGetNumberOfModules(HebiLogFilePtr log_file);

/**
 * \brief Retrieve the next group feedback from the opened log file
 *
 * \param feedback the feedback object into which the contents will be copied
 * \return HebiStatusSuccess on success, otherwise HebiStatusFailure
 */
HebiStatusCode hebiLogFileGetNextFeedback(HebiLogFilePtr log_file, HebiGroupFeedbackPtr field);

////////////////////////////////////////////////////////////////////////////////
// Misc Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Gets the version for the library
 */
void hebiGetLibraryVersion(int* major, int* minor, int* revision);

/**
 * \brief Frees all resources created by the library.  Note: any calls to the
 * HEBI library functions after this will result in undefined behavior!
 */
void hebiCleanup(void);

#ifdef __cplusplus
} // extern "C"
#endif
