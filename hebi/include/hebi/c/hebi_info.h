/* This file has been automatically generated. Do not edit by hand. */

#pragma once

#include "hebi_status_codes.h"
#include "stdint.h"
#include "stddef.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiInfo* HebiInfoPtr;

// Define all fields
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

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiInfoGetFloat(HebiInfoPtr, HebiInfoFloatField, float*);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.  The value written to the pointer will be 1 for true
 * and 0 for false.
 */
HebiStatusCode hebiInfoGetBool(HebiInfoPtr, HebiInfoBoolField, int*);

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
HebiStatusCode hebiInfoGetString(HebiInfoPtr, HebiInfoStringField, char* buffer, size_t* length);

/**
 * Checks whether this flag is set. Returns '1' for yes, '0' for no.
 */
int hebiInfoGetFlag(HebiInfoPtr, HebiInfoFlagField);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiInfoGetEnum(HebiInfoPtr, HebiInfoEnumField, int*);

/**
 * If the led color is set, writes it to the three output integer pointers (if
 * all are not NULL), each 0-255, and returns HebiStatusSuccess.  Otherwise,
 * returns HebiStatusValueNotSet.
 * For command-style messages, this refers to the color to override the module's
 * default control of the LED.
 */
HebiStatusCode hebiInfoGetLedColor(HebiInfoPtr, HebiInfoLedField, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif
