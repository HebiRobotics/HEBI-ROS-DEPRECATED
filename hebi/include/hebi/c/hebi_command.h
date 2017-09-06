/* This file has been automatically generated. Do not edit by hand. */

#pragma once

#include "hebi_status_codes.h"
#include "stdint.h"
#include "stddef.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiCommand* HebiCommandPtr;

// Define all fields
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

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetFloat(HebiCommandPtr, HebiCommandFloatField, float*);
/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetFloat(HebiCommandPtr, HebiCommandFloatField, float*);

/**
 * If the specified value is set, writes the value of the field to the pointers
 * (if both are not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetHighResAngle(HebiCommandPtr, HebiCommandHighResAngleField, int64_t* int_part, float* dec_part);
/**
 * Sets the given field. If any of the provided pointers are null, the field is cleared.
 */
void hebiCommandSetHighResAngle(HebiCommandPtr, HebiCommandHighResAngleField, int64_t* int_part, float* dec_part);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetNumberedFloat(HebiCommandPtr, HebiCommandNumberedFloatField, int, float*);
/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetNumberedFloat(HebiCommandPtr, HebiCommandNumberedFloatField, int, float*);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.  The value written to the pointer will be 1 for true
 * and 0 for false.
 */
HebiStatusCode hebiCommandGetBool(HebiCommandPtr, HebiCommandBoolField, int*);
/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetBool(HebiCommandPtr, HebiCommandBoolField, int*);

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
HebiStatusCode hebiCommandGetString(HebiCommandPtr, HebiCommandStringField, char* buffer, size_t* length);
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
void hebiCommandSetString(HebiCommandPtr, HebiCommandStringField, const char* buffer, size_t* length);

/**
 * Checks whether this flag is set. Returns '1' for yes, '0' for no.
 */
int hebiCommandGetFlag(HebiCommandPtr, HebiCommandFlagField);
/**
 * Sets or clears a flag value. A value of '1' sets this flag and a value of '0'
 * clears this flag.
 */
void hebiCommandSetFlag(HebiCommandPtr, HebiCommandFlagField, int);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetEnum(HebiCommandPtr, HebiCommandEnumField, int*);
/**
 * Sets the given field. If the provided pointer is null, the field is cleared.
 */
void hebiCommandSetEnum(HebiCommandPtr, HebiCommandEnumField, int*);

/**
 * If the indicated pin has an integer value, writes it to the pointer (if not
 * NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetIoPinInt(HebiCommandPtr, HebiCommandIoPinBank, unsigned int pin_number, int64_t*);
/**
 * If the indicated pin has an floating point value, writes it to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiCommandGetIoPinFloat(HebiCommandPtr, HebiCommandIoPinBank, unsigned int pin_number, float*);
/**
 * Sets the indicated pin to this integer value. If the provided pointer is NULL
 * the field is cleared (of values of any type).
 */
void hebiCommandSetIoPinInt(HebiCommandPtr, HebiCommandIoPinBank, unsigned int pin_number, int64_t* value);
/**
 * Sets the indicated pin to this floating point value. If the provided pointer
 * is NULL the field is cleared (of values of any type).
 */
void hebiCommandSetIoPinFloat(HebiCommandPtr, HebiCommandIoPinBank, unsigned int pin_number, float* value);

/**
 * If the led color is set, writes it to the three output integer pointers (if
 * all are not NULL), each 0-255, and returns HebiStatusSuccess.  Otherwise,
 * returns HebiStatusValueNotSet.
 * For command-style messages, this refers to the color to override the module's
 * default control of the LED.
 */
HebiStatusCode hebiCommandGetLedColor(HebiCommandPtr, HebiCommandLedField, uint8_t *r, uint8_t *g, uint8_t *b);
/**
 * Returns '1' if this message indicates that the module should resume control of the LED.  A '0' can indicate either
 * an override command (if and only if HasLedColor() returns '1'), or no information about the LED (i.e., the module
 * should maintain it's current state regarding the LED).
 */
int hebiCommandHasLedModuleControl(HebiCommandPtr, HebiCommandLedField);
/**
 * Commands a color that overrides the module's control of the LED.
 */
void hebiCommandSetLedOverrideColor(HebiCommandPtr, HebiCommandLedField, uint8_t r, uint8_t g, uint8_t b);
/**
 * Sets the module to regain control of the LED.
 */
void hebiCommandSetLedModuleControl(HebiCommandPtr, HebiCommandLedField);
/**
 * Clears the given LED field, so that the module maintains its previous state of LED control/color (i.e., does not have an override color command or an explicit 'module control' command).
 */
void hebiCommandClearLed(HebiCommandPtr, HebiCommandLedField);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif
