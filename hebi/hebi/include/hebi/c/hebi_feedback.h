/* This file has been automatically generated. Do not edit by hand. */

#pragma once

#include "hebi_status_codes.h"
#include "stdint.h"
#include "stddef.h"
#include "hebi_vector_3_f.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiFeedback* HebiFeedbackPtr;

// Define all fields
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

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetFloat(HebiFeedbackPtr, HebiFeedbackFloatField, float*);

/**
 * If the specified value is set, writes the value of the field to the pointers
 * (if both are not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetHighResAngle(HebiFeedbackPtr, HebiFeedbackHighResAngleField, int64_t* int_part, float* dec_part);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetNumberedFloat(HebiFeedbackPtr, HebiFeedbackNumberedFloatField, int, float*);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetUInt64(HebiFeedbackPtr, HebiFeedbackUInt64Field, uint64_t*);

/**
 * If the specified value is set, writes the value of the field to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetVector3f(HebiFeedbackPtr, HebiFeedbackVector3fField, HebiVector3f*);

/**
 * If the indicated pin has an integer value, writes it to the pointer (if not
 * NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetIoPinInt(HebiFeedbackPtr, HebiFeedbackIoPinBank, unsigned int pin_number, int64_t*);
/**
 * If the indicated pin has an floating point value, writes it to the pointer
 * (if not NULL), and returns HebiStatusSuccess.  Otherwise, returns
 * HebiStatusValueNotSet.
 */
HebiStatusCode hebiFeedbackGetIoPinFloat(HebiFeedbackPtr, HebiFeedbackIoPinBank, unsigned int pin_number, float*);

/**
 * If the led color is set, writes it to the three output integer pointers (if
 * all are not NULL), each 0-255, and returns HebiStatusSuccess.  Otherwise,
 * returns HebiStatusValueNotSet.
 * For command-style messages, this refers to the color to override the module's
 * default control of the LED.
 */
HebiStatusCode hebiFeedbackGetLedColor(HebiFeedbackPtr, HebiFeedbackLedField, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif
