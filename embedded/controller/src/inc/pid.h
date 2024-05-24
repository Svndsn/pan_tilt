#ifndef _PID_H
#define _PID_H
#include "emp_type.h"

typedef struct {
  // PID controller constants
  FP32 Kp, Ki, Kd;
  // Sampling time
  FP32 T;
  // Motor voltage limits
  FP32 maxLimit, minLimit;
  // Minimum angle step
  FP32 angleStep;
  // Motor offset voltage where motor start moving (feed forward)
  FP32 offsetVoltage;
  // Measurement
  FP32 measurement;
  // Setpoint
  FP32 setpoint;
  // Prev
  FP32 prevSetpoint;
  // Output
  FP32 output;
  // Prev output (for anti-windup)
  FP32 prevOutput;
  // Error
  FP32 error;
  // Integrator
  FP32 integrator;
  // Previous error (for trapzoidal integration and derivative)
  FP32 prevError;
} PID_t;

extern void vControllerInit();
extern void vControllerTask();

#endif // _PID_H
