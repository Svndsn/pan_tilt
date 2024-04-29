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

extern void vController_init();
extern void vController_task();

#endif // _PID_H
