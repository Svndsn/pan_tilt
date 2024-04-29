#include "FreeRTOS.h"
#include "pid.h"
#include "task.h"
#include "queue.h"
#include "projectdefs.h"

PID_t pidPan;
PID_t pidTilt;

void vController_init() {
  // Pan PID
  pidPan.Kp = 0.1f;
  pidPan.Ki = 0.f;
  pidPan.Kd = 0.f;

  pidPan.T = 0.01f; // 100Hz

  pidPan.maxLimit = 12.f; // 12 volts
  pidPan.minLimit = -12.f; // -12 volts

  pidPan.prevOutput = 0.f;
  pidPan.prevError = 0.f;
  pidPan.integrator = 0.f;
  pidPan.output = 0.f;

  // Tilt PID
  pidTilt.Kp = 0.1f;
  pidTilt.Ki = 0.f;
  pidTilt.Kd = 0.f;

  pidTilt.T = 0.01f; // 100Hz

  pidTilt.maxLimit = 12.f; // 12 volts
  pidTilt.minLimit = -12.f; // -12 volts

  pidTilt.prevOutput = 0.f;
  pidTilt.prevError = 0.f;
  pidTilt.integrator = 0.f;
  pidTilt.output = 0.f;
}

void vUpdateController(PID_t *pid, FP32 setPoint, FP32 measurement) {
  pid->error = setPoint - measurement;

  // Proportional term
  FP32 Pout = pid->Kp * pid->error;

  // Anti-windup 
  if (pid->prevOutput <= pid->maxLimit && pid->prevOutput >= pid->minLimit) {
    // Only integrate if the last output is within limits
    // Trapezoidal integration
    pid->integrator += (pid->T / 2.f) * (pid->error + pid->prevError);
  }
  // Integral term
  FP32 Iout = pid->Ki * pid->integrator;

  // Derivative
  FP32 derivative = (pid->error - pid->prevError) / pid->T;
  // Derivative term
  FP32 Dout = pid->Kd * derivative;

  // Calculate total output
  pid->output = Pout + Iout + Dout;

  // Update previous values
  pid->prevOutput = pid->output; // Done before limiting (use for anti-windup)
  pid->prevError = pid->error;

  // Limit output
  if (pid->output > pid->maxLimit) {
    pid->output = pid->maxLimit;
  } else if (pid->output < pid->minLimit) {
    pid->output = pid->minLimit;
  }
}

void vController_task(){
  vController_init();
  while(1){
    // Get current tick for correct timing
    portTickType xLastWakeTime = xTaskGetTickCount();


    // Get the latest setpoint (uart0)
    // Get the latest measurement (SPI0)
    // Send the latest measurements to the computer (uart0)


    // Update controller (PID)


    // Send the latest output to the motor (SPI0)


    // Make sure the task runs at 100Hz
    vTaskDelayUntil(&xLastWakeTime, CONTROLLER_PERIOD_TICKS);
  }
}
