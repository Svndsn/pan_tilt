#include "pid.h"
#include "FreeRTOS.h"
#include "projectdefs.h"
#include "queue.h"
#include "task.h"

PID_t pidPan;
PID_t pidTilt;

void vControllerInit() {
  // Pan PID
  pidPan.Kp = 0.1f;
  pidPan.Ki = 0.f;
  pidPan.Kd = 0.f;

  pidPan.T = 0.01f; // 100Hz

  pidPan.maxLimit = 12.f;  // 12 volts
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

  pidTilt.maxLimit = 12.f;  // 12 volts
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

void vControllerTask() {
  vControllerInit();
  while (1) {
    // Get current tick for correct timing
    portTickType xLastWakeTime = xTaskGetTickCount();

    // Get the latest setpoint (take all values from q_uartSetpoint)
    // Get the latest measurement (take all values from q_spiAngle)
    // Send the latest measurements to the computer (q_uartAngle)

    // Update controller (PID)

    // Send the latest output to the motor (q_spiDutyCycle)

    // Make sure the task runs at 100Hz
    vTaskDelayUntil(&xLastWakeTime, CONTROLLER_PERIOD_MS / portTICK_RATE_MS);
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
