#include "pid.h"
#include "FreeRTOS.h"
#include "projectdefs.h"
#include "queue.h"
#include "spi1.h"
#include "task.h"
#include "uart0.h"

// UART queues
extern xQueueHandle q_uartDebug;
extern xQueueHandle q_uartAngle;
extern xQueueHandle q_uartSetpoint;

// SPI1 queues
extern xQueueHandle q_spiDutyCycle;
extern xQueueHandle q_spiAngle;

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
  pidPan.setpoint = 0.f;
  pidPan.prevSetpoint = 0.f;
  pidPan.measurement = 0.f;

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
  pidTilt.setpoint = 0.f;
  pidTilt.prevSetpoint = 0.f;
  pidTilt.measurement = 0.f;
}

void vUpdateController(PID_t *pid) {
  pid->error = pid->setpoint - pid->measurement;

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
  FP32 derivative = (pid->error - pid->prevError) / pid->T - (pid->setpoint - pid->prevSetpoint) / pid->T;
  // Derivative term
  FP32 Dout = pid->Kd * derivative;

  // Calculate total output
  pid->output = Pout + Iout + Dout;

  // Update previous values
  pid->prevOutput = pid->output; // Done before limiting (use for anti-windup)
  pid->prevError = pid->error;
  pid->prevSetpoint = pid->setpoint;

  // Limit output
  if (pid->output > pid->maxLimit) {
    pid->output = pid->maxLimit;
  } else if (pid->output < pid->minLimit) {
    pid->output = pid->minLimit;
  }
  // TODO: Maybe add acceleration limits
  // To avoid the belt slipping/breaking
}

void vUpdateSetpoints() {
  if (!uxQueueMessagesWaiting(q_uartSetpoint)) { return; }

  uartAngle_t setpoint;
  while (xQueueReceive(q_uartSetpoint, &setpoint, 0)) {
    // Setpoint received
    if (setpoint.axis == PAN) {
      if (setpoint.relative) {
        pidPan.setpoint += setpoint.angle;
      } else {
        pidPan.setpoint = setpoint.angle;
      }
    } else {
      if (setpoint.relative) {
        pidTilt.setpoint += setpoint.angle;
      } else {
        pidTilt.setpoint = setpoint.angle;
      }
    }
  }
  // TODO: Add limits to setpoints
}

void vReceiveAngles() {
  if(!uxQueueMessagesWaiting(q_spiAngle)) { return; }

  spiAngle_t angle;
  while (xQueueReceive(q_spiAngle, &angle, 0)) {
    if (angle.axis == PAN) {
      pidPan.measurement = angle.angle;
    } else {
      pidTilt.measurement = angle.angle;
    }
  }
}

void vSendAngles() {
  uartAngle_t panAngle = {PAN, pidPan.measurement, FALSE};
  uartAngle_t tiltAngle = {TILT, pidTilt.measurement, FALSE};
  xQueueSendToBack(q_uartAngle, &panAngle, 0);
  xQueueSendToBack(q_uartAngle, &tiltAngle, 0);
}

void vSendDutyCycles() {
  spiDutyCycle_t panDutyCycle = {PAN, pidPan.output};
  spiDutyCycle_t tiltDutyCycle = {TILT, pidTilt.output};
  xQueueSendToBack(q_spiDutyCycle, &panDutyCycle, 0);
  xQueueSendToBack(q_spiDutyCycle, &tiltDutyCycle, 0);
}

void vControllerTask() {
  // Initialize the controller variables
  vControllerInit();

  // Task loop
  while (1) {
    // Get current tick for correct timing
    portTickType xLastWakeTime = xTaskGetTickCount();

    // Get the latest setpoint (take all values from q_uartSetpoint)
    vUpdateSetpoints();

    // Get the latest measurement (take all values from q_spiAngle)
    vReceiveAngles();

    // Send the latest measurements to the computer (q_uartAngle)
    vSendAngles();

    // Update controller (PID)
    vUpdateController(&pidPan);
    vUpdateController(&pidTilt);

    // Send the latest output to the motor (q_spiDutyCycle)
    vSendDutyCycles();

    // Make sure the task runs at 100Hz
    vTaskDelayUntil(&xLastWakeTime, CONTROLLER_PERIOD_MS / portTICK_RATE_MS);
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
