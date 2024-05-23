#include "FreeRTOS.h"
#include "emp_type.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pid.h"
#include "projectdefs.h"
#include "spi1.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define PAN_ANGLE_MIN -180
#define PAN_ANGLE_MAX 180
#define TILT_ANGLE_MIN -180
#define TILT_ANGLE_MAX 180
#define VOLTAGE_TO_DUTY 85.25f

// UART0 queues
extern xQueueHandle q_uartDebug;
extern xQueueHandle q_uartAngle; // Send current angle
extern xQueueHandle q_uartSetpoint; // Receive setpoint
extern xQueueHandle q_uartVoltage; // Send motor voltages
// UART0 Mutex
extern xSemaphoreHandle m_uartDebug;
extern xSemaphoreHandle m_uartAngle;
extern xSemaphoreHandle m_uartSetpoint;
extern xSemaphoreHandle m_uartVoltage;

// SPI1 queues
extern xQueueHandle q_spiDutyCycle; // Send duty cycle
extern xQueueHandle q_spiAngle; // Receive angle
// SPI1 Mutex
extern xSemaphoreHandle m_spiDutyCycle;
extern xSemaphoreHandle m_spiAngle;


static PID_t pidPan;
static PID_t pidTilt;

void vControllerInit() {
  // Pan PID
  pidPan.Kp = 0.05f;
  pidPan.Kd = 0.01f;
  pidPan.Ki = 0.f;

  pidPan.T = 0.01f; // 100Hz

  pidPan.maxLimit = 12.f;  // 12 volts
  pidPan.minLimit = -12.f; // -12 volts
  pidPan.offsetVoltage = 3.f;

  pidPan.prevOutput = 0.f;
  pidPan.prevError = 0.f;
  pidPan.integrator = 0.f;
  pidPan.output = 0.f;
  pidPan.setpoint = 0.f;
  pidPan.prevSetpoint = 0.f;
  pidPan.measurement = 0.f;

  // Tilt PID
  pidTilt.Kp = 0.15f;
  pidTilt.Kd = 0.01f;
  pidTilt.Ki = 0.f;

  pidTilt.T = 0.01f; // 100Hz

  pidTilt.maxLimit = 12.f;  // 12 volts
  pidTilt.minLimit = -12.f; // -12 volts
  pidTilt.offsetVoltage = 3.f;

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
  if (pid->error < 0.7f && pid->error > -0.7f) {
    pid->error = 0;
  }

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

  // Derivative (remove setpoint step)
  FP32 derivative = (pid->error - pid->prevError) / pid->T - (pid->setpoint - pid->prevSetpoint) / pid->T;
  // Derivative term
  FP32 Dout = pid->Kd * derivative;

  FP32 offset;
  if (pid->error > 0) {
    offset = pid->offsetVoltage;
  } else if (pid->error < 0) {
    offset = -pid->offsetVoltage;
  } else {
    offset = 0;
  }
  // Calculate total output
  pid->output = Pout + Iout + Dout + offset;
  // pid->output = 0;

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
}

void vUpdateSetpoints() {
  if (!uxQueueMessagesWaiting(q_uartSetpoint)) { return; }

  if (xSemaphoreTake(m_uartSetpoint, 1)) {
    uartAngle_t setpoint;
    while (xQueueReceive(q_uartSetpoint, &setpoint, 0)) {
      // Setpoint received
      if (setpoint.axis == PAN) {
        if (setpoint.relative) {
          pidPan.setpoint = pidPan.measurement + setpoint.angle;
        } else {
          pidPan.setpoint = setpoint.angle;
        }
      } else {
        if (setpoint.relative) {
          pidTilt.setpoint = pidTilt.measurement + setpoint.angle;
        } else {
          pidTilt.setpoint = setpoint.angle;
        }
      }
    }

    if (pidPan.setpoint > PAN_ANGLE_MAX) {
      pidPan.setpoint = PAN_ANGLE_MAX;
    } else if (pidPan.setpoint < PAN_ANGLE_MIN) {
      pidPan.setpoint = PAN_ANGLE_MIN;
    }
    if (pidTilt.setpoint > TILT_ANGLE_MAX) {
      pidTilt.setpoint = TILT_ANGLE_MAX;
    } else if (pidTilt.setpoint < TILT_ANGLE_MIN) {
      pidTilt.setpoint = TILT_ANGLE_MIN;
    }

    xSemaphoreGive(m_uartSetpoint);
  }
}

void vReceiveAngles() {
  if(!uxQueueMessagesWaiting(q_spiAngle)) { return; }

  if (xSemaphoreTake(m_spiAngle, 1)) {
    spiAngle_t angle;
    while (xQueueReceive(q_spiAngle, &angle, 0)) {
      if (angle.axis == PAN) {
        pidPan.measurement = angle.angle;
      } else {
        pidTilt.measurement = angle.angle;
      }
    }
    // Release the mutex
    xSemaphoreGive(m_spiAngle);
  }
}

void vSendAngles() {
  // Get the mutex
  if (xSemaphoreTake(m_uartAngle, 1)) {
    uartAngle_t panAngle = {PAN, pidPan.measurement, FALSE};
    uartAngle_t tiltAngle = {TILT, pidTilt.measurement, FALSE};
    xQueueSendToBack(q_uartAngle, &panAngle, 0);
    xQueueSendToBack(q_uartAngle, &tiltAngle, 0);

    // Release the mutex
    xSemaphoreGive(m_uartAngle);
  }
}

void vSendDutyCycles() {
  if(xSemaphoreTake(m_spiDutyCycle, 0)) {
    INT16S panDuty = pidPan.output * VOLTAGE_TO_DUTY;
    INT16S tiltDuty = pidTilt.output * VOLTAGE_TO_DUTY;

    // Make sure the duty cycle is within limits
    if (panDuty > 1023) {
      panDuty = 1023;
    } else if (panDuty < -1023) {
      panDuty = -1023;
    }
    if (tiltDuty > 1023) {
      tiltDuty = 1023;
    } else if (tiltDuty < -1023) {
      tiltDuty = -1023;
    }
    spiDutyCycle_t panDutyCycle = {PAN, panDuty};
    spiDutyCycle_t tiltDutyCycle = {TILT, tiltDuty};
    
    // Send the duty cycles (output from the PID controller
    xQueueSendToBack(q_spiDutyCycle, &panDutyCycle, 0);
    xQueueSendToBack(q_spiDutyCycle, &tiltDutyCycle, 0);

    // Release the mutex
    xSemaphoreGive(m_spiDutyCycle);
  }
}

void vSendVoltages(){
  if(xSemaphoreTake(m_uartVoltage, 1)) {
    uartVoltage_t panVoltage = {PAN, pidPan.output};
    uartVoltage_t tiltVoltage = {TILT, pidTilt.output};
    xQueueSendToBack(q_uartVoltage, &panVoltage, 0);
    xQueueSendToBack(q_uartVoltage, &tiltVoltage, 0);

    // Release the mutex
    xSemaphoreGive(m_uartVoltage);
  }
}

void vControllerTask() {
  // Initialize the controller variables
  vControllerInit();

  BOOLEAN controllerEnabled = FALSE;

  // Task loop
  while (1) {
    // Get current tick for correct timing
    portTickType xLastWakeTime = xTaskGetTickCount();
    if ((GPIO_PORTF_DATA_R & 0b00010000)==0) {
      pidPan.setpoint = pidPan.measurement;
      pidTilt.setpoint = pidTilt.measurement;
      controllerEnabled = TRUE;
    } else if ((GPIO_PORTF_DATA_R & 0b00000001)==0) {
      controllerEnabled = FALSE;
    }

    // Get the latest setpoint (take all values from q_uartSetpoint)
    vUpdateSetpoints();

    // Get the latest measurement (take all values from q_spiAngle)
    vReceiveAngles();

    // Send the latest measurements to the computer (q_uartAngle)
    vSendAngles();

    // Update controller (PID)
    if (controllerEnabled) {
      vUpdateController(&pidPan);
      vUpdateController(&pidTilt);
    }

    // Send the latest output to the motor (q_spiDutyCycle)
    vSendDutyCycles();
    
    // Send the latest output to the computer (q_uartVoltage)
    vSendVoltages();

    // Make sure the task runs at 100Hz
    vTaskDelayUntil(&xLastWakeTime, CONTROLLER_PERIOD_MS / portTICK_RATE_MS);
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
