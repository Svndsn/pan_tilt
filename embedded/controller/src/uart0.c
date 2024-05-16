#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#include "tm4c123gh6pm.h"
#include "projectdefs.h"
#include "uart0.h"
#include "emp_type.h"
#include "projectdefs.h"

typedef enum { Type, Length, Data, CheckSum } RxState_t ;

// UART0 queues
extern xQueueHandle q_uartDebug;    // Send debug messages
extern xQueueHandle q_uartAngle;    // Send current angle
extern xQueueHandle q_uartSetpoint; // Receive setpoint
extern xQueueHandle q_uartVoltage;  // Send motor voltages
extern xQueueHandle q_uartRawData;  // Input buffer
// UART0 Mutex
extern xSemaphoreHandle m_uartDebug;
extern xSemaphoreHandle m_uartAngle;
extern xSemaphoreHandle m_uartSetpoint;
extern xSemaphoreHandle m_uartVoltage;

void vUart0Init() {
  // Setup UART0
  // Enable the UART0 clock
  SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R0;
  // Set the alternate function for PA0 and PA1
  GPIO_PORTA_AFSEL_R |= 0b00000011;
  // Enable the alternate function for PA0 and PA1
  GPIO_PORTA_PCTL_R  |= 0x00000011;
  // Enable the GPIO pins for digital function (PA0 and PA1)
  GPIO_PORTA_DEN_R   |= 0b00000011;
  // Handle direction of the pins
  GPIO_PORTA_DIR_R   |= 0b00000010;

  // Disable UART0
  UART0_CTL_R &= ~(UART_CTL_UARTEN);

  INT32U baud_rate = 115200;
  INT32U brd;

  // Set the baud rate
  // IBRD = int(16,000,000 / (16 * 9600)) = int(104.1667)
  // Set the fractional part of the baud rate
  // FBRD = 0.1667 * 64 + 0.5 = int(11,16688)
  brd = 64000000 / baud_rate; // X-sys*64/(16*baudrate) = 16M*4/baudrate
  UART0_IBRD_R = brd / 64;
  UART0_FBRD_R = brd & 0x0000003F;

  // Set the transmission parameters
  UART0_LCRH_R = 0b01100000;  // 8-bit word length
  UART0_LCRH_R |= 0b00000110; // even parity
  UART0_LCRH_R |= (1 << 4);   // Enable FIFO

  // Setup interrupt
  // clear the interrupt
  UART0_ICR_R |= (1 << 4);
  // Select fifo interrupt level
  UART0_IFLS_R = 0b010000; // 1/2 full
  UART0_IM_R |= (1 << 4); // Enable the receive interrupt
  NVIC_EN0_R |= (1 << (INT_UART0 - 16)); // Enable the UART0 interrupt

  // Set the clock source
  UART0_CC_R = 0;
  // Enable the UART0
  UART0_CTL_R |=
      (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN); // Enable UART0
}

void vSendCharUart(INT8U chr) {
  while (UART0_FR_R & (1 << 5)){}
  UART0_DR_R = chr;
}

void vSendStringUart(const char *str) {
  while (*str) {
    vSendCharUart(*str);
    str++;
  }
}

void vCheckSum(INT8U *data, INT16U length, INT8U *checksum) {
  for (INT16U i = 0; i < length; i++) {
    *checksum ^= data[i];
  }
}
void vSendDebugUart() {
  if(uxQueueMessagesWaiting(q_uartDebug) == 0) { return; }

  // Send debug data
  uartDebug_t debug_data;
  if (xSemaphoreTake(m_uartDebug, 1)) {
    while(xQueueReceive(q_uartDebug, &debug_data, 0)) {
      INT8U checksum = 0;
      INT8U data_type = DEBUG_STRING;
      // Send the data type
      vSendCharUart(data_type);
      vCheckSum(&data_type, 1, &checksum);
      // Send the length of the string
      INT8U strlength = strlen(debug_data.string);
      vSendCharUart(strlength);
      vCheckSum(&strlength, 1, &checksum);

      // Send the string
      vSendStringUart(debug_data.string);
      vCheckSum((INT8U *)debug_data.string, strlength, &checksum);

      // Send the checksum
      vSendCharUart(checksum);
    }
    // Release the mutex
    xSemaphoreGive(m_uartDebug);
  }
}

void vSendAnglesUart(){
  // Check if there are any messages in the queue
  if(!uxQueueMessagesWaiting(q_uartAngle)) { return; }

  if (xSemaphoreTake(m_uartAngle, 1)) {
    // Send current angle (pan and tilt)
    uartAngle_t angle_data;

    // Create a union to store the angle
    union {
      FP32 angle;
      INT8U bytes[4];
    } angle_u;
    
    // Empty the queue to get the latest angles
    while (xQueueReceive(q_uartAngle, &angle_data, 0)) {
      INT8U checksum = 0;
      INT8U data_type;
      if (angle_data.axis == PAN) {
        data_type = ANGLE_PAN;
      } else {
        data_type = ANGLE_TILT;
      }
      vSendCharUart(data_type);
      vCheckSum(&data_type, 1, &checksum);

      // Send length of data (4 bytes FP32)
      INT8U length = 4;
      vSendCharUart(length);
      vCheckSum(&length, 1, &checksum);

      // Store the angle in the union
      angle_u.angle = angle_data.angle;

      // Send angle data
      vSendCharUart(angle_u.bytes[0]);
      vSendCharUart(angle_u.bytes[1]);
      vSendCharUart(angle_u.bytes[2]);
      vSendCharUart(angle_u.bytes[3]);

      vCheckSum(angle_u.bytes, 4, &checksum);
      // Send checksum
      vSendCharUart(checksum);
    }

    // Release the mutex
    xSemaphoreGive(m_uartAngle);
  }
}

void vSendMotorVoltageUart(){
  // Check if there are any messages in the queue
  if(!uxQueueMessagesWaiting(q_uartVoltage)) { return; }

  if (xSemaphoreTake(m_uartVoltage, 1)) {
    // Send motor voltage

    uartVoltage_t voltage_data;
    // Create a union to store the voltage
    union {
      FP32 voltage;
      INT8U bytes[4];
    } voltage_u;
    
    // Empty the queue to get the latest voltages
    while (xQueueReceive(q_uartVoltage, &voltage_data, 0)) {
      INT8U checksum = 0;
      INT8U data_type;
      if (voltage_data.motor == PAN) {
        data_type = VOLTAGE_PAN;
      } else {
        data_type = VOLTAGE_TILT;
      }
      vSendCharUart(data_type);
      vCheckSum(&data_type, 1, &checksum);

      // Send length of data (4 bytes FP32)
      INT8U length = 4;
      vSendCharUart(length);
      vCheckSum(&length, 1, &checksum);

      // Store the voltage in the union
      voltage_u.voltage = voltage_data.voltage;

      // Send voltage data
      vSendCharUart(voltage_u.bytes[0]);
      vSendCharUart(voltage_u.bytes[1]);
      vSendCharUart(voltage_u.bytes[2]);
      vSendCharUart(voltage_u.bytes[3]);

      vCheckSum(voltage_u.bytes, 4, &checksum);
      // Send checksum
      vSendCharUart(checksum);
    }

    // Release the mutex
    xSemaphoreGive(m_uartVoltage);
  }
}

void vReceiveSetpointsUart() {
  // Return if there is no data
  // if (!(UART0_FR_R & (1 << 4))) {
  //   INT8U data = UART0_DR_R;
  //   xQueueSendToBack(q_uartRawData, &data, 0);
  // }

  INT32U n_data = uxQueueMessagesWaiting(q_uartRawData);
  if (n_data == 0) {
    return;
  }

  static RxState_t rx_state = Type;
  static uartCommandType_t command_type;
  static INT8U data_length;
  static INT8U data_counter = 0;
  static union {
    FP32 value;
    INT8U bytes[4];
  } setpoint_u;
  // Initialize setpoints
  INT8U rxData;
  for (INT32U i = 0; i < n_data; i++) {
    xQueueReceive(q_uartRawData, &rxData, 0);
    switch (rx_state) {
    case Type: {
      if (rxData <= 8) {
        rx_state = Length;
        command_type = rxData;
      }
      break;
    }
    case Length: {
      data_length = rxData;
      if (data_length == 0 || data_length > 4) {
        rx_state = Type;
      } else {
        rx_state = Data;
        data_counter = 0;
      }
      break;
    }
    case Data: {
      if (data_counter < data_length) {
        setpoint_u.bytes[data_counter] = rxData;
        data_counter++;
      }
      if (data_counter == data_length) {
        rx_state = CheckSum;
      }
      break;
    }
    case CheckSum: {
      uint8_t checksum = 0;
      uint8_t data_type = command_type;
      vCheckSum(&data_type, 1, &checksum);
      vCheckSum(&data_length, 1, &checksum);
      vCheckSum(setpoint_u.bytes, data_length, &checksum);
      rx_state = Type;
      if (checksum != rxData) {
        continue;
      }
      // Checksum passed
      // Set the setpoint
      if (xSemaphoreTake(m_uartSetpoint, 1)) {
        switch (command_type) {
        case SETPOINT_PAN_ABSOLUTE: {
          uartAngle_t setpoint = {PAN, setpoint_u.value, FALSE};
          xQueueSendToBack(q_uartSetpoint, &setpoint, 0);
          break;
        }
        case SETPOINT_TILT_ABSOLUTE: {
          uartAngle_t setpoint = {TILT, setpoint_u.value, FALSE};
          xQueueSendToBack(q_uartSetpoint, &setpoint, 0);
          break;
        }
        case SETPOINT_PAN_RELATIVE: {
          uartAngle_t setpoint = {PAN, setpoint_u.value, TRUE};
          xQueueSendToBack(q_uartSetpoint, &setpoint, 0);
          break;
        }
        case SETPOINT_TILT_RELATIVE: {
          uartAngle_t setpoint = {TILT, setpoint_u.value, TRUE};
          xQueueSendToBack(q_uartSetpoint, &setpoint, 0);
          break;
        }
        default:
          break;
        }
        rx_state = Type;
        xSemaphoreGive(m_uartSetpoint);
      }
      break;
    }
    }
  }
}

void UART0_Handler() {
  // Clear the interrupt
  UART0_ICR_R |= (1 << 4);
  // This only recieves data and puts it in the queue
  // Only called when data is coming in fast to avoid missing data
  // Read the data
  while(!(UART0_FR_R & (1 << 4))) {
    INT8U data = UART0_DR_R;
    xQueueSendToBackFromISR(q_uartRawData, &data, 0);
  }
}

void vUart0Task(void *pvParameters) {
  // Task loop
  while (1) {
    // Send debug
    vSendDebugUart();

    // Send angles
    vSendAnglesUart();

    // Send motor voltage
    vSendMotorVoltageUart();

    // Receive setpoint
    vReceiveSetpointsUart();

    // Delay task
    vTaskDelay(2 / portTICK_RATE_MS); 
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
