/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: main.c
*
* PROJECT....: Control and Regulation Project
*
* DESCRIPTION: main file for TIVA C TM4C123GH6PM
*
* Change Log:
******************************************************************************
* Date    Id    Change
* 240401
* --------------------
* 240401  MoH   Module created.
*
*****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "SPI1.h"
#include "SPI1_Write.h"
#include "delay.h"
#include "led.h"
#include "tm4c123gh6pm.h"
/*****************************    Defines    *******************************/
#define LED_PORT (GPIO_PORTF_R)     // Assuming LED is connected to Port F
#define LED_PIN  (1<<1)             // Assuming LED is connected to PF1

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
unsigned char received_data;
unsigned char led_on;

/*****************************   Functions   *******************************/

int main(void)
{
    SPI1_init();
    LED_init();

    while(1)
    {
        if (GPIO_PORTF_DATA_R & (1<<4))     // Check if SW1 is pressed
        {
            SPI1_Write('X');                // Send any arbitrary data (e.g., 'X')
            delay_ms(10);                   // Brief delay for SPI communication
            received_data = SPI1_Read();    // Read response from PYNQ

            if (received_data == 'Y')       // Example response for LED control
            {
                led_on = !led_on;           // Toggle LED state
                ControlLED(led_on);
            }
        }
    }
}
