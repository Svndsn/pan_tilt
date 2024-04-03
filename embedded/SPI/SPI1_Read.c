/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: SPI1_Read.c
*
* PROJECT....: Control and Regulation Project
*
* DESCRIPTION: SPI read for TIVA C TM4C123GH6PM
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
#include "SPI1_Read.h"
#include "tm4c123gh6pm.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/

void SPI1_Read(unsigned char *data){
    while((SSI1_SR_R & 0x01) == 0)     // Wait for Rx-FIFO not empty
    *data = SSI1_DR_R;                 // Read received data

}
/****************************** End Of Module *******************************/
