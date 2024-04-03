/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: SPI1_Write.c
*
* PROJECT....: Control and Regulation Project
*
* DESCRIPTION: SPI write for TIVA C TM4C123GH6PM
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
#include "SPI1_Write.h"
#include "tm4c123gh6pm.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/

void SPI1_Write(unsigned char data)
{
    GPIO_PORTF_DATA_R &= ~(1<<2);     /* Make PF2 Selection line (SS) low */
    while((SSI1_SR_R & 2) == 0); /* wait until Tx FIFO is not full */
    SSI1_DR_R = data;            /* transmit byte over SSI1Tx line */
    while(SSI1_SR_R & 0x10);     /* wait until transmit complete */
    GPIO_PORTF_DATA_R |= 0x04;        /* keep selection line (PF2) high in idle condition */
}
/****************************** End Of Module *******************************/
