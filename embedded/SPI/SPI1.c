/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: SPI1.c
*
* PROJECT....: Control and Regulation Project
*
* DESCRIPTION: Initialization of SPI for TIVA C TM4C123GH6PM
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
#include "SPI1.h"
#include "tm4c123gh6pm.h"


/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/

void SPI1_init(void)
{
    /* Enable clock to SPI1, GPIOD and GPIOF */
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;           /*set clock enabling bit for SPI1 */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;          /* enable clock to GPIOD for SPI1 */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;          /* enable clock to GPIOF for slave select */

    /*Initialize PD3 and PD0 for SPI1 alternate function*/

    GPIO_PORTD_AMSEL_R &= ~0b00001111;              /* disable analog functionality RD0 to RD3 */
    GPIO_PORTD_DEN_R |= 0b00001111;                 /* Set RD0 to RD3 as digital pin */
    GPIO_PORTD_DIR_R |= 0b00001011;
    GPIO_PORTD_AFSEL_R |= 0b00001111;               /* enable alternate function of RD0 to RD3*/
    GPIO_PORTD_PCTL_R &= ~0x0000FFFF;         /* assign RD0 and RD3 pins to SPI1 */
    GPIO_PORTD_PCTL_R |= 0x00002222;          /* assign RD0 and RD3 pins to SPI1  */

    /* Select SPI1 as a Master, POL = 0, PHA = 0, clock = 4 MHz, 8 bit data */
    SSI1_CR1_R = 0;       /* disable SPI1 and configure it as a Master */
    SSI1_CC_R = 0;        /* Enable System clock Option */
    SSI1_CPSR_R = 4;      /* Select prescaler value of 4 .i.e 16MHz/4 = 4MHz */
    SSI1_CR0_R = 0x0000F; /* 4MHz SPI1 clock, SPI mode, 8 bit data */
    SSI1_CR1_R |= 2;      /* enable SPI1 */
}

void SPI1_Read(unsigned char *data) {
  if ((SSI1_SR_R & (1<<2))) // Wait for Rx-FIFO not empty
    *data = SSI1_DR_R;            // Read received data
}

void SPI1_Write(unsigned char data) {
  while ((SSI1_SR_R & (1<<1)) == 0)
    ;               /* wait until Tx FIFO is not full */
  SSI1_DR_R = data; /* transmit byte over SSI1Tx line */
}
/****************************** End Of Module *******************************/
