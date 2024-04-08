/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: SPI1.h
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

#ifndef SPI1_H_
#define SPI1_H_

/***************************** Include files *******************************/

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/

void SPI1_init(void);
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Test function
******************************************************************************/
void SPI1_Write(unsigned char);
void SPI1_Read(unsigned char *data);


#endif /*SPI1_H_*/
/****************************** End Of Module *******************************/
