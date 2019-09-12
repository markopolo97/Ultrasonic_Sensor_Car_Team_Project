// PWM.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>
//#include "inc/tm4c123gh6pm.h"
#include "tm4c123gh6pm.h"
// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 80 MHz/2 = 40 MHz (in this example)
// Output on PB6/M0PWM0
// Output on PB7/M0PWM1
//*******************************************************************************
void PortC_PWM_Init(int SPEED){
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R |= 0x01;									// activate clock for PWM0 module
	SYSCTL_RCGCGPIO_R |= 0x04;								// activate clock for Port C
	delay = SYSCTL_RCGCGPIO_R;								// delay
	SYSCTL_RCC_R |= 0x00100000;								// start clock divider
	SYSCTL_RCC_R |= 1 << 17;									// convert 80MHz/4, 20MHz
//------------------------------------------------------------------------------	
	GPIO_PORTC_AFSEL_R |= 0x30;								// enable alternate function of PC4,5
	GPIO_PORTC_PCTL_R &= ~0x00FF0000;					// clear PCTL for PWM
	GPIO_PORTC_PCTL_R |= 0x00440000;					// setting PCTL
	GPIO_PORTC_DEN_R |= 0x30;									// output of PC4,5
//------------------------------------------------------------------------------	
	PWM0_3_CTL_R = 	0;												// disable PWM0 Module 1 for set up
	PWM0_3_GENA_R = 0x0000008C;							
	PWM0_3_GENB_R = 0x0000080C;
	PWM0_3_LOAD_R = 5000 - 1;									// reload value for 62.5Hz, 16ms
	// Setting each duty cycle high for almost entire period:
	PWM0_3_CMPA_R = SPEED;								
	PWM0_3_CMPB_R = SPEED;		 						
	PWM0_3_CTL_R = 1;													// start timer
	PWM0_ENABLE_R = 0xC0;											// enable PWM0 Channel 6 & 7
}
//********************************************************************************************
void Motor_Init(void){
	SYSCTL_RCGC2_R |= 0x00000008; 			 // activate clock for port D
	// Forward and Backward Motors
	GPIO_PORTD_LOCK_R = 0x4C4F434B;
	GPIO_PORTD_AMSEL_R &= ~0x0C;      	// disable analog functionality on PD3 & PD2
  GPIO_PORTD_PCTL_R &= ~0x0000FF00;		// configure PD3 & PD2 as GPIO
	GPIO_PORTD_DIR_R |= 0x0C;    				// make PD3 & PD2 out
	GPIO_PORTD_DR8R_R |= 0x0C;    			// enable 8 mA drive on PD3 & PD2
	GPIO_PORTD_AFSEL_R = 0x00;  				// disable alt funct on PD3 & PD2
	GPIO_PORTD_DEN_R |= 0x0C;     			// enable digital I/O on PD3 & PD2
	GPIO_PORTD_DATA_R = 0x0C;   				// make PD3 & PD2 low
}
//********************************************************************************************

