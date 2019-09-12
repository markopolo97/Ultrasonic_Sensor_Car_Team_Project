//*************************************************************
// CECS 347 FINAL PROJECT: THREE DIGITAL SENSORS:
//*************************************************************
// FinalProject.c
// WITHOUT POT USED YET!!!!
//------------------------------------------------------------
// Runs on TM4C123
//------------------------------------------------------------
// Use Timer0A, Timer 1A, Timer2A in input edge mode 16-bits count.  
//------------------------------------------------------------
// PB7, PB5, PB1 connects to left Ultrasonic sensor trigger pin.
// PB6, PB4, PB0 connects to right Ultrasonic sensor trigger pin.
//*************************************************************
// THREE DIGITAL SENSOR PORT LOCATIONS AND TIMERS USED:
//*************************************************************
// PB6 and PB7 : Left  Sensor | Timer 0A
// PB4 and PB5 : Front Sensor | Timer 1A
// PB0 and PB1 : Right Sensor | Timer 2A
//------------------------------------------------------------
// Each H-Bridge connects to a pair of motors(left motors & right motors
// PC4 and PC5 operate |   PWM 	 |of each H-Bridge
// PD2 and PD3 operate |DIRECTION|of each H-Birdge
//------------------------------------------------------------
// PE2 will operate the POT using analog signal(ACD0/Sequencer 3)
//------------------------------------------------------------
// Onboard Switches and LEDs:
// Emergency Stop 		| from PF0 | Switch 1
// Speed Mode Change 	| from PF4 | Switch 2
// RED	: STOP
// BLUE	: LEFT 	
// GREEN: RIGHT 
//------------------------------------------------------------
// Nokia LCD displays distance of both sensors in cm and PWM percentage
// 		SSI0Fss 		 - PA3
//		Reset   		 - PA7
//	  Data/Command - PA6
// 		SSI0Tx       - PA5
// 		SSI0Clk      - PA2
//------------------------------------------------------------
// Software by Mark Aquiapao and TrienVy Le
// Last Date Revised: December 14th, 2018
//********************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "ADCSWTrigger.h"
#include "PWM.h"
#include "PLL.h"
#include "SysTick.h"
#include "Nokia5110.h"
#include "tm4c123gh6pm.h"
//********************************************************************************************
#define MC_LEN 				0.0625 // length of one machine cyce in microsecond for 16MHz clock
#define SOUND_SPEED 	0.0343 // centimeter per micro-second
#define MAX_DURATION 	0xFFFF
#define MAX						70
// Port Definitions
#define L_Echo			0x40
#define R_Echo			0x01
#define F_Echo			0x10
#define Triggers	  0xA2
#define PF4					0x10
#define PF0					0x01
#define PF04				0x11
// FOR LIGHTS:
#define LIGHT  			(*((volatile unsigned long *)0x400253FC))	
#define RED 				0x02 // STOP
#define BLUE 				0x04 // LEFT
#define GREEN 			0x08 // RIGHT
// For Direction
#define FORWARD 1
#define REVERSE 0
//********************************************************************************************
// For Distance
#define DISTANCE  20
#define SHARP_DIS	28
//********************************************************************************************
// For Switches
#define SW2					0x01
#define SW1 				0x10
// For Speed
#define PERCENT_0 	1
#define PERCENT_25 	1024
#define PERCENT_50 	2048
#define PERCENT_75 	3072
#define PERCENT_100 4096
#define TOGGLE 			1
#define ALLDATA 		0xFF
//********************************************************************************************
// For Mode
#define RACE   1
#define NORMAL 0
//********************************************************************************************
//********************************************************************************************
//void Timer0_Init(void);
void Timer0_Init(void);
void Timer1_Init(void);
void Timer2_Init(void);
void WaitForInterrupt(void);  // low power mode
void EnableInterrupts(void);
void DisableInterrupts(void);
void delay(unsigned long);
void Switch_Init(void);
void Motor_Init(void);
void updatePWM(void);
int Speed_Mode(int, int);
//********************************************************************************************
// INITIALIZE VARIABLES:
uint32_t 	L_period 	= 0, R_period 	= 0, F_period 	= 0;
uint8_t 	L_done 	 	= 0, R_done 	 	= 0, F_done 	 	= 0, 
					L_timeout	= 0, R_timeout	= 0, F_timeout	= 0;
//-----------------------------------------------------------------------------
// the following variables are for testing purpose, need to move inside main after testing
uint32_t 	L_distance		=	0, R_distance 	  = 0, F_distance;
uint32_t 	L_first_time 	= 0, R_first_time 	= 0, F_first_time;
uint32_t 	L_second_time	= 0, R_second_time	= 0, F_second_time;
uint32_t 	L_first_read 	= 0, R_first_read 	= 0, F_first_read,
					L_second_read	= 0, R_second_read	= 0, F_second_read;
uint8_t  	L_OutOfRange 	= 0, R_OutOfRange 	= 0, F_OutOfRange;
//------ THESE VALUES MUST BE INITIALIZED -------
bool 						stop 			= false;	//set to FALSE
unsigned long 	distance 	= 20;  		//set to 20 cm
//--------------------------------------------------
// For POT
//static unsigned long POT;
int SPEED; 
int DIR = FORWARD;
int MODE = NORMAL;
static unsigned int DUTY;
int POT_PERCENT = 0;
//********************************************************************************************
int main(void){
	DisableInterrupts();
	Switch_Init();
	//PLL_Init();   								// 80 MHz clock
	PortC_PWM_Init(4096);							// 100% duty cycle, Forward
	//ADC0_InitSWTriggerSeq3_Ch1();
	Nokia5110_Init();							// initialize the nokia display
  Nokia5110_Clear();						// clear the nokia screen
	Motor_Init();									// initialize motors 
	SysTick_Init();         			// use default 16MHz clock
	Timer0_Init();         				// initialize timer0
	Timer1_Init();								// initialize timer2
	Timer2_Init();								// initialize timer1
  EnableInterrupts();
  while(1){
		if(!stop){
			//updatePWM();
			//------------------------------------------------------------------
			delay(833333);  										// delay for 
			Nokia5110_Clear();									// clear the nokia screen
				//------------------------------------------------------------------
			Nokia5110_SetCursor(0, 0);        // one leading spaces, first row
			Nokia5110_OutString("Right:");		// display left distance
			if((!((R_OutOfRange)))&&(R_distance<MAX))
				Nokia5110_OutUDec(R_distance);	
			else 															// Out of Range(OOR)
				Nokia5110_OutString("OOR");
			//------------------------------------------------------------------
			Nokia5110_SetCursor(0, 2);        // one leading spaces, second row
			Nokia5110_OutString("Left:");			// display left distance
			if((!((L_OutOfRange)))&&(L_distance<MAX))
				Nokia5110_OutUDec(L_distance);	
			else 															// Out of Range(OOR)
				Nokia5110_OutString("OOR");
			//------------------------------------------------------------------
			Nokia5110_SetCursor(0, 4);        // one leading spaces, fourth row
			Nokia5110_OutString("Front:");		// display front distance
			if((!((F_OutOfRange)))&&(F_distance<MAX))
				Nokia5110_OutUDec(F_distance);	
			else 															// Out of Range(OOR)
				Nokia5110_OutString("OOR");
			updatePWM();}
			//------------------------------------------------------------------
		else{
			LIGHT = RED;
			Nokia5110_Clear();									// clear the nokia screen
			Nokia5110_OutString("E-STOP!");
			PWM0_3_CMPA_R = 0;  	// Left 
			PWM0_3_CMPB_R = 0;
		}
			
		GPIO_PORTB_DATA_R &= ~Triggers; // send low to triggers
		SysTick_Wait1us(2);
		GPIO_PORTB_DATA_R |=  Triggers; // send high to triggers
		SysTick_Wait1us(10);
		GPIO_PORTB_DATA_R &= ~Triggers; // send low to triggers
		//------------------------------------------------------------------
		// Timer 0A
		TIMER0_IMR_R 	 |= 0x00000004;    // enable capture mode event interrupt
	  TIMER0_CTL_R  	= 0x0000000D;    // Enable TIMER0A capture mode: both edges
    TIMER0_TAILR_R 	= MAX_DURATION;    // reload start value
		// Timer 1A
		TIMER1_IMR_R 	 |= 0x00000004;    // enable capture mode event interrupt
	  TIMER1_CTL_R  	= 0x0000000D;    // Enable TIMER1A capture mode: both edges
    TIMER1_TAILR_R 	= MAX_DURATION;    // reload start value
		// Timer 2A
		TIMER2_IMR_R 	 |= 0x00000004;    // enable capture mode event interrupt
	  TIMER2_CTL_R  	= 0x0000000D;    // Enable TIMER2A capture mode: both edges
    TIMER2_TAILR_R 	= MAX_DURATION;    // reload start value
		//------------------------------------------------------------------
		// Use general purpose timer input edge mode 16 bits count, 
		// detectable range: (65535*62.58*10^(-3)*0.0343)/2=70.2cm
		// Notice that the detect range for HC - SR04 ultrasonic sensor is 400cm
    // Since our application only need to detect obstcle within 70cm, 
    // 16 bits count is good enough for us.		
		// Timer 0A
		//------------------------------------------------------------------
		SysTick_Wait(MAX_DURATION);
		// Timer 0A
	  TIMER0_CTL_R  =  0x00000000;    // disable TIMER0A
    TIMER0_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
		// Timer 1A
	  TIMER1_CTL_R  =  0x00000000;    // disable TIMER1A
    TIMER1_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
		// Timer 2A
	  TIMER2_CTL_R  =  0x00000000;    // disable TIMER2A
    TIMER2_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
		//------------------------------------------------------------------------
		if (L_done) {
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // distance = (duration * 0.0343)/2;
		  L_distance = (L_period*MC_LEN*SOUND_SPEED)/2;		
			L_OutOfRange = 0;}
		//------------------------------------------------------------------------
		else{ // Out of Range:
			L_distance = 0;
			L_OutOfRange = 1;}
		//------------------------------------------------------------------------
		if (R_done) { 
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // distance = (duration * 0.0343)/2;
		  R_distance = (R_period*MC_LEN*SOUND_SPEED)/2;		
			R_OutOfRange = 0;}
		//------------------------------------------------------------------------
		else{ // Out of Range:
			R_distance = 0;
			R_OutOfRange = 1;}
		//------------------------------------------------------------------------
		if (F_done) { 
			// The speed of sound is approximately 340 meters per second, 
			// or  .0343 c/µS.
      // distance = (duration * 0.0343)/2;
		  F_distance = (F_period*MC_LEN*SOUND_SPEED)/2;		
			F_OutOfRange = 0;}
		//------------------------------------------------------------------------
		else{ // Out of Range:
			F_distance = 0;
			F_OutOfRange = 1;}
		//------------------------------------------------------------------------
	}
}
//********************************************************************************************
// Left Sensor
//********************************************************************************************
/// ***************** Timer0_Init ****************
// Activate TIMER0 interrupts to capture 
// the period between a rising edge and a falling edge
// to be used to calculate distance detected by
// an ultrasonic sensor.
/// **********************************************
// Ports PB6(Echo) and PB7(Trigger) are used.
//********************************************************************************************
void Timer0_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B 
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
	//---------------------------------------------------------------------
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
  GPIO_PORTB_DEN_R |= 0x40;        // enable digital I/O on PB6
                                   // configure PB6 as T0CCP0
	//---------------------------------------------------------------------
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
  GPIO_PORTB_AMSEL_R &= ~0x02;     // disable analog functionality on PB6
	//---------------------------------------------------------------------
	// PB7 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x80;     // disable alt funct on PB7
  GPIO_PORTB_DEN_R |= 0x80;        // enable digital I/O on PB7
                                   // configure PB7 as GPIO
	//---------------------------------------------------------------------
  GPIO_PORTB_PCTL_R &= ~0xF0000000;
  GPIO_PORTB_AMSEL_R &= ~0x80;     // disable analog functionality on PB7
	GPIO_PORTB_DIR_R |= 0x80;        // PB7 is output
	//---------------------------------------------------------------------
  TIMER0_CTL_R  &= ~0x0000000F;   // 1) disable TIMER0A during setup
  TIMER0_CFG_R 	 = 0x00000004;    // 2) configure for 16-bit timer mode
	TIMER0_TAMR_R  = 0x00000007;   	// 3A) edge time capture mode: count down
  TIMER0_TAILR_R = MAX_DURATION;  // 4A) start value
  TIMER0_ICR_R 	 = 0x00000004;    // 6) clear TIMER0A capture and timeout flag
  TIMER0_IMR_R 	 = 0x00000000;    // 7) disable capture mode event interrupt
  // Timer 0A 
	NVIC_PRI4_R = (NVIC_PRI4_R&0x1FFFFFFF)|0x80000000; // 8) priority 2
  // interrupts enabLIGHT in the main program after all devices initialized
  // vector number 35, interrupt number 19
  NVIC_EN0_R |= 0x80000;           // 9) enable IRQ 19 in NVIC
}
//********************************************************************************************
void Timer0A_Handler(void)
{  //----------------------------------------------------------------------------------------
	static uint32_t L_first = 0;
	 //----------------------------------------------------------------------------------------
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowLIGHTge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & L_Echo)==L_Echo) { //rising edge
		L_first = TIMER0_TAR_R;  
		L_first_time = L_first; // this line of code is for debugging purpose, can be removed
		L_done = 0;
	}//----------------------------------------------------------------------------------------
	else if (L_first != 0){
		L_period = (L_first - TIMER0_TAR_R)&0x00FFFFFF; // 24 bits counter
		L_second_time = TIMER0_TAR_R; // this line of code is for debugging purpose, can be removed
		L_done = 1;
		L_first = 0;
	  TIMER0_CTL_R = 0x00000000;    // disable TIMER0A 
    TIMER0_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
	}//----------------------------------------------------------------------------------------                                 
}
//********************************************************************************************
// Front Sensor
/// ***************** Timer1_Init ****************
// Activate TIMER1 interrupts to capture 
// the period between a rising edge and a falling edge
// to be used to calculate distance detected by
// an ultrasonic sensor.
/// *******************************************
/// **********************************************
// Ports PB4(Echo) and PB5(Trigger) are used.
//********************************************************************************************
void Timer1_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x02;      // activate timer1
	//---------------------------------------------------------------------
	SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_AFSEL_R |= 0x10;      // enable alt funct on PB4
  GPIO_PORTB_DEN_R |= 0x10;        // enable digital I/O on PB4
                                   // configure PB4 as T3CCP0
	//---------------------------------------------------------------------
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFF0FFFF)+0x00070000;
  GPIO_PORTB_AMSEL_R &= ~0x10;     // disable analog functionality on PB4
	//---------------------------------------------------------------------	
	// PF1 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x20;      // disable alt funct on PB5
  GPIO_PORTB_DEN_R |= 0x20;        	// enable digital I/O on PB5
																		// configure PB5 as GPIO
  GPIO_PORTB_PCTL_R &= ~0x00F00000;
  GPIO_PORTB_AMSEL_R &= ~0x20;     	// disable analog functionality on PB5
	GPIO_PORTB_DIR_R |= 0x20;        	// PB5 is output
	//---------------------------------------------------------------------
  TIMER1_CTL_R &= ~0x0000000F;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000004;    	// 2) configure for 16-bit timer mode
	TIMER1_TAMR_R = 0x00000007;   	// 3) edge time capture mode: count down
  TIMER1_TAILR_R = MAX_DURATION;  // 4) start value
  TIMER1_ICR_R = 0x00000004;    	// 6) clear TIMER1A capture and timeout flag
  TIMER1_IMR_R = 0x00000000;    	// 7) disable capture mode event interrupt
	//---------------------------------------------------------------------		
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF0FFF)|0x00008000; // 8) priority 4
	// interrupts enabLIGHT in the main program after all devices initialized
	// vector number 37, interrupt number 21
  NVIC_EN0_R |= 0x200000;           // 9) enable IRQ 21 in NVIC
	//---------------------------------------------------------------------		
}
//********************************************************************************************
void Timer1A_Handler(void)
{ //----------------------------------------------------------------------------------------
	static uint32_t F_first = 0;
	//----------------------------------------------------------------------------------------
  TIMER1_ICR_R = TIMER_ICR_CAECINT;// acknowLIGHTge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & F_Echo)==F_Echo) { //rising edge
		F_first = TIMER1_TAR_R;  
		F_first_time = F_first; // this line of code is for debugging purpose, can be removed
		F_done = 0;
	}//----------------------------------------------------------------------------------------
	else if (F_first != 0){
		F_period = (F_first - TIMER1_TAR_R)&0x00FFFFFF; // 24 bits counter
		F_second_time = TIMER1_TAR_R; // this line of code is for debugging purpose, can be removed
		F_done = 1;
		F_first = 0;
	  TIMER1_CTL_R = 0x00000000;    // disable TIMER2A 
    TIMER1_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
	}//----------------------------------------------------------------------------------------
}
//********************************************************************************************
// Right Sensor
// ***************** Timer2_Init ****************
// Activate TIMER2 interrupts to capture 
// the period between a rising edge and a falling edge
// to be used to calculate distance detected by
// an ultrasonic sensor.
/// **********************************************
// Ports PB0(Echo) and PB1(Trigger) are used.
//********************************************************************************************
void Timer2_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x04;      // activate timer2
	//---------------------------------------------------------------------
	SYSCTL_RCGCGPIO_R |= 0x0002;     // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_AFSEL_R |= 0x01;      // enable alt funct on PB0
  GPIO_PORTB_DEN_R |= 0x01;        // enable digital I/O on PB0
                                   // configure PB0 as T2CCP0
	//---------------------------------------------------------------------
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF0)+0x00000007;
  GPIO_PORTB_AMSEL_R &= ~0x01;     // disable analog functionality on PB0
	//---------------------------------------------------------------------	
	// PF1 connects to Ultrasonic sensor trigger pin
  GPIO_PORTB_AFSEL_R &= ~0x02;      // disable alt funct on PB1
  GPIO_PORTB_DEN_R |= 0x02;        	// enable digital I/O on PB1
																		// configure PB1 as GPIO
  GPIO_PORTB_PCTL_R &= ~0x00000F0;
  GPIO_PORTB_AMSEL_R &= ~0x02;     	// disable analog functionality on PB1
	GPIO_PORTB_DIR_R |= 0x02;        	// PB1 is output
	//---------------------------------------------------------------------
  TIMER2_CTL_R &= ~0x0000000F;    // 1) disable TIMER0A during setup
  TIMER2_CFG_R = 0x00000004;    	// 2) configure for 16-bit timer mode
	TIMER2_TAMR_R = 0x00000007;   	// 3) edge time capture mode: count down
  TIMER2_TAILR_R = MAX_DURATION;  // 4) start value
  TIMER2_ICR_R = 0x00000004;    	// 6) clear TIMER0A capture and timeout flag
  TIMER2_IMR_R = 0x00000000;    	// 7) disable capture mode event interrupt
  //---------------------------------------------------------------------
	NVIC_PRI5_R = (NVIC_PRI4_R&0x1FFFFFFF)|0x80000000; // 8) priority 2
  // interrupts enabLIGHT in the main program after all devices initialized
  // vector number 39, interrupt number 23
  NVIC_EN0_R |= 0x800000;           // 9) enable IRQ 23 in NVIC
	//---------------------------------------------------------------------
}
//********************************************************************************************
void Timer2A_Handler(void)
{ //----------------------------------------------------------------------------------------
	static uint32_t R_first = 0;
	//----------------------------------------------------------------------------------------
  TIMER2_ICR_R = TIMER_ICR_CAECINT;// acknowLIGHTge TIMER0A capture interrupt
	if ((GPIO_PORTB_DATA_R & R_Echo)==R_Echo) { //rising edge
		R_first = TIMER2_TAR_R;  
		R_first_time = R_first; // this line of code is for debugging purpose, can be removed
		R_done = 0;
	}//----------------------------------------------------------------------------------------
	else if (R_first != 0){
		R_period = (R_first - TIMER2_TAR_R)&0x00FFFFFF; // 24 bits counter
		R_second_time = TIMER2_TAR_R; // this line of code is for debugging purpose, can be removed
		R_done = 1;
		R_first = 0;
	  TIMER2_CTL_R = 0x00000000;    // disable TIMER2A 
    TIMER2_IMR_R &= ~0x00000004;    // disable capture mode event interrupt
	}//----------------------------------------------------------------------------------------
}
//********************************************************************************************
// Front Sensor
//********************************************************************************************
//-----------------SWITCH INITIALIZE---------------------
	//	This function is to initialize the switch that will
	//	be used and the LIGHT to indicate the robot's direction
	//	and motion
	//	IN PROJECT 
	//	SW1 	= emergency stop!!
	//	SW2 	= changes avoidance mode
	//	LIGHT G = will show when the right	
	//	LIGHT B = will show when the left 
	//	LIGHT R = will show when the motor is stopped			
	//--------------------------------------------------------
//********************************************************************************************
void Switch_Init(void){  unsigned long volatile delay;
	SYSCTL_RCGC2_R |= 0x00000020; 		// activate clock for port F
	delay = SYSCTL_RCGC2_R;
	//-------------------------------------------------------------------------
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
	GPIO_PORTF_AMSEL_R = 0x00;        // disable analog function
	GPIO_PORTF_PCTL_R = 0;   					// GPIO clear bit PCTL  
	GPIO_PORTF_DIR_R = 0x0E;          // PF4,PF0 input, PF3,PF2,PF1 output   
	GPIO_PORTF_AFSEL_R = 0x00;        // no alternate function
	GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
	GPIO_PORTF_DEN_R = 0x1F;          // enable digital pins PF4-PF0  
	//-------------------------------------------------------------------------
	GPIO_PORTF_IS_R &= ~0x11;     		// PF4,PF0 is edge-sensitive
	GPIO_PORTF_IBE_R &= ~0x11;    		// PF4,PF0 is not both edges
	GPIO_PORTF_IEV_R &= ~0x11;    		// PF4,PF0 falling edge event
	GPIO_PORTF_ICR_R = 0x11;      		// clear flags 4,0
	GPIO_PORTF_IM_R |= 0x11;      		// arm interrupt on PF4,PF0 
	//-------------------------------------------------------------------------
	// priority 2
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; 
	NVIC_EN0_R = 0x40000000;      		// enable interrupt 30 in NVIC
}
//********************************************************************************************
	//----------------------GPIO F HANDLER---------------------
	// this function allows us to change the mode through
	// the port F interupt 
	// SW 1 -> EMERGENCY STOP
	// SW 2 -> CHANGES THE AVOIDANCE DISTANCE 
	//					-> toggles between 20 cm & 30 cm
	//----------------------------------------------------------
//*******************************************************************
// Handler Logic for Port F
//*******************************************************************
void GPIOPortF_Handler(void){
	//----- SW 1 -------  -> EMERGENCY STOP
	if(GPIO_PORTF_RIS_R&SW1){
		GPIO_PORTF_ICR_R = SW1;
		stop = (stop==true) ? false : true;} // toggle the stop
		//if stop is true then change the pwm to 0 
		if(stop==true){
			//PWM0_3_CMPA_R = Speed_Mode(100, FORWARD); 4096
			//PWM0_3_CMPB_R = Speed_Mode(25, REVERSE);} 100
			PWM0_3_CMPA_R = 100;  	// Left 
			PWM0_3_CMPB_R = 100;} 	// Right
	//---- SW 2 ------  -> CHANGES THE AVOIDANCE DISTANCE 
	if(GPIO_PORTF_RIS_R&SW2){
		GPIO_PORTF_ICR_R = SW2;
		distance = (distance == 20) ? 30 : 20; }
}
//---------------UPDATE THE PWM---------------------------
	//	This function will be calLIGHT everytime a SysTick interupt
	//	is calLIGHT.   
	//
	//	NOTE**: this function will update the pwm through hardware
	//					pwm and not systick. Also this function will also 
	//					update the pwm accordingly to the distance
	//---------------------------------------------------------
//********************************************************************************************
void updatePWM(void){
	if(R_distance <30 && !R_OutOfRange){ // Left Turn
		LIGHT = GREEN;
		PWM0_3_CMPA_R = PERCENT_25;
		PWM0_3_CMPB_R = PERCENT_75;
		if(R_distance <33 ){ // Sharp Left
			PWM0_3_CMPA_R = 100;//Speed_Mode(100, FORWARD);
			PWM0_3_CMPB_R = PERCENT_75;//Speed_Mode(0, FORWARD);
		}
	}
	else if(L_distance <30 && !L_OutOfRange){ // Right Turn
		LIGHT = BLUE;
		PWM0_3_CMPA_R = PERCENT_75;//Speed_Mode(25, FORWARD);
		PWM0_3_CMPB_R = PERCENT_25;//(75, FORWARD);
		if(L_distance <33 ){ // Sharp Right
			PWM0_3_CMPA_R = PERCENT_75;//Speed_Mode(0, FORWARD);
			PWM0_3_CMPB_R = 100;//Speed_Mode(100, FORWARD);
		}
	}
	//-------------------------------------------------------------------------
	// ADJUST DISTANCE OF LEFT AND RIGHT - Comment Out For Debugging
	//-------------------------------------------------------------------------
	else if( ((L_distance-R_distance <= 8) || (R_distance-L_distance <= 8)) 
						&& !L_OutOfRange && !R_OutOfRange){
	//*************************************************************************
		// RIGHT Adjust
		if(L_distance < R_distance && (!L_OutOfRange && !R_OutOfRange)){
			PWM0_3_CMPA_R = PERCENT_75;
			PWM0_3_CMPB_R = PERCENT_50;
		}
		// LEFT Adjust
		else if(R_distance < L_distance && (!L_OutOfRange && !R_OutOfRange)){
			PWM0_3_CMPA_R = PERCENT_50;
			PWM0_3_CMPB_R = PERCENT_75;
		}
	}
	else if(F_distance <30 && !F_OutOfRange){
		LIGHT = RED;
		PWM0_3_CMPA_R = PERCENT_0;//Speed_Mode(0, FORWARD);
		PWM0_3_CMPB_R = PERCENT_0;}//Speed_Mode(0, FORWARD);}
	//-------------------------------------------------------------------------
	/*else if( (!(F_distance <DISTANCE && !F_OutOfRange)) && 
					 (!(L_distance <DISTANCE && !L_OutOfRange)) &&
					 (!(R_distance <DISTANCE && !R_OutOfRange)) ){
		// Every Sensor is Out Of Range
		LIGHT = RED;
		PWM0_3_CMPA_R = PERCENT_0;
		PWM0_3_CMPB_R = PERCENT_0;
	}*/
	//-------------------------------------------------------------------------
	else{
		LIGHT ^= 0x0E;
		PWM0_3_CMPA_R = PERCENT_50;//Speed_Mode(75, FORWARD);
		PWM0_3_CMPB_R = PERCENT_50;}//Speed_Mode(75, FORWARD);}
	//-------------------------------------------------------------------------
	/*POT = ADC0_InSeq3();
	//-------------------------------------------------------------------------
	//calculates the duty cycle in percentage to be
	SPEED = (POT >= 2700) ? 100: (POT >= 1400 && POT <= 2300) ?  50: 35;
	PWM0_3_CMPA_R = Speed_Mode(SPEED, FORWARD);
	PWM0_3_CMPB_R = Speed_Mode(SPEED, FORWARD);*/
		// END PWM LOGIC
}
//********************************************************************************************
int Percent_Mode(int PERCENT, int DIR, int MODE){
	if(MODE==NORMAL){
		//-----------------------------------------------------	
		// 	0% Duty Cycle
		//-----------------------------------------------------	
		if(PERCENT== 0)
			DUTY = (DIR) ? 1 : 4096;
		//-----------------------------------------------------	
		// 	25% Duty Cycle
		//-----------------------------------------------------	
		else if(PERCENT == 25)
			DUTY = (DIR) ? 1024 : 3072;
		//-----------------------------------------------------
		// 	50% Duty Cycle
		//-----------------------------------------------------	
		else if(PERCENT == 50)
			DUTY = (DIR) ? 2048 : 2048;
		//-----------------------------------------------------	
		// 	75% Duty Cycle
		//-----------------------------------------------------	
		else if(PERCENT == 75)
			DUTY = (DIR) ? 3072 : 1024;
		//-----------------------------------------------------	
		// 	100% Duty Cycle
		//-----------------------------------------------------	
		else
			DUTY = (DIR) ? 4096 : 1;
		//-----------------------------------------------------	
	}
	else{ // RACE MODE
		//-----------------------------------------------------	
		// 	0% Duty Cycle
		//-----------------------------------------------------	
		if(PERCENT== 0)
			DUTY = (DIR) ? 1 : 4998;
		//-----------------------------------------------------	
		// 	25% Duty Cycle
		//-----------------------------------------------------	
		else if(PERCENT == 25)
			DUTY = (DIR) ? 1250 : 3749;
		//-----------------------------------------------------
		// 	50% Duty Cycle
		//-----------------------------------------------------	
		else if(PERCENT == 50)
			DUTY = (DIR) ? 2500 : 2500;
		//-----------------------------------------------------	
		// 	75% Duty Cycle
		//-----------------------------------------------------	
		else if(PERCENT == 75)
			DUTY = (DIR) ? 3749 : 1250;
		//-----------------------------------------------------	
		// 	100% Duty Cycle
		//-----------------------------------------------------	
		else
			DUTY = (DIR) ? 4998 : 1;
		//-----------------------------------------------------	
	}
	return DUTY;
}
//********************************************************************************************
// Delays 3*ulCount cycles:
void delay(unsigned long ulCount){
  do{
    ulCount--;
	}while(ulCount);
}
//********************************************************************************************
//********************************************************************************************
void Sensor_LCD(void){
	//------------------------------------------------------------------
	delay(833333);  										// delay for 
	Nokia5110_Clear();									// clear the nokia screen
	//----------------------------------------------------------------
	Nokia5110_SetCursor(0, 0);        	// one leading spaces, first row
	Nokia5110_OutString("Right:");			// display left distance
	if((!((R_OutOfRange)))&&(R_distance<MAX))
		Nokia5110_OutUDec(R_distance);	
	else 																// Out of Range(OOR)
		Nokia5110_OutString("OOR");
	//------------------------------------------------------------------
	Nokia5110_SetCursor(0, 1);        	// one leading spaces, second row
	Nokia5110_OutString("Left:");				// display left distance
	if((!((L_OutOfRange)))&&(L_distance<MAX))
		Nokia5110_OutUDec(L_distance);	
	else 																// Out of Range(OOR)
		Nokia5110_OutString("OOR");
	//------------------------------------------------------------------
	Nokia5110_SetCursor(0, 2);        	// one leading spaces, fourth row
	Nokia5110_OutString("Front:");			// display front distance
	if((!((F_OutOfRange)))&&(F_distance<MAX))
		Nokia5110_OutUDec(F_distance);	
	else 																// Out of Range(OOR)
		Nokia5110_OutString("OOR");
	//------------------------------------------------------------------
	Nokia5110_SetCursor(0, 4);        	// one leading spaces, fourth row
	Nokia5110_OutString("Mode:");				// display mode
	if(MODE==RACE)
		Nokia5110_OutString("RACE");
	else
		Nokia5110_OutString("NORMAL");
	//------------------------------------------------------------------
}
void Emergency_Stop(void){
	/*//----------------------------------------------------------
	POT = ADC0_InSeq3();
	//----------------------------------------------------------
	//calculates the duty cycle in percentage to be
	if(POT>= 2700)							 POT_PERCENT = 100;
	else if(POT<2700&&POT>=2025) POT_PERCENT = 75;
	else if(POT<2025&&POT>=1350) POT_PERCENT = 50;
	else if(POT<1350&&POT>=675)  POT_PERCENT = 25;
	else 												 POT_PERCENT = 0;
	//-----------------------------------------------------------
	PWM0_3_CMPA_R = Percent_Mode(POT_PERCENT, FORWARD, MODE);
	PWM0_3_CMPB_R = Percent_Mode(POT_PERCENT, FORWARD, MODE);*/
	//-----------------------------------------------------------
	LIGHT = RED;
	Nokia5110_Clear();								// clear the nokia screen
	Nokia5110_SetCursor(0, 0); 
	Nokia5110_OutString("EMERGENCY");
	Nokia5110_SetCursor(0, 1); 
	Nokia5110_OutString("STOP!");
	Nokia5110_SetCursor(0, 3); 
	Nokia5110_OutString("DUTY CYCLE: ");
	Nokia5110_OutUDec(POT_PERCENT);
	Nokia5110_OutString("%");
	PWM0_3_CMPA_R = Percent_Mode(0, FORWARD, MODE); 
	PWM0_3_CMPB_R = Percent_Mode(0, FORWARD, MODE);
}
//********************************************************************************************
