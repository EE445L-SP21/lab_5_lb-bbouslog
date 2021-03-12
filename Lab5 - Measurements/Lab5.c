// Lab3.c
// Runs on TM4C123
// This program periodically samples ADC channel 0 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Daniel Valvano
// Jan 18, 2021

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// center of X-ohm potentiometer connected to PE3/AIN0
// bottom of X-ohm potentiometer connected to ground
// top of X-ohm potentiometer connected to +3.3V 
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "../inc/ADCSWTrigger.h"
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/CortexM.h"
#include "../inc/TExaS.h"
#include "../inc/ST7735.h"

#include "DAC.h"
#include "Switch.h"

// Instrument Voices

uint32_t Trumpet64[64]={
987, 1049, 1090, 1110, 1134, 1160, 1139, 1092, 1070, 1042, 1035, 1029, 1008, 1066, 1150, 1170, 1087, 915, 679, 372, 151, 
558, 1014, 1245, 1260, 1145, 1063, 984, 934, 960, 1027, 1077, 1081, 1074, 1064, 1042, 1010, 974, 968, 974, 994, 1039, 
1094, 1129, 1125, 1092, 1056, 1056, 1082, 1059, 1046, 1058, 1061, 1045, 1034, 1050, 1094, 1112, 1092, 1063, 1053, 1065, 1052, 992};
	
uint32_t Oboe64[64]={
1024, 1024, 1014, 1008, 1022, 1065, 1093, 1006, 858, 711, 612, 596, 672, 806, 952, 1074, 1154, 1191, 
1202, 1216, 1236, 1255, 1272, 1302, 1318, 1299, 1238, 1140, 1022, 910, 827, 779, 758, 757, 782, 856, 
972, 1088, 1177, 1226, 1232, 1203, 1157, 1110, 1067, 1028, 993, 958, 929, 905, 892, 900, 940, 1022, 
1125, 1157, 1087, 965, 836, 783, 816, 895, 971, 1017};

uint32_t Wave[32] = {  
  1024,1122,1215,1302,1378,1440,1486,1514,1524,1514,1486,
  1440,1378,1302,1215,1122,1024,926,833,746,670,608,
  562,534,524,534,562,608,670,746,833,926
};

void ADC1_Init_2(void) {
	SYSCTL_RCGCADC_R |= 0x02;
	
	while((SYSCTL_PRADC_R&0x0002) != 0x0002){};    // good code, but not yet implemented in simulator

	ADC1_SAC_R |= 0x6;              // Hardware Averaging
	ADC1_PC_R &= ~0xF;              // 9) clear max sample rate field
  ADC1_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC1_SSPRI_R = 0x3210;          // 10) Sequencer 3 is lowest priority
  ADC1_ACTSS_R &= ~0x0008;        // 11) disable sample sequencer 3
  ADC1_EMUX_R &= ~0xF000;         // 12) seq3 is software trigger
  ADC1_SSMUX3_R &= ~0x000F;       // 13) clear SS3 field
  ADC1_SSMUX3_R += 5;    //     set channel
  ADC1_SSCTL3_R = 0x0006;         // 14) no TS0 D0, yes IE0 END0
  ADC1_IM_R &= ~0x0008;           // 15) disable SS3 interrupts
  ADC1_ACTSS_R |= 0x0008;         // 16) enable sample sequencer 3
}

//------------ADC0_InSeq3------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
uint32_t ADC1_InSeq3(void){  uint32_t result;
  ADC1_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC1_RIS_R&0x08)==0){};   // 2) wait for conversion done
    // if you have an A0-A3 revision number, you need to add an 8 usec wait here
  result = ADC1_SSFIFO3_R&0xFFF;   // 3) read result
  ADC1_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}

//********************************************************************************
// debuging profile, pick up to 7 unused bits and send to Logic Analyzer
// TExaSdisplay logic analyzer shows 5 bits 0,0,PF4,PF3,PF2,PF1,PF0 
// edit this to output which pins you use for profiling
// you can output up to 7 pins
// use for debugging profile
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
void LogicAnalyzerTask(void){
  UART0_DR_R = 0x80|GPIO_PORTF_DATA_R; // sends at 10kHz
}

// measures analog voltage on PD3
void ScopeTask(void){  // called 10k/sec
  UART0_DR_R = ADC1_InSeq3()>>4; // send ADC to TExaSdisplay
}

// ----------------------------------------------------------------------------
// ---------------- PORT D Initialization --------------------------------
// ----------------------------------------------------------------------------
//
// PD7 = U2TX
// PD6 = U2RX
// PD3 = SSI1_MOSI (to TLV5616)
// PD2 = AIN5 (Monitor DAC_OUT signal from TLV5616)
// PD1 = SSI1_FS/CS (to TLV5616)
// PD0 = SSI1_SCK (to TLV5616)

void Port_D_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x08; // activate port D
  while((SYSCTL_PRGPIO_R & 0x08)==0){}; // allow time for clock to stabilize
  // --------------- Initialize PB7 as U2TX, PB6 as U2RX -----------------------

  GPIO_PORTD_LOCK_R = 0x4C4F434B; // unlock REQUIRED for PD7
  GPIO_PORTD_CR_R |= 0xC0; // commit PD6, PD7

  GPIO_PORTD_AMSEL_R &= ~0xC0; // disable analog functionality on PD6, PD7
  GPIO_PORTD_AFSEL_R |= 0xC0; // enable alternate function on PD6, PD7
  GPIO_PORTD_DEN_R |= 0xC0; // enable digital on PD6, PD7
  // (PD6 is U2RX, PD7 is U2TX)
  GPIO_PORTD_PCTL_R =(GPIO_PORTD_PCTL_R & 0x00FFFFFF) | 0x11000000; // configure PD6, PD7 as UART
  // --------------- Initialize PD2 as AIN5 ----------------------------------

  GPIO_PORTD_DIR_R &= ~0x04; // make PD2 input
  GPIO_PORTD_AFSEL_R |= 0x04; // enable alternate function on PD2
  GPIO_PORTD_DEN_R &= ~0x04; // disable digital I/O on PD2
  GPIO_PORTD_AMSEL_R |= 0x04; // enable analog functionality on PD2
  // --------------- Initialize PD3,1,0 as SSI1 MOSI, FS & SCK ----------------

  GPIO_PORTD_AMSEL_R &= ~0x0B; // disable analog functionality on PD
  GPIO_PORTD_AFSEL_R |= 0x0B; // enable alt funct on PD3,1,0
  GPIO_PORTD_DEN_R |= 0x0B; // enable digital I/O on PD3,1,0
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0xFFFF0F00) + 0x00002022;
}
 
// This debug function initializes Timer0A to request interrupts
// at a 100 Hz frequency.  It is similar to FreqMeasure.c.
void Timer0A_Init(uint32_t period){
  volatile uint32_t delay;
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  delay = SYSCTL_RCGCTIMER_R;      // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = 0;                // configure for 32-bit timer mode
  // **** timer0A initialization ****
                                   // configure for periodic mode
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAILR_R = period-1;         // start value for 100 Hz interrupts
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 32-b, periodic, interrupts
  // **** interrupt initialization ****
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = 1<<19;              // enable interrupt 19 in NVIC
}

// ***************** Timer2A_Init ****************
// Activate Timer2 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
//          priority 0 (highest) to 7 (lowest)
// Outputs: none
void Timer2A_Init(uint32_t priority){
  SYSCTL_RCGCTIMER_R |= 0x04;   // 0) activate timer2
  TIMER2_CTL_R = 0x00000000;    // 1) disable timer2A during setup
  TIMER2_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER2_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAILR_R = 0xFFFFFFFF;  // 4) reload value
  TIMER2_TAPR_R = 0;            // 5) bus clock resolution
  TIMER2_ICR_R = 0x00000001;    // 6) clear timer2A timeout flag
  TIMER2_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|(priority<<29); // priority  
// interrupts enabled in the main program after all devices initialized
// vector number 39, interrupt number 23
  NVIC_EN0_R = 1<<23;           // 9) enable IRQ 23 in NVIC
  TIMER2_CTL_R = 0x00000001;    // 10) enable timer2A
}

uint32_t wave_pos = 0;

// Trumpet
void Timer0A_Handler(void){
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
	PF0 ^= 0x01;
	
	DAC_Out(Wave[wave_pos]);
	wave_pos = (wave_pos + 1) % 32;
}

void Timer2A_Handler(void){
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER2A timeout
  PF3 ^= 0x08;
}

void DelayWait10ms(uint32_t n){
	uint32_t volatile time;
  while(n){
    time = 727240*2/91;  // 10msec
    while(time){
	  	time--;
    }
    n--;
  }
}

void Pause_SW1(void){
  while(PF4==0x00){ 
    DelayWait10ms(10);
  }
  while(PF4==0x10){
    DelayWait10ms(10);
  }
}

int main(void){
// pick one of the following three lines, all three set PLL to 80 MHz
  //PLL_Init(Bus80MHz);              // 1) call to have no TExaS debugging
  //TExaS_SetTask(&LogicAnalyzerTask); // 2) call to activate logic analyzer
  TExaS_SetTask(&ScopeTask);       // or 3) call to activate analog scope PD3
  
  LaunchPad_Init();                  // activate port F and D
	Port_D_Init();
	Switch_Init();
  Timer0A_Init(3200);            // set up Timer0A for 1 micro-sec interrupts
	//Timer2A_Init(2);
	DAC_Init(2047);
	ST7735_InitR(INITR_REDTAB);
	ADC1_Init_2();
	
	PF2 = 0;                           // turn off LED
	
  EnableInterrupts();
	
	while(1) {
		PF1 ^= 0x02;
	}
}


