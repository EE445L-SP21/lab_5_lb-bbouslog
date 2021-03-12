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

typedef struct
{
	uint32_t amplitude; // 0 to 1024
	double   frequency; // In Hz
	uint32_t duration;  // In ms
	uint32_t duty;      // 0 to 100 percent
} Note_t;

Note_t trumpet[151];
Note_t oboe[100];

uint32_t trumpet_len = 148;

// Cut Time Note Lengths (ms)

#define Wd 1500
#define W  1000
#define Hd 750
#define H  500
#define Qd 375
#define Q  250
#define Ed 187.5
#define E  125
#define Sd 93.75
#define S  62.5
#define Td 46.875
#define T  31.25

// Note Frequencies

#define C_3  130.81
#define Cs_3 138.59
#define D_3  146.83
#define Ds_3 155.56
#define E_3  164.81
#define F_3  174.61
#define Fs_3 185
#define G_3  196
#define Gs_3 207.65
#define A_3  220
#define As_3 233.08
#define B_3  246.94

#define C_4  261.63
#define Cs_4 277.18
#define D_4  293.66
#define Ds_4 311.13
#define E_4  329.63
#define F_4  349.23
#define Fs_4 369.99
#define G_4  392
#define Gs_4 415.3
#define A_4  440
#define As_4 466.16
#define B_4  493.88

#define C_5  523.25
#define Cs_5 554.37
#define D_5  587.33
#define Ds_5 622.25
#define E_5  659.26
#define F_5  698.46
#define Fs_5 739.99
#define G_5  783.99
#define Gs_5 830.61
#define A_5  880
#define As_5 932.33
#define B_5  987.77

#define Rest 0

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
	
void trumpet_init(void){
	trumpet[0] = (Note_t) {1024, B_3, Q, 90};
	trumpet[1] = (Note_t) {1024, E_4, Q, 90};
	trumpet[2] = (Note_t) {1024, B_3, Q, 90};
	trumpet[3] = (Note_t) {1024, E_4, Q, 90};
	
	trumpet[4] = (Note_t) {1024, B_3, E, 95};
	trumpet[5] = (Note_t) {1024, E_4, Q, 90};
	trumpet[6] = (Note_t) {1024, B_3, E, 90};
	trumpet[7] = (Note_t) {1024, Rest, E, 0};
	trumpet[8] = (Note_t) {1024, As_3, E, 90};
	trumpet[9] = (Note_t) {1024, B_3, Q, 90};
	
	trumpet[10] = (Note_t) {1024, B_3, E, 90};
	trumpet[11] = (Note_t) {1024, As_3, E, 90};
	trumpet[12] = (Note_t) {1024, B_3, E, 90};
	trumpet[13] = (Note_t) {1024, A_3, E, 90};
	trumpet[14] = (Note_t) {1024, Rest, E, 0};
	trumpet[15] = (Note_t) {1024, Gs_3, E, 90};
	trumpet[16] = (Note_t) {1024, A_3, E, 90};
	trumpet[17] = (Note_t) {1024, G_3, E, 90};
	
	trumpet[18] = (Note_t) {1024, G_3, Qd, 90};
	trumpet[19] = (Note_t) {1024, E_3, E, 95};
	trumpet[20] = (Note_t) {1024, E_3, H, 90};
	
	trumpet[21] = (Note_t) {1024, B_3, Q, 90};
	trumpet[22] = (Note_t) {1024, E_4, Q, 90};
	trumpet[23] = (Note_t) {1024, B_3, Q, 90};
	trumpet[24] = (Note_t) {1024, E_4, Q, 90};
	
	trumpet[25] = (Note_t) {1024, B_3, E, 95};
	trumpet[26] = (Note_t) {1024, E_4, Q, 90};
	trumpet[27] = (Note_t) {1024, B_3, E, 90};
	trumpet[28] = (Note_t) {1024, Rest, E, 0};
	trumpet[29] = (Note_t) {1024, As_3, E, 90};
	trumpet[30] = (Note_t) {1024, B_3, Q, 90};
	
	trumpet[31] = (Note_t) {1024, A_3, E, 90};
	trumpet[32] = (Note_t) {1024, Rest, E, 0};
	trumpet[33] = (Note_t) {1024, A_3, Q, 95};
	trumpet[34] = (Note_t) {1024, A_3, E, 90};
	trumpet[35] = (Note_t) {1024, Gs_3, E, 90};
	trumpet[36] = (Note_t) {1024, A_3, Q, 90};
	
	trumpet[37] = (Note_t) {1024, D_4, E, 90};
	trumpet[38] = (Note_t) {1024, C_4, Q, 90};
	trumpet[39] = (Note_t) {1024, B_3, Q, 90};
	trumpet[40] = (Note_t) {1024, A_3, Qd, 90};
	
	trumpet[41] = (Note_t) {1024, B_3, Q, 90};
	trumpet[42] = (Note_t) {1024, E_4, Q, 90};
	trumpet[43] = (Note_t) {1024, B_3, Q, 90};
	trumpet[44] = (Note_t) {1024, E_4, Q, 90};
	
	trumpet[45] = (Note_t) {1024, B_3, E, 95};
	trumpet[46] = (Note_t) {1024, E_4, Q, 90};
	trumpet[47] = (Note_t) {1024, B_3, E, 90};
	trumpet[48] = (Note_t) {1024, Rest, E, 0};
	trumpet[49] = (Note_t) {1024, As_3, E, 90};
	trumpet[50] = (Note_t) {1024, B_3, Q, 90};
	
	trumpet[51] = (Note_t) {1024, D_4, E, 90};
	trumpet[52] = (Note_t) {1024, Rest, E, 0};
	trumpet[53] = (Note_t) {1024, D_4, Q, 95};
	trumpet[54] = (Note_t) {1024, D_4, E, 90};
	trumpet[55] = (Note_t) {1024, B_3, E, 90};
	trumpet[56] = (Note_t) {1024, A_3, Q, 90};
	
	trumpet[57] = (Note_t) {1024, G_3, Qd, 90};
	trumpet[58] = (Note_t) {1024, E_3, E, 95};
	trumpet[59] = (Note_t) {1024, E_3, H, 90};
	
	trumpet[60] = (Note_t) {1024, E_3, H, 90};
	trumpet[61] = (Note_t) {1024, G_3, H, 90};
	
	trumpet[62] = (Note_t) {1024, B_3, H, 90};
	trumpet[63] = (Note_t) {1024, D_4, H, 90};
	
	trumpet[64] = (Note_t) {1024, Fs_4, Q, 90};
	trumpet[65] = (Note_t) {1024, E_4, Q, 90};
	trumpet[66] = (Note_t) {1024, As_3, E, 95};
	trumpet[67] = (Note_t) {1024, B_3, Q, 90};
	trumpet[68] = (Note_t) {1024, G_3, E, 90};
	
	trumpet[69] = (Note_t) {1024, Rest, W, 0};
	
	trumpet[70] = (Note_t) {1024, Rest, E, 0};
	trumpet[71] = (Note_t) {1024, B_3, Q, 90};
	trumpet[72] = (Note_t) {1024, G_3, E, 90};
	trumpet[73] = (Note_t) {1024, B_3, E, 90};
	trumpet[74] = (Note_t) {1024, Rest, E, 0};
	trumpet[75] = (Note_t) {1024, Rest, Q, 0};
	
	trumpet[76] = (Note_t) {1024, Rest, E, 0};
	trumpet[77] = (Note_t) {1024, B_3, Q, 90};
	trumpet[78] = (Note_t) {1024, G_3, E, 90};
	trumpet[79] = (Note_t) {1024, B_3, E, 90};
	trumpet[80] = (Note_t) {1024, Rest, E, 0};
	trumpet[81] = (Note_t) {1024, Rest, Q, 0};
	
	trumpet[82] = (Note_t) {1024, Rest, E, 0};
	trumpet[83] = (Note_t) {1024, B_3, Q, 90};
	trumpet[84] = (Note_t) {1024, G_3, E, 90};
	trumpet[85] = (Note_t) {1024, As_3, E, 90};
	trumpet[86] = (Note_t) {1024, B_3, Q, 90};
	trumpet[87] = (Note_t) {1024, G_3, E, 95};
	
	trumpet[88] = (Note_t) {1024, G_3, Qd, 90};
	trumpet[89] = (Note_t) {1024, E_3, E, 95};
	trumpet[90] = (Note_t) {1024, E_3, H, 90};
	
	trumpet[91] = (Note_t) {1024, Rest, E, 0};
	trumpet[92] = (Note_t) {1024, B_3, Q, 90};
	trumpet[93] = (Note_t) {1024, G_3, E, 90};
	trumpet[94] = (Note_t) {1024, B_3, E, 90};
	trumpet[95] = (Note_t) {1024, Rest, E, 0};
	trumpet[96] = (Note_t) {1024, Rest, Q, 0};
	
	trumpet[97] = (Note_t) {1024, Rest, E, 0};
	trumpet[98] = (Note_t) {1024, B_3, Q, 90};
	trumpet[99] = (Note_t) {1024, G_3, E, 90};
	trumpet[100] = (Note_t) {1024, B_3, E, 90};
	trumpet[101] = (Note_t) {1024, Rest, E, 0};
	trumpet[102] = (Note_t) {1024, Rest, Q, 0};
	
	trumpet[103] = (Note_t) {1024, Rest, E, 0};
	trumpet[104] = (Note_t) {1024, B_3, Q, 90};
	trumpet[105] = (Note_t) {1024, G_3, E, 90};
	trumpet[106] = (Note_t) {1024, As_3, E, 90};
	trumpet[107] = (Note_t) {1024, B_3, Q, 90};
	trumpet[108] = (Note_t) {1024, A_3, E, 95};
	
	trumpet[109] = (Note_t) {1024, A_3, H, 95};
	trumpet[110] = (Note_t) {1024, D_3, H, 90};
	
	trumpet[111] = (Note_t) {1024, Rest, E, 0};
	trumpet[112] = (Note_t) {1024, B_3, Q, 90};
	trumpet[113] = (Note_t) {1024, G_3, E, 90};
	trumpet[114] = (Note_t) {1024, B_3, E, 90};
	trumpet[115] = (Note_t) {1024, Rest, E, 0};
	trumpet[116] = (Note_t) {1024, Rest, Q, 0};
	
	trumpet[117] = (Note_t) {1024, Rest, E, 0};
	trumpet[118] = (Note_t) {1024, B_3, Q, 90};
	trumpet[119] = (Note_t) {1024, G_3, E, 90};
	trumpet[120] = (Note_t) {1024, B_3, E, 90};
	trumpet[121] = (Note_t) {1024, Rest, E, 0};
	trumpet[122] = (Note_t) {1024, Rest, Q, 0};
	
	trumpet[123] = (Note_t) {1024, Rest, E, 0};
	trumpet[124] = (Note_t) {1024, B_3, Q, 90};
	trumpet[125] = (Note_t) {1024, G_3, E, 90};
	trumpet[126] = (Note_t) {1024, As_3, E, 90};
	trumpet[127] = (Note_t) {1024, B_3, Q, 90};
	trumpet[128] = (Note_t) {1024, G_3, E, 95};
	
	trumpet[129] = (Note_t) {1024, G_3, Qd, 90};
	trumpet[130] = (Note_t) {1024, E_3, E, 95};
	trumpet[131] = (Note_t) {1024, E_3, H, 90};
	
	trumpet[132] = (Note_t) {1024, C_3, E, 90};
	trumpet[133] = (Note_t) {1024, E_3, E, 95};
	trumpet[134] = (Note_t) {1024, G_3, Q, 90};
	trumpet[135] = (Note_t) {1024, Cs_3, E, 90};
	trumpet[136] = (Note_t) {1024, E_3, E, 95};
	trumpet[137] = (Note_t) {1024, G_3, Q, 90};
	
	trumpet[138] = (Note_t) {1024, As_3, E, 90};
	trumpet[139] = (Note_t) {1024, B_3, Q, 90};
	trumpet[140] = (Note_t) {1024, E_3, (H+E+E), 90};
	
	trumpet[141] = (Note_t) {1024, G_3, E, 90};
	trumpet[142] = (Note_t) {1024, C_4, E, 90};
	trumpet[143] = (Note_t) {1024, E_4, E, 90};
	trumpet[144] = (Note_t) {1024, As_3, E, 90};
	trumpet[145] = (Note_t) {1024, B_3, Q, 90};
	trumpet[146] = (Note_t) {1024, G_3, (Q+Hd), 90};
	
	trumpet[147] = (Note_t) {1024, Rest, Q, 0};
}

void apply_volume(uint32_t *new_wave, uint32_t *old_wave, uint32_t wave_len, uint32_t volume){
	if (volume > 1024) { volume = 1024; }
	
	for (int i = 0; i < wave_len; i++) {
		new_wave[i] = (old_wave[i] - 1024)*(volume/1024) + 1024;
	}
}

volatile uint32_t trumpet_note_pos;

volatile uint32_t trumpet_note_count;

volatile uint32_t trumpet_note_period; // in micro-sec

volatile uint32_t trumpet_wave_pos;

volatile uint32_t trumpet_wave_count;

volatile uint32_t trumpet_wave_period; // in micro-sec

volatile uint16_t trumpet_duty_flag;

uint32_t * inst_ptr = Trumpet64;

uint32_t switch_curr;
uint32_t switch_last;

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
  UART0_DR_R = (ADC1_InSeq3()>>4); // send ADC to TExaSdisplay
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

uint32_t MaxElapsedTime0;

// Trumpet
void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
	
	if (trumpet_wave_count == 0 && trumpet_duty_flag == 0) {
		DAC_Out(1024+inst_ptr[trumpet_wave_pos]);
		PF2 ^= 0x04;
	}
	
	trumpet_note_count++;
	trumpet_wave_count++;
}

void Timer2A_Handler(void){
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER2A timeout
  PF3 ^= 0x08;
}

void Play(void) {
	if (trumpet_note_count >= trumpet_note_period) {
		trumpet_note_pos = (trumpet_note_pos + 1) % trumpet_len;
		trumpet_note_count = 0;
		trumpet_note_period = trumpet[trumpet_note_pos].duration * 1000;
		
		trumpet_wave_pos = 0;
		trumpet_wave_count = 0;
		
		if (trumpet[trumpet_note_pos].frequency == 0) { // Rest
			trumpet_duty_flag = 1;
			DAC_Out(1024);
		}
		else { // Regular Note
			trumpet_wave_period = (uint32_t)(1000000.0/(trumpet[trumpet_note_pos].frequency * 64));
			trumpet_duty_flag = 0;
		}
	}
	else {
		if (trumpet_wave_count >= trumpet_wave_period) {
			trumpet_wave_count = 0;
			trumpet_wave_pos = (trumpet_wave_pos + 1) % 64;
		}
		if (trumpet_note_count >= (trumpet_note_period * trumpet[trumpet_note_pos].duty)/100) {
			trumpet_duty_flag = 1;
			DAC_Out(1024);
		}
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
  Timer0A_Init(80);            // set up Timer0A for 1 micro-sec interrupts
	//Timer2A_Init(2);
	DAC_Init(2047);
	ST7735_InitR(INITR_REDTAB);
	ADC1_Init_2();
	
	PF2 = 0;                           // turn off LED
	
	trumpet_init();
	
	trumpet_note_pos = 0;
	trumpet_note_count = 0;
	trumpet_note_period = trumpet[trumpet_note_pos].duration * 1000;
	
	trumpet_wave_pos = 0;
	trumpet_wave_count = 0;
	trumpet_wave_period = (uint32_t)(1000000.0/(trumpet[trumpet_note_pos].frequency * 64));
	
	trumpet_duty_flag = 0;
	
	uint16_t play_flag = 1;
	
  EnableInterrupts();
	
	while(1) {
		PF1 ^= 0x02;
		Switch_Data();
		switch_curr = get_Curr_Data();
		switch_last = get_Last_Data();
		
		if (switch_curr != switch_last && switch_last == 0xF0) {
			if (switch_curr == 0xD0) { // RIGHT - Play
				play_flag = 1;
			}
			else if (switch_curr == 0xB0) { // LEFT - Rewind
				play_flag = 0;
				
				trumpet_note_pos = 0;
				trumpet_note_count = 0;
				trumpet_note_period = trumpet[trumpet_note_pos].duration * 1000;
				
				trumpet_wave_pos = 0;
				trumpet_wave_count = 0;
				trumpet_wave_period = (uint32_t)(1000000.0/(trumpet[trumpet_note_pos].frequency * 64));
				
				trumpet_duty_flag = 0;
			}
			else if (switch_curr == 0x70) { // DOWN - Stop
				play_flag = 0;
			}
			else if (switch_curr == 0xE0) {
				if (inst_ptr == Trumpet64) { inst_ptr = Oboe64; }
				else { inst_ptr = Trumpet64; }
			}
		}
		
		if (play_flag == 1) {
			Play();
		}
	}
}


