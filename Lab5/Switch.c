// Switch.c

#include "Switch.h"

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/CortexM.h"

uint32_t last_data;
uint32_t curr_data;

void Switch_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x04; // Activate clock for Port C
  while((SYSCTL_PRGPIO_R & 0x04) != 0x04){}; // Allow time for clock to start

  GPIO_PORTC_PCTL_R &= ~0xFFFF0000; // regular GPIO
  GPIO_PORTC_AMSEL_R &= ~0xF0; // disable analog function
  GPIO_PORTC_DIR_R &= ~0xF0; // inputs on PC7-PC4
  GPIO_PORTC_AFSEL_R &= ~0xF0; // regular port function
  GPIO_PORTC_PUR_R = 0xF0; // enable pull-up on PC7-PC4
  GPIO_PORTC_DEN_R |= 0xF0; // enable digital port
}

void Switch_Data(void) {
	last_data = curr_data;
	curr_data = GPIO_PORTC_DATA_R & 0xF0;
}

uint32_t get_Last_Data(void) {
	return last_data;
}

uint32_t get_Curr_Data(void) {
	return curr_data;
}

