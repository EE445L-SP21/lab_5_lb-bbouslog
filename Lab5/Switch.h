// Switch.h

#ifndef _SWITCHH_
#define _SWITCHH_

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/CortexM.h"

// ----------------------------------------------------------------------------
// ---------------- PORT C Initialization --------------------------------
// ----------------------------------------------------------------------------
//
// PC4 = UP switch
// PC5 = RIGHT switch
// PC6 = LEFT switch
// PC7 = DOWN switch
//
void Switch_Init(void);

// ------------ Switch_Data --------------------------------------
//
// Sets current and last data for the switch
// inputs: none
// outputs: none
//
void Switch_Data(void);

uint32_t get_Last_Data(void);

uint32_t get_Curr_Data(void);

#endif
