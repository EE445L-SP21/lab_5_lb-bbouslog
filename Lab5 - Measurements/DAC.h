// DAC.h

#ifndef _DACH_
#define _DACH_

#include <stdint.h>

// ------------ DAC_Init --------------------------------------
//
// Initialize TLV5616 12-bit DAC
// assumes bus clock is 80 MHz
// inputs: initial voltage output (0 to 4095)
// outputs: none
//
void DAC_Init(uint16_t data);

// ------------------- DAC_Out -------------------------------------
//
// Send data to TLV5616 12-bit DAC
// inputs: voltage output (0 to 4095)
// outputs: return parameter from SSI (not meaningful)
//
void DAC_Out(uint16_t code);

#endif
