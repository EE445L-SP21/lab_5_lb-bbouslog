
#ifndef _MUSICH_
#define _MUSICH_

#include <stdint.h>

typedef struct
{
	uint32_t amplitude; // 0 to 1024
	double   frequency; // In Hz
	uint32_t duration;  // In ms
	uint32_t duty;      // 0 to 100 percent
	
	uint32_t * voice; // Shape of the note
} Note_t;

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
	
// Song Init Methods
	
void trumpet_init(void);

void oboe_init(void);
	
void Play(void);
	
void Rewind(void);
	
void Stop(void);

void music_handler(void);

#endif
