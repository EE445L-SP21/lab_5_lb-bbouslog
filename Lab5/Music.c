
#include <stdint.h>
#include "Music.h"
#include "DAC.h"

Note_t trumpet[151];
Note_t oboe[100];

uint32_t trumpet_len = 70;

// Instrument Voices

uint32_t Oboe64[64]={
1024, 1024, 1014, 1008, 1022, 1065, 1093, 1006, 858, 711, 612, 596, 672, 806, 952, 1074, 1154, 1191, 
1202, 1216, 1236, 1255, 1272, 1302, 1318, 1299, 1238, 1140, 1022, 910, 827, 779, 758, 757, 782, 856, 
972, 1088, 1177, 1226, 1232, 1203, 1157, 1110, 1067, 1028, 993, 958, 929, 905, 892, 900, 940, 1022, 
1125, 1157, 1087, 965, 836, 783, 816, 895, 971, 1017};

uint32_t Trumpet64[64]={
987, 1049, 1090, 1110, 1134, 1160, 1139, 1092, 1070, 1042, 1035, 1029, 1008, 1066, 1150, 1170, 1087, 915, 679, 372, 151, 
558, 1014, 1245, 1260, 1145, 1063, 984, 934, 960, 1027, 1077, 1081, 1074, 1064, 1042, 1010, 974, 968, 974, 994, 1039, 
1094, 1129, 1125, 1092, 1056, 1056, 1082, 1059, 1046, 1058, 1061, 1045, 1034, 1050, 1094, 1112, 1092, 1063, 1053, 1065, 1052, 992};

volatile uint32_t trumpet_note_pos;

volatile uint32_t trumpet_note_count;

volatile uint32_t trumpet_note_period; // in micro-sec

volatile uint32_t trumpet_wave_pos;

volatile uint32_t trumpet_wave_count;

volatile uint32_t trumpet_wave_period; // in micro-sec

volatile uint16_t trumpet_duty_flag;

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
	
	trumpet_note_pos = 0;
	trumpet_note_count = 0;
	trumpet_note_period = trumpet[trumpet_note_pos].duration * 1000;
	
	trumpet_wave_pos = 0;
	trumpet_wave_count = 0;
	trumpet_wave_period = (uint32_t)(1000000.0/(trumpet[trumpet_note_pos].frequency * 64));
	
	trumpet_duty_flag = 0;
}

void oboe_init(void){

}

void Play(void){
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
	
void Rewind(void){
	
}
	
void Stop(void){
	
}

void music_handler(void) {
	if (trumpet_wave_count == 0 && trumpet_duty_flag == 0) {
		DAC_Out(1024+Trumpet64[trumpet_wave_pos]);
	}
	
	trumpet_note_count++;
	trumpet_wave_count++;
}
