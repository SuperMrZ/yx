#ifndef _YK_H
#define _YK_H

#include "main.h"
typedef struct
{
	struct
	{ 
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned char s1;
		unsigned char s2;
		unsigned char s2_last;
		unsigned char s1_last;
	}rc;
	

	
}RC_Ctl_t;

extern 	RC_Ctl_t RC_Ctl;

extern uint8_t sbus_rx_buffer[18];//����ң�������ջ�������

extern int16_t yuntai_speed_target[3];

#endif