#ifndef _DAMIAO_H
#define _DAMIAO_H
#include "stm32f407xx.h"

float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void ctrl_damiao_motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void enable_damiao_motor(uint16_t id);
void disable_damiao_motor(uint16_t id);

 typedef struct 
{
	int16_t p;//λ��
	int16_t v;//�ٶ�
	int16_t t;//����
	float position;
	float velocity;
	float torque;
    
}damiao_recieve;


#endif
