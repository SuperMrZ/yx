#ifndef _POWER_CONTROL_H
#define _POWER_CONTROL_H

#include "main.h"
#include "can_user.h"

void chassis_power_control(motor_recieve motor_dipan3508[4],int16_t currnt[4]);
extern	float power_scale;

#endif
