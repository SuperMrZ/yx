#ifndef _DIPAN_CONTROL
#define _DIPAN_CONTROL
#include "stm32f4xx.h"  
#include "yaokong.h"
   



extern int16_t dipan_speedtarget[4];
extern int16_t dipan_speed[4];
extern int8_t power_control;
void dipan_speed_jiesuan(RC_Ctl_t RC_Ctl);



#endif
