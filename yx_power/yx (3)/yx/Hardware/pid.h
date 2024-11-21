# ifndef _PID_H
#define  _PID_H
#include "stm32f4xx.h"  


typedef struct
{
//p,i,d参数值,maxI积分限幅，maxO输出限幅
float kp;
float ki;
float kd;
int16_t maxI;  //maxI积分限幅
int16_t maxO;  //maxO输出限幅
int16_t error_now;
int16_t error_last;
float pout ;
int16_t iout;
float dout;
int16_t output;

} PID;


extern PID pid_dipan3508[4];
extern PID pid_bodan3508 ;
extern PID pid_yaw6020;

int16_t pid_output(PID *pid, int16_t feedback, int16_t target) ;



#endif
