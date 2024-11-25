#include "stm32f4xx.h"                  // Device header

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
int32_t output;

} PID;




//不同电机的pid参数
PID pid_dipan3508[4] = {
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0},
    {10, 0.01, 0, 0x3000, 0x3000, 0, 0,0,0,0,0}
};
extern PID pid_dipan3508[4];

PID pid_bodan3508={20,0.15,0.15,0x5000,0x5000,0,0,0,0};
PID pid_yaw6020={8, 0.15, 0,0x1000, 0x5000, 0, 0,0,0};
PID pid_yaw6020_angle={20,0.1,5,0x3000, 0x5000, 0, 0,0,0};
 
/**
  * @brief  pid_output此函数用于输出一个pid输出
  * @param  kp,ki,kd,maxI,maxO,分别指pid算法参数，target为你想达到的目标值，feedback为当前目标的值（反馈值）
  * @note   注意feedback与target的变量类型置否需要更改
  * @retval 无
  */
  


int32_t pid_output(PID *pid, int16_t feedback, int16_t target) 
{
    // 更新误差
    pid->error_last = pid->error_now;
    pid->error_now = target - feedback;

    int16_t a = pid->error_now;
    int16_t b = pid->error_last;

    // 计算P部分
    int16_t pout = pid->kp * pid->error_now;
	pid->pout = pout;

    // 计算并限制I部分
    pid->iout += (pid->ki * pid->error_now);
    int16_t c = pid->iout;        
    if (pid->iout > pid->maxI) {
        pid->iout = pid->maxI;
    }
	else if (pid->iout < - pid->maxI) {
        pid->iout =  - pid->maxI;
    }


    // 计算D部分
    int16_t dout = pid->kd * (pid->error_now - pid->error_last);
	pid->dout = dout;

    // 计算输出并限制
    int32_t output = pout - dout + pid->iout;
//    if (output > pid->maxO) {
//        output = pid->maxO;
//    }
	pid->output = output;

    return output;
}




