#include "stm32f4xx.h"                  // Device header

typedef struct
{
//p,i,d����ֵ,maxI�����޷���maxO����޷�
float kp;
float ki;
float kd;
int16_t maxI;  //maxI�����޷�
int16_t maxO;  //maxO����޷�
int16_t error_now;
int16_t error_last;
float pout ;
int16_t iout;
float dout;
int32_t output;

} PID;




//��ͬ�����pid����
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
  * @brief  pid_output�˺����������һ��pid���
  * @param  kp,ki,kd,maxI,maxO,�ֱ�ָpid�㷨������targetΪ����ﵽ��Ŀ��ֵ��feedbackΪ��ǰĿ���ֵ������ֵ��
  * @note   ע��feedback��target�ı��������÷���Ҫ����
  * @retval ��
  */
  


int32_t pid_output(PID *pid, int16_t feedback, int16_t target) 
{
    // �������
    pid->error_last = pid->error_now;
    pid->error_now = target - feedback;

    int16_t a = pid->error_now;
    int16_t b = pid->error_last;

    // ����P����
    int16_t pout = pid->kp * pid->error_now;
	pid->pout = pout;

    // ���㲢����I����
    pid->iout += (pid->ki * pid->error_now);
    int16_t c = pid->iout;        
    if (pid->iout > pid->maxI) {
        pid->iout = pid->maxI;
    }
	else if (pid->iout < - pid->maxI) {
        pid->iout =  - pid->maxI;
    }


    // ����D����
    int16_t dout = pid->kd * (pid->error_now - pid->error_last);
	pid->dout = dout;

    // �������������
    int32_t output = pout - dout + pid->iout;
//    if (output > pid->maxO) {
//        output = pid->maxO;
//    }
	pid->output = output;

    return output;
}




