#include "math.h"
#include "main.h"
#include "can_user.h"
#include "math.h"

char cap_state = 0;
extern uint16_t TIM4_Cnt1;
extern uint8_t time;
extern int16_t target[4];
extern int16_t dipan_speedtarget[4];
//uint16_t rrr=0;
float powww=0;

float power_scale=1;
float temp=0;

void chassis_power_control(motor_recieve motor_dipan3508[4],int16_t currnt[4])
{
	int max_power_limit =20;
	float chassis_max_power = 0.0f;
	float input_power = 0.0f;		 
	float initial_give_power[4]; 
	float initial_total_power = 0;
	float scaled_give_power[4];
	float chassis_power = 0.0f;
	float chassis_power_buffer = 0.0f;
	float toque_coefficient = 1.996889e-6f; // (20/16384)*(0.3)*(187/3591)/9.55 = 力矩/Icmd/9.55
	float k1 = 1.23e-07;					// k1
	float k2 = 1.453e-07;					// k2
	float constant = 4.081f-1.2f;
  
	//get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
	//PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);
	//get_chassis_max_power(&max_power_limit);
	
  input_power = max_power_limit;
	
	chassis_max_power=input_power;	
	
	
	
	for (char i = 0; i < 4; i++) 
	{
		initial_give_power[i] = toque_coefficient * motor_dipan3508[i].speed * motor_dipan3508[i].currunt+
								k2 * motor_dipan3508[i].speed * motor_dipan3508[i].speed +
								k1 * motor_dipan3508[i].currunt * motor_dipan3508[i].currunt + constant;

		if (initial_give_power < 0){continue;}
		
		initial_total_power += initial_give_power[i];
		powww=initial_total_power;
	}//计算底盘总功率

	uint8_t time_flag=0;
	time_flag=TIM4_Cnt1%10;
//&&(time_flag==0)
	if((power_scale<1)&&(TIM4_Cnt1%20==0))
	{
	power_scale+=0.02;
	}
	//&&(time_flag==0)
	if (initial_total_power>chassis_max_power)
	{
		//power_scale = chassis_max_power/initial_total_power;
		power_scale-=0.01;
//		char t=0;
//		for(t=0;t<4;t++)
//		{
//			dipan_speedtarget[t]=power_scale*dipan_speedtarget[t];
//		}
//		
		//rrr=1;
		char i=0;
		for (i=0;i<4;i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale;
			
			
      float a=  k1;
			float b = toque_coefficient*motor_dipan3508[i].speed;
			float c = k2 * motor_dipan3508[i].speed * motor_dipan3508[i].speed - scaled_give_power[i] + constant;
			
			if (motor_dipan3508[i].currunt > 0) 
			{
				
				temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)
				{
					currnt[i]=motor_dipan3508[i].currunt= 16000;
				}
				else
					currnt[i] =motor_dipan3508[i].currunt= temp;
			}
			else
			{
			 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
				  currnt[i] =motor_dipan3508[i].currunt= -16000;
				}
				else
			    currnt[i] =motor_dipan3508[i].currunt= temp;
	    }
    }
  }
	//if(rrr==0)
	//{
		CAN_cmd_current_3508motor(currnt[0],currnt[1],currnt[2],currnt[3]);
	//}
	//if(rrr==1)
	//{CAN_cmd_current_3508motor(0,0,0,0);}

}
