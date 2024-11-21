#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "yaokong.h"
#include "power_control.h"


int16_t dipan_speedtarget[4];
int16_t dipan_x_speed,dipan_y_speed,dipan_z_speed;//底盘前后，左右，自旋速度
int8_t power_control = 3;
extern	float power_scale;

void dipan_speed_jiesuan(RC_Ctl_t RC_Ctl)
{

	int16_t a1=1,a2=1,a3=1;//速度系数
	//
	dipan_y_speed = RC_Ctl.rc.ch3-1024;
	dipan_x_speed = RC_Ctl.rc.ch2-1024;
	dipan_z_speed = RC_Ctl.rc.ch0-1024;
	

//	
//	dipan_speedtarget[0] = 	 power_scale*power_control*(a1*-dipan_y_speed - a2*dipan_x_speed  +dipan_z_speed);//右前
//	dipan_speedtarget[1] = 	 power_scale*power_control*(a1*dipan_y_speed  - a2*dipan_x_speed  +dipan_z_speed);//左前
//	dipan_speedtarget[2] = 	 power_scale*power_control*(-a1*dipan_y_speed  + a2*dipan_x_speed +dipan_z_speed);//右后
//	dipan_speedtarget[3] = 	 power_scale*power_control*(-a1*-dipan_y_speed + a2*dipan_x_speed +dipan_z_speed);//左后
	
		
	dipan_speedtarget[0] = 	 power_control*(a1*-dipan_y_speed - a2*dipan_x_speed  +dipan_z_speed);//右前
	dipan_speedtarget[1] = 	 power_control*(a1*dipan_y_speed  - a2*dipan_x_speed  +dipan_z_speed);//左前
	dipan_speedtarget[2] = 	 power_control*(-a1*dipan_y_speed  + a2*dipan_x_speed +dipan_z_speed);//右后
	dipan_speedtarget[3] = 	 power_control*(-a1*-dipan_y_speed + a2*dipan_x_speed +dipan_z_speed);//左后
	
	

}	








	

