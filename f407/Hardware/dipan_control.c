#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "yaokong.h"
#include "power_control.h"


int16_t dipan_speedtarget[4];
float yuntai_locationtarget[2]={4000,-1.5};



int16_t dipan_x_speed,dipan_y_speed,yaw_speed;//底盘前后，左右，自旋速度
int8_t power_control = 10;
extern	float power_scale;

void dipan_speed_jiesuan(RC_Ctl_t RC_Ctl)
{

	int16_t a1=1,a2=1,a3=1;//速度系数
	//
	dipan_y_speed = RC_Ctl.rc.ch3-1024;
	dipan_x_speed = RC_Ctl.rc.ch2-1024;
	yaw_speed = RC_Ctl.rc.ch0-1024;
	
	if(RC_Ctl.rc.s2==1)
	{

	yuntai_locationtarget[1] =0.001*(RC_Ctl.rc.ch1-1024)-1.5;
	}
	
	if(RC_Ctl.rc.s2==3)
	{
	yuntai_locationtarget[0] -=0.06*(yaw_speed);
	yuntai_locationtarget[1] =0.001*((float)(RC_Ctl.rc.ch1-1024))-1.5;	
	
	
		if(yuntai_locationtarget[0]>8190)
		{
			yuntai_locationtarget[0]=10;
		}
		if(yuntai_locationtarget[0]<5)
		{
			yuntai_locationtarget[0]=8190;
		}
		
		
		if(yuntai_locationtarget[1]>=-1)
		{
			yuntai_locationtarget[1]=-1;
		}
		else if(yuntai_locationtarget[1]<=-2)
		{
			yuntai_locationtarget[1]=-2;
		}
	
  }

}	








	

