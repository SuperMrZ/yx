#include "stm32f4xx.h"                  // Device header
#include "usart.h"
#include "yaokong.h" 
#include "can_user.h"
#include "damiao.h"
#include "main.h"
#include "tim.h"

extern 	RC_Ctl_t RC_Ctl;

extern uint8_t sbus_rx_buffer[18];//声明遥控器接收缓存数组


extern motor_recieve motor_recieve_yuntai3508[3];

extern int16_t fashe_speed[4];

extern int8_t yaokongjishi; 

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
				yaokongjishi=10;	
				RC_Ctl.rc.s1_last = RC_Ctl.rc.s1;
				RC_Ctl.rc.s2_last = RC_Ctl.rc.s2;
				
        RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;
				
				RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;   
    			
				RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;
					
				RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;
					
				RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
				RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);
				
//				RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis        
//				RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis      
//				RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis         
//				RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press      
//				RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press 
//				RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			//!< KeyBoard value
	
		if(RC_Ctl.rc.s2 ==2)
		{
			disable_damiao_motor(0x01);
			
		}
		if(RC_Ctl.rc.s2 ==3)
		{

				enable_damiao_motor(0x01);

		}
		
		if(RC_Ctl.rc.s1 ==1)
		{
			//开启摩擦轮
			fashe_speed[0]=500;
			fashe_speed[1]=500;
			fashe_speed[2]=500;
		}
		
		if(RC_Ctl.rc.s1 ==3)
		{
	
			CAN_cmd_current_3508motor(0,0,0,0);
			
		}

		if(RC_Ctl.rc.s1 ==2)
		{
			
			//CAN_cmd_current_3508motor(0,0,0,0);
			fashe_speed[0]=0;
			fashe_speed[1]=0;
			fashe_speed[2]=0;
		}		
		
				
				
				HAL_UARTEx_ReceiveToIdle_DMA(&huart3,sbus_rx_buffer,18);	
				__HAL_DMA_DISABLE_IT(huart3.hdmarx ,DMA_IT_HT );

				
	}


		void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == (&htim2))
    {
        yaokongjishi--;
			if(yaokongjishi<=0)
			{
				RC_Ctl.rc.ch0=1024;
				RC_Ctl.rc.ch1=1024;
				RC_Ctl.rc.ch2=1024;
				RC_Ctl.rc.ch3=1024;
				RC_Ctl.rc.s1=2;
				RC_Ctl.rc.s2=2;
				yaokongjishi=0;
	
			}
    }
}