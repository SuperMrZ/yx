#ifndef _CAN_USER_H
#define _CAN_USER_H


 #include "main.h"
 typedef struct 
{
	uint16_t motor_id;
	uint16_t angle;
    int16_t speed;
	int16_t last_angle;
	int16_t currunt;
    
}motor_recieve;

void can_filter_init(void);
void can_filter2_init(void);
void CAN_cmd_current_3508motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_speed_3508motor(int16_t target[4], motor_recieve motor_recieve_info[4]);



void CAN_cmd_current_bodan3508(int16_t motor1);
void CAN_cmd_speed_bodan(int16_t target, motor_recieve motor_recieve_info);


void CAN_cmd_current_yaw6020(int16_t motor1);
void CAN_cmd_speed_yaw6020(int16_t target, motor_recieve motor_recieve_info);
void CAN_cma_angle_yaw6020(int16_t target, motor_recieve motor_recieve_info);

#endif
