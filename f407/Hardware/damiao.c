#include "stm32f407xx.h"
#include "damiao.h"
#include "can.h"




float	uint_to_float(uint16_t x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
  
		
    uint16_t max_input = (1 << (bits)) - 1; // 2^(y-1) - 1 
		uint16_t	x_int1 = x_int;
		float   out = ((float)x_int / (float)max_input) * (x_max - x_min) + x_min;
    return  out;
}


int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits)-1 ) / span));
}



static CAN_TxHeaderTypeDef  damiao_tx_message;//�������ݵ�����ͷ
static uint8_t              damiao_can_send_data[8];//Ҫ���͵���������
void ctrl_damiao_motor( uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    // ���ݹ�һ����ת��Ϊ�޷�������
    pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);//ע�����е��õ�float_to_uint�����������������ɵ�������϶���
    vel_tmp = float_to_uint(_vel, -30, 30, 12);    //������Ļᵼ�����Ľ������
    kp_tmp = float_to_uint(_KP, 0, 500, 12);
    kd_tmp = float_to_uint(_KD, 0, 5, 12);
    tor_tmp = float_to_uint(_torq, -10, 10, 12);

    // CAN ����֡����
		uint32_t send_mail_box;
    damiao_tx_message.StdId = id;//����C620�ֲᣬIDΪ1-4ʱ���ͱ�ʶΪ0x200
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;
    // ���ݷ��䵽 CAN ����֡
    damiao_can_send_data[0] = (pos_tmp >> 8);
    damiao_can_send_data[1] = pos_tmp;
    damiao_can_send_data[2] = (vel_tmp >> 4);
    damiao_can_send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    damiao_can_send_data[4] = kp_tmp;
    damiao_can_send_data[5] = (kd_tmp >> 4);
    damiao_can_send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    damiao_can_send_data[7] = tor_tmp;
		
		HAL_CAN_AddTxMessage(&hcan2, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}


void enable_damiao_motor(uint16_t id)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

   
    // CAN ����֡����
		uint32_t send_mail_box;
    damiao_tx_message.StdId = id;
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;
    // ���ݷ��䵽 CAN ����֡
    damiao_can_send_data[0] = 0xFF;
    damiao_can_send_data[1] = 0xFF;
    damiao_can_send_data[2] = 0xFF;
    damiao_can_send_data[3] = 0xFF;
    damiao_can_send_data[4] = 0xFF;
    damiao_can_send_data[5] = 0xFF;
    damiao_can_send_data[6] = 0xFF;
    damiao_can_send_data[7] = 0xFC;
		
		HAL_CAN_AddTxMessage(&hcan2, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}

void disable_damiao_motor(uint16_t id)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

   
    // CAN ����֡����
		uint32_t send_mail_box;
    damiao_tx_message.StdId = id;//����C620�ֲᣬIDΪ1-4ʱ���ͱ�ʶΪ0x200
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;
    // ���ݷ��䵽 CAN ����֡
    damiao_can_send_data[0] = 0xFF;
    damiao_can_send_data[1] = 0xFF;
    damiao_can_send_data[2] = 0xFF;
    damiao_can_send_data[3] = 0xFF;
    damiao_can_send_data[4] = 0xFF;
    damiao_can_send_data[5] = 0xFF;
    damiao_can_send_data[6] = 0xFF;
    damiao_can_send_data[7] = 0xFD;
		
		HAL_CAN_AddTxMessage(&hcan2, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}
