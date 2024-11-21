#include "stm32f4xx.h"                  // Device header
#include "can.h"
#include "pid.h"
#include "power_control.h"
#include "can_user.h"
#include "yaokong.h"

extern motor_recieve motor_recieve_dipan3508[4];

extern PID pid_dipan3508[4];
extern 	RC_Ctl_t RC_Ctl;
extern motor_recieve motor_recieve_dipan6020;
extern motor_recieve motor_reciver_bodan3508;




/**
  * @brief  can_filter_init �˺���Ϊ���������ú�����ĿǰΪȫͨ
  * @param  ��
  * @retval ��
  */

//can���������ú���
//void can_filter_init(void)
//{

//  CAN_FilterTypeDef can_filter_st;          
//	for(uint8_t FilterBank=0;FilterBank<14;FilterBank++)
//	{
//    can_filter_st.FilterActivation = ENABLE;   
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//    can_filter_st.FilterIdHigh = 0x0000;
//    can_filter_st.FilterIdLow = 0x0000;
//    can_filter_st.FilterMaskIdHigh = 0x0000;
//    can_filter_st.FilterMaskIdLow = 0x0000;
//    can_filter_st.FilterBank = FilterBank;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//	  can_filter_st.SlaveStartFilterBank = 14;         
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//	}

//		HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//}

void can_filter_init(void)
{
	HAL_CAN_Start(&hcan1);

  CAN_FilterTypeDef can_filter_st;          
	for(uint8_t FilterBank=0;FilterBank<14;FilterBank++)
	{
    can_filter_st.FilterActivation = ENABLE;   
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = FilterBank;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  can_filter_st.SlaveStartFilterBank = 14;         
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	}

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


	  HAL_CAN_Start(&hcan2);

    CAN_FilterTypeDef can_filter_st2;            
    can_filter_st2.FilterActivation = ENABLE;     
    can_filter_st2.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st2.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st2.FilterIdHigh = 0x0000;
    can_filter_st2.FilterIdLow = 0x0000;
    can_filter_st2.FilterMaskIdHigh = 0x0000;
    can_filter_st2.FilterMaskIdLow = 0x0000;
    can_filter_st2.FilterBank = 14;
    can_filter_st2.FilterFIFOAssignment = CAN_RX_FIFO1;
	  can_filter_st2.SlaveStartFilterBank = 14;          
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st2);

	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

}



/**
  * @brief  HAL_CAN_RxFifo0MsgPendingCallback�˺���Ϊfifo0�Ľ����жϺ����������յ����ĺ󣬵��ô��жϺ���
  * @param  
  * @return ����ֵ
  * @retval ��
  * @note   Ŀǰ����ֻ����һ������ĽǶ����ٶ���Ϣ�������ص��id�������Ľ�
  * @note   �����ڼ��������ٶ�speed������Ƕ�angle�������������������ã������Ľ�
  */

//can���������жϻص�����

  uint8_t rx_data[8]; 

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    //uint8_t rx_data[8];  // ����ÿ֡���ݳ������Ϊ8���ֽ�
 
    // ��CAN����FIFO0��ȡ��Ϣ
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    // �ɹ����յ�CAN��Ϣ
    // ������������洢���յ�������


	if(rx_header.StdId==0x200)
	{
		RC_Ctl.rc.ch0=(rx_data[0] << 8) |rx_data[1];
		RC_Ctl.rc.ch1=(rx_data[2] << 8) |rx_data[3];
		RC_Ctl.rc.ch2=(rx_data[4] << 8) |rx_data[5];
		RC_Ctl.rc.ch3=(rx_data[6] << 8) |rx_data[7];
	}

		else if(rx_header.StdId==0x123)
		{
			RC_Ctl.rc.s1=rx_data[0];
		  RC_Ctl.rc.s1_last=rx_data[1];
		  RC_Ctl.rc.s2=rx_data[2];
		  RC_Ctl.rc.s2_last=rx_data[3];
		}

		else if(rx_header.StdId==0x20B)
		{
		motor_recieve_dipan6020.motor_id=rx_header.StdId;
		motor_recieve_dipan6020.angle =(rx_data[0] << 8) |rx_data[1];// ʾ���������һ���ֽں͵ڶ����ֽ�Ϊ����Ƕ�����
		motor_recieve_dipan6020.speed= (rx_data[2] << 8) |rx_data[3];  // ʾ��������������ֽ�Ϊ���ת������
		motor_recieve_dipan6020.currunt = (rx_data[4] << 8) |rx_data[5];
		}
		else 
			
		{
		motor_reciver_bodan3508.motor_id=rx_header.StdId;
		motor_reciver_bodan3508.angle =(rx_data[0] << 8) |rx_data[1];// ʾ���������һ���ֽں͵ڶ����ֽ�Ϊ����Ƕ�����
		motor_reciver_bodan3508.speed= (rx_data[2] << 8) |rx_data[3];  // ʾ��������������ֽ�Ϊ���ת������
		motor_reciver_bodan3508.currunt = (rx_data[4] << 8) |rx_data[5];
		}
		
		

	


}








/**
  * @brief  can_filter2_init �˺���Ϊ���������ú�����ĿǰΪȫͨ
  * @param  ��
  * @retval ��
  */

//can���������ú���
//void can_filter2_init(void)
//{
//    CAN_FilterTypeDef can_filter_st;
//    can_filter_st.FilterActivation = ENABLE;           // ���ù�����
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  // ������ģʽΪ��ʶ������ģʽ
//    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;  // ���������س���Ϊ32λ
//    can_filter_st.FilterIdHigh = 0x0000;               // ��ʶ����λ�������ˣ�
//    can_filter_st.FilterIdLow = 0x0000;                // ��ʶ����λ�������ˣ�
//    can_filter_st.FilterMaskIdHigh = 0x0000;           // �����λ�������ˣ�
//    can_filter_st.FilterMaskIdLow = 0x0000;            // �����λ�������ˣ�
//    can_filter_st.FilterBank = 14;                      // ���������
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1; // �������洢��FIFO0
//    can_filter_st.SlaveStartFilterBank = 14;            // ��CANʵ������ʼ���������У����ڵ�CANʵ�������壩
//    
//    // ����CAN������
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    
//    // ����CAN������
//    HAL_CAN_Start(&hcan2);
//    
//    // ����CAN�����ж�֪ͨ
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
//}


/**
  * @brief  HAL_CAN_RxFifo1MsgPendingCallback�˺���Ϊfifo1�Ľ����жϺ����������յ����ĺ󣬵��ô��жϺ���
  * @param  
  * @return ����ֵ
  * @retval ��
  * @note   Ŀǰ����ֻ����һ������ĽǶ����ٶ���Ϣ�������ص��id�������Ľ�
  * @note   �����ڼ��������ٶ�speed������Ƕ�angle�������������������ã������Ľ�
  */

//can���������жϻص�����


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];  // ����ÿ֡���ݳ������Ϊ8���ֽ�
 
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_data);
    // �ɹ����յ�CAN��Ϣ
    // ������������洢���յ�������
	

		int16_t i =rx_header.StdId -0x201;
		
		motor_recieve_dipan3508[i].motor_id=rx_header.StdId;
		motor_recieve_dipan3508[i].angle =(rx_data[0] << 8) |rx_data[1];// ʾ���������һ���ֽں͵ڶ����ֽ�Ϊ����Ƕ�����
		motor_recieve_dipan3508[i].speed= (rx_data[2] << 8) |rx_data[3];  // ʾ��������������ֽ�Ϊ���ת������
		motor_recieve_dipan3508[i].currunt = (rx_data[4] << 8) |rx_data[5];
	
	
}




/**
  * @brief  CAN_cmd_3508motor�˺������ڿ��Ƶ����ĸ�3508������������
  * @param  motor1-4ָ��������ָ����Ӧ����ĵ�������ֵ������Ϊint_16�Σ�����Ӧ���޸ģ�
  * @retval ��
  */
static CAN_TxHeaderTypeDef  chassis_tx_message;//�������ݵ�����ͷ
static uint8_t              chassis_can_send_data[8];//Ҫ���͵���������
void CAN_cmd_current_3508motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;//����C620�ֲᣬIDΪ1-4ʱ���ͱ�ʶΪ0x200
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8; //id1��� ���õ���ֵ��8λ
    chassis_can_send_data[1] = motor1;      //id1��� ���õ���ֵ��8λ
    chassis_can_send_data[2] = motor2 >> 8; //id2��� ���õ���ֵ��8λ
    chassis_can_send_data[3] = motor2;      //id2��� ���õ���ֵ��8λ
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
 
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}






/**
  * @brief  CAN_cmd_speed_3508motor�˺������ڿ��Ƶ����ĸ�35098������ٶ�
  * @param  motor1-4ָ��������ָ����Ӧ������ٶ����������Ϊint_16�Σ�����Ӧ���޸ģ�
  * @retval ��
  */



void CAN_cmd_speed_3508motor(int16_t target[4], motor_recieve motor_recieve_info[4])
{

	int16_t motor_currnt[4];
	
	for (uint16_t i = 0; i < 4; i++) 
	{
	
    motor_currnt[i] = pid_output(&pid_dipan3508[i], motor_recieve_info[i].speed, target[i]);
    }

	//chassis_power_control(motor_recieve_dipan3508, motor_currnt);
		CAN_cmd_current_3508motor(motor_currnt[0],motor_currnt[1],motor_currnt[2],motor_currnt[3]);

}








