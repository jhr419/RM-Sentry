#include "CAN_in_motors.h"
#include "main.h"
#include "motors.h"
extern CAN_HandleTypeDef hcan1;

/**********************************************************************/
/*motor paramaters instances*/
extern motor_3508_measure_t		motors_3508_measure[4];
extern motor_6020_measure_t 	motors_6020_measure[2];
extern motor_2006_measure_t		motors_2006_measure[2];

extern motor_9025_measure_t 	motor_9025;
extern motor_9025_pid_t 		motor_9025_pid;
extern motor_9025_ecd_data_t 	motor_9025_ecd_data;
extern motor_9025_t motors_9025;

CAN_TxHeaderTypeDef  motor_3508_tx_message;
uint8_t              motor_3508_can_tx_data[8];

CAN_TxHeaderTypeDef  motor_6020_tx_message;
uint8_t              motor_6020_can_tx_data[8];

CAN_TxHeaderTypeDef  motor_2006_tx_message;
uint8_t              motor_2006_can_tx_data[8];

CAN_TxHeaderTypeDef  motor_9025_tx_message;
uint8_t              motor_9025_can_tx_data[8];

/**********************************************************************/
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


//    can_filter_st.SlaveStartFilterBank = 14;
//    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}



//motor CAN potocol send

//3508motor
void CAN_Control3508Current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_3508_tx_message.StdId = CAN_3508_SEND_ID;
    motor_3508_tx_message.IDE = CAN_ID_STD;
    motor_3508_tx_message.RTR = CAN_RTR_DATA;
    motor_3508_tx_message.DLC = 0x08;
    motor_3508_can_tx_data[0] = motor1 >> 8;
    motor_3508_can_tx_data[1] = motor1;
    motor_3508_can_tx_data[2] = motor2 >> 8;
    motor_3508_can_tx_data[3] = motor2;
    motor_3508_can_tx_data[4] = motor3 >> 8;
    motor_3508_can_tx_data[5] = motor3;
    motor_3508_can_tx_data[6] = motor4 >> 8;
    motor_3508_can_tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&MOTOR_3508_CAN, &motor_3508_tx_message, motor_3508_can_tx_data, &send_mail_box);
}

//6020motor
void CAN_Control6020Voltage(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_6020_tx_message.StdId = CAN_6020_SEND_ID;
    motor_6020_tx_message.IDE = CAN_ID_STD;
    motor_6020_tx_message.RTR = CAN_RTR_DATA;
    motor_6020_tx_message.DLC = 0x08;
    motor_6020_can_tx_data[0] = motor1 >> 8;
    motor_6020_can_tx_data[1] = motor1;
    motor_6020_can_tx_data[2] = motor2 >> 8;
    motor_6020_can_tx_data[3] = motor2;
    motor_6020_can_tx_data[4] = motor3 >> 8;
    motor_6020_can_tx_data[5] = motor3;
    motor_6020_can_tx_data[6] = motor4 >> 8;
    motor_6020_can_tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&MOTOR_6020_CAN, &motor_6020_tx_message, motor_6020_can_tx_data, &send_mail_box);
}

//2006motor
void CAN_Control2006Current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_2006_tx_message.StdId = CAN_2006_SEND_ID;
    motor_2006_tx_message.IDE = CAN_ID_STD;
    motor_2006_tx_message.RTR = CAN_RTR_DATA;
    motor_2006_tx_message.DLC = 0x08;
    motor_2006_can_tx_data[0] = motor1 >> 8;
    motor_2006_can_tx_data[1] = motor1;
    motor_2006_can_tx_data[2] = motor2 >> 8;
    motor_2006_can_tx_data[3] = motor2;
    motor_2006_can_tx_data[4] = motor3 >> 8;
    motor_2006_can_tx_data[5] = motor3;
    motor_2006_can_tx_data[6] = motor4 >> 8;
    motor_2006_can_tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&MOTOR_2006_CAN, &motor_2006_tx_message, motor_2006_can_tx_data, &send_mail_box);
}

//9025motor
//control state of 9025: CMD_9025_SHUTDOWN, CMD_9025_STOP, CMD_9025_START
void CAN_Manage9025State(uint32_t id,uint8_t CMD)
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = CMD;//CMD_9025_SHUTDOWN CMD_9025_STOP CMD_9025_START
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = 0x00;
    motor_9025_can_tx_data[3] = 0x00;
    motor_9025_can_tx_data[4] = 0x00;
    motor_9025_can_tx_data[5] = 0x00;
    motor_9025_can_tx_data[6] = 0x00;
    motor_9025_can_tx_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

void CAN_Read9025CircleAngle(uint32_t id)
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = 0x94;
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = 0x00;
    motor_9025_can_tx_data[3] = 0x00;
    motor_9025_can_tx_data[4] = 0x00;
    motor_9025_can_tx_data[5] = 0x00;
    motor_9025_can_tx_data[6] = 0x00;
    motor_9025_can_tx_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}

void CAN_Control9025CiecleAngle(uint32_t id, uint8_t spinDirection, uint32_t maxSpeed, int32_t angleControl)
{
    uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = 0xA6;
    motor_9025_can_tx_data[1] = spinDirection;
    motor_9025_can_tx_data[2] = ((uint8_t *)&maxSpeed)[0]; // 访问 maxSpeed 的第一个字节
    motor_9025_can_tx_data[3] = ((uint8_t *)&maxSpeed)[1]; // 访问 maxSpeed 的第二个字节
    motor_9025_can_tx_data[4] = ((uint8_t *)&angleControl)[0]; // 访问 angleControl 的第一个字节
    motor_9025_can_tx_data[5] = ((uint8_t *)&angleControl)[1]; // 访问 angleControl 的第二个字节
    motor_9025_can_tx_data[6] = 0x00;
    motor_9025_can_tx_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);	
}

void CAN_Control9025IncrementAngle(uint32_t id, int32_t angleControl)//TODO MAX SPEED确定
{
	uint32_t send_mail_box;
    motor_9025_tx_message.StdId = id;
    motor_9025_tx_message.IDE   = CAN_ID_STD;
    motor_9025_tx_message.RTR   = CAN_RTR_DATA;
    motor_9025_tx_message.DLC   = 0x08;

    motor_9025_can_tx_data[0] = 0xA7;
    motor_9025_can_tx_data[1] = 0x00;
    motor_9025_can_tx_data[2] = 0x00;
    motor_9025_can_tx_data[3] = 0x00;
    motor_9025_can_tx_data[4] = * (uint8_t*)(&angleControl);
    motor_9025_can_tx_data[5] = *((uint8_t*)(&angleControl)+1);
    motor_9025_can_tx_data[6] = *((uint8_t*)(&angleControl)+2);
    motor_9025_can_tx_data[7] = *((uint8_t*)(&angleControl)+3);
    HAL_CAN_AddTxMessage(&MOTOR_9025_CAN, &motor_9025_tx_message, motor_9025_can_tx_data, &send_mail_box);
}
/*------------------------------------------------------------------------*/
/*
	CAN recive interrupt
*/

uint8_t rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
   
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
		case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_3508_measure(&motors_3508_measure[i], rx_data);
            break;
        }
				
		case CAN_6020_M1_ID:
        case CAN_6020_M2_ID:
		{
			static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_6020_M1_ID;
            get_motor_6020_measure(&motors_6020_measure[i], rx_data);
            break;
		}
				
		case CAN_2006_M1_ID:
        case CAN_2006_M2_ID:        
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_2006_M1_ID;
            get_motor_2006_measure(&motors_2006_measure[i], rx_data);
            break;
        }
				
		case CAN_9025_M1_ID:
		{
			get_motor_9025_measure(&motor_9025, rx_data);
			break;
		}
				
        default: break;	
    }
}

/*------------------------------------------------------------------------*/
/*
	3508ָ��
*/

const motor_3508_measure_t *get_motor_3508_measure_point(uint8_t i)
{
    return &motors_3508_measure[(i & 0x03)];
}

/*
	6020ָ��
*/

const motor_6020_measure_t *get_motor_6020_measure_point(uint8_t i)
{
    return &motors_6020_measure[(i & 0x03)];
}


/*------------------------------------------------------------------------*/
/*
	2006ָ��
*/

const motor_2006_measure_t *get_motor_2006_measure_point(uint8_t i)
{
    return &motors_2006_measure[(i & 0x03)];
}

/*------------------------------------------------------------------------*/
/*
	9025ָ��
*/

const motor_9025_measure_t *get_motor_9025_measure_point(void)
{
    return &motor_9025;
}

const motor_9025_pid_t *get_motor_9025_pid_point(void)
{
    return &motor_9025_pid;
}

const motor_9025_ecd_data_t* get_motor_9025_ecd_data_point(void)
{
    return &motor_9025_ecd_data; 
}

