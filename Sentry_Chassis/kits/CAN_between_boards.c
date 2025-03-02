#include "CAN_between_boards.h"
#include "main.h"
extern CAN_HandleTypeDef hcan1;

//CAN板间通信

CAN_TxHeaderTypeDef  motor_c_board_tx_message;
uint8_t              motor_c_board_can_tx_data[8];

void CAN_sned(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    motor_c_board_tx_message.StdId = CAN_C_BOARD_SEND_ID;
    motor_c_board_tx_message.IDE = CAN_ID_STD;
    motor_c_board_tx_message.RTR = CAN_RTR_DATA;
    motor_c_board_tx_message.DLC = 0x08;
    motor_c_board_can_tx_data[0] = motor1 >> 8;
    motor_c_board_can_tx_data[1] = motor1;
    motor_c_board_can_tx_data[2] = motor2 >> 8;
    motor_c_board_can_tx_data[3] = motor2;
    motor_c_board_can_tx_data[4] = motor3 >> 8;
    motor_c_board_can_tx_data[5] = motor3;
    motor_c_board_can_tx_data[6] = motor4 >> 8;
    motor_c_board_can_tx_data[7] = motor4;

    HAL_CAN_AddTxMessage(&C_BOARD_CAN, &motor_c_board_tx_message, motor_c_board_can_tx_data, &send_mail_box);
}
