#ifndef CAN_IN_MOTORS_H
#define CAN_IN_MOTORS_H

#define CMD_9025_SHUTDOWN 0x80
#define CMD_9025_STOP 0x81
#define CMD_9025_START 0x88

#define MOTOR_3508_CAN hcan1
#define MOTOR_6020_CAN hcan1
#define MOTOR_2006_CAN hcan1
#define MOTOR_9025_CAN hcan1

#include "typedef.h"
#include "motors.h"
#include "main.h"

//some multi-used function macros
#define get_motor_3508_measure(ptr, data)   														\
    {                                       														\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#define get_motor_6020_measure(ptr, data)   														\
    {                                       														\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
#define get_motor_2006_measure(ptr, data)   														\
    {                                       														\
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
#define get_motor_9025_measure(ptr, data)   														\
    {                                       														\
        (ptr)->temperate = (int8_t)   (data)[1];                    		\
        (ptr)->iq        = (int16_t)  ( (data)[3]<<8 | (data)[2] ); 		\
        (ptr)->speed     = (int16_t)  ( (data)[5]<<8 | (data)[4] ); 		\
        (ptr)->ecd       = (uint16_t) ( (data)[7]<<8 | (data)[6] ); 		\
    }

#define get_motor_9025_pid(ptr, data)       														\
    {                                       														\
        (ptr)->anglePidKp = (uint8_t)(data)[2]; 												\
        (ptr)->anglePidKi = (uint8_t)(data)[3]; 												\
        (ptr)->speedPidKp = (uint8_t)(data)[4]; 												\
        (ptr)->speedPidKi = (uint8_t)(data)[5]; 												\
        (ptr)->iqPidKp    = (uint8_t)(data)[6]; 												\
        (ptr)->iqPidKi    = (uint8_t)(data)[7]; 												\
    }

#define get_motor_9025_ecd_data(ptr, data)															\
    {   																																\
        (ptr)->encoder       = (uint16_t) ( (data)[3]<<8 | (data)[2] ); \
        (ptr)->encoderRaw    = (uint16_t) ( (data)[5]<<8 | (data)[4] ); \
        (ptr)->encoderOffset = (uint16_t) ( (data)[7]<<8 | (data)[6] ); \
    }

typedef enum
{
		CAN_3508_SEND_ID = 0x200,
		CAN_3508_M1_ID = 0x201,
		CAN_3508_M2_ID = 0x202,
		
		CAN_6020_SEND_ID = 0x1FF,
		CAN_6020_M1_ID = 0x205,
		CAN_6020_M2_ID = 0x206,
		
		CAN_2006_SEND_ID = 0x1FF,
		CAN_2006_M1_ID = 0x207,
//		CAN_2006_M2_ID = 0x208,
		
		CAN_MF_SEND_ID = 0x140,
		CAN_9025_M1_ID = 0x141
}can_msg_id_e;
		
void can_filter_init(void);

void CAN_Control3508Current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void CAN_Control6020Voltage(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void CAN_Control2006Current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void CAN_Control9025AngleIncrement(uint32_t id, uint32_t maxSpeed, int32_t angleIncrement);

const motor_3508_measure_t *get_motor_3508_measure_point(uint8_t i);

const motor_6020_measure_t *get_motor_6020_measure_point(uint8_t i);

const motor_2006_measure_t *get_motor_2006_measure_point(uint8_t i);

const motor_9025_measure_t *get_motor_9025_measure_point(void);

const motor_9025_pid_t *get_motor_9025_pid_point(void);

const motor_9025_ecd_data_t* get_motor_9025_ecd_data_point(void);

#endif
