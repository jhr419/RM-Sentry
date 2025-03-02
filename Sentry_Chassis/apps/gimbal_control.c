#include "gimbal_control.h"
#include "CAN_in_motors.h"
#include "remote_control.h"

extern motor_6020_measure_t 	motors_6020_measure[2];
extern motor_2006_measure_t		motors_2006_measure[2];

gimbal_t gimbal;

void gimbal_control(gimbal_t* pGimbal, rc_info_t* pRC)//TODO: 如何确定控制方式，遥控器or上位机，接口如何定义
{
	rc_to_main_gimbal(pGimbal, pRC);
	
	CAN_Control9025IncrementAngle(CAN_9025_M1_ID, pGimbal->given_increment_angle);	
}
