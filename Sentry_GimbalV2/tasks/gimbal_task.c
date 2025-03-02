#include "gimbal_task.h"
#include "gimbal_control.h"
#include "cmsis_os.h"
#include "car.h"
#include "motors.h"
#include "pid.h"
#include "communication.h"

extern motor_6020_t motors_6020[3];
extern car_t car;


void StartGimbalTask(void const * argument)
{   
	HAL_GPIO_WritePin(GPIOH, LED_B_Pin, GPIO_PIN_SET);
	osDelay(500);
	car.pGimbal->given_pose_angle_yaw = (int16_t)(car.pImu_info->yaw.data);
	car.pGimbal->given_pose_angle_pitch = (int16_t)(car.pImu_info->roll.data);
	osDelay(500);
	while(1)
	{
		gimbal_control(car.pGimbal,car.pRC);
		osDelay(1);
	}
}
