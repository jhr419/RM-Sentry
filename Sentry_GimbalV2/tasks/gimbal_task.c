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
	car.pGimbal->yaw->given_ecd = car.pGimbal->yaw->hmotor_6020_measure->ecd;
	car.pGimbal->pitch->given_ecd = car.pGimbal->pitch->hmotor_6020_measure->ecd;
	osDelay(500);
	while(1)
	{
		gimbal_control(car.pGimbal,car.pRC);
		osDelay(1);
	}
}
