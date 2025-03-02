#include "gimbal_task.h"
#include "cmsis_os.h"
#include "car.h"
#include "motors.h"
#include "pid.h"
#include "communication.h"

extern car_t car;

void StartGimbalTask(void const * argument)
{   
	HAL_GPIO_WritePin(GPIOG, LED_8_Pin, GPIO_PIN_RESET);
	while(1)
	{
		gimbal_control(car.pGimbal, car.pRC);
		osDelay(1);
	}
}
