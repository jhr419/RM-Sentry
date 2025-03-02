#include "chassis_task.h"
#include "CAN_in_motors.h"
#include "cmsis_os.h"
#include "car.h"
#include "motors.h"
#include "pid.h"
#include "communication.h"

extern motor_3508_t motors_3508[4];
extern car_t car;


void StartChassisTask(void const * argument)
{   
		HAL_GPIO_WritePin(GPIOG, LED_7_Pin, GPIO_PIN_RESET);
		while(1)
		{
			chassis_control(car.pChassis,car.pRC);
			osDelay(1);
		}
}
