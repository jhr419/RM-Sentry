#include "chassis_task.h"
#include "cmsis_os.h"
#include "car.h"
#include "motors.h"
#include "pid.h"
#include "communication.h"

extern motor_3508_t motors_3508[4];
extern car_t car;


void StartChassisTask(void const * argument)
{   
//		HAL_GPIO_WritePin(GPIOG, LED_7_Pin, GPIO_PIN_RESET);
		while(1)
		{
			chassis_control(car.pChassis,car.pRC);
//			uart7_printf("RF set rpm:%d  RF rpm:%d  out:%d \n",	car.pChassis->RF->given_speed_rpm,
//																car.pChassis->RF->hmotor_3508_measure->speed_rpm,
//																car.pChassis->RF->pid->out);
			osDelay(1);
		}
}
