#include "ammo_booster_task.h"
#include "ammo_booster_control.h"
#include "cmsis_os.h"
#include "car.h"
#include "motors.h"
#include "pid.h"
#include "communication.h"

extern car_t car;

void StartAmmoTriggerTask(void const * argument)
{   
	HAL_GPIO_WritePin(GPIOH, LED_R_Pin, GPIO_PIN_SET);
	while(1)
	{
		ammo_booster_control(car.pAmmoBooster,car.pRC);
		osDelay(1);
	}
}
