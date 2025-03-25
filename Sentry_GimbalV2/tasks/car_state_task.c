#include "car_state_task.h"
#include "car.h"
#include "cmsis_os.h"
extern car_t car;

void StartCarStateTask(void const * argument)
{
		while(1)
		{
//				uint8_t state = car.pRC->sw2;
//				if(state == 1) 			{HAL_GPIO_WritePin(GPIOG, LED_1_Pin, GPIO_PIN_RESET);	HAL_GPIO_WritePin(GPIOG, LED_2_Pin|LED_3_Pin, GPIO_PIN_SET);}
//				else if(state == 2) {HAL_GPIO_WritePin(GPIOG, LED_2_Pin, GPIO_PIN_RESET);	HAL_GPIO_WritePin(GPIOG, LED_1_Pin|LED_3_Pin, GPIO_PIN_SET);}
//				else if(state == 3) {HAL_GPIO_WritePin(GPIOG, LED_3_Pin, GPIO_PIN_RESET);	HAL_GPIO_WritePin(GPIOG, LED_1_Pin|LED_2_Pin, GPIO_PIN_SET);}
			osDelay(1);
		}
}
