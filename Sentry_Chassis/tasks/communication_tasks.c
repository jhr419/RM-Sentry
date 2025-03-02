#include "communication_tasks.h"
#include "communication.h"
#include "communication.h"
#include "cmsis_os.h"
#include "car.h"
#include "typedef.h"
#include "main.h"
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart6;
extern car_t car;

void StartCommunicationTask(void const * argument)
{   
	uart_dma_rx_enable();
	while(1)
	{
		#ifdef CHASSIS
			encode_send_data(&huart7, (uint8_t*)car.pRC, MSG_CMD_BOARD_RC, sizeof(rc_info_t));
			uart_printf(&huart6, "%.1f, %.1f\n", car.pReferee_system_into->power_heat_data->chassis_power.data, 100.0);
		#endif
		osDelay(1);
	}
}
