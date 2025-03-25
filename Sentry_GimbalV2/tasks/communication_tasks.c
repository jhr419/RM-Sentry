#include "communication_tasks.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "communication.h"
#include "car.h"
#include "imu_protocol.h"
#include "auto_aim.h"
extern car_t car;
extern imu_info_t imu_info;
void StartCommunicationTask(void const * argument)
{   
	uart_dma_rx_enable();
	while(1)
	{
//		uart1_printf("%d,%d\n", car.pAmmoBooster->booster->given_speed_rpm,car.pAmmoBooster->booster->hmotor_2006_measure->speed_rpm);
//		auto_aim_send();
		osDelay(1);
	}
}
