#include "protocol.h"
#include "communication.h"
#include "car.h"

extern UART_HandleTypeDef huart1;
extern car_t car;

void send_rc_data(void)
{
	uart_tx(&huart1, (uint8_t*)&car.pRC->rc,sizeof(car.pRC->rc));
}
