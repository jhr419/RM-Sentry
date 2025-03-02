#include "referee_system_communication.h"
#include "communication.h"
#include "typedef.h"
#include "car.h"
referee_system_into_t referee_system_into;
power_heat_data_t power_heat_data;

//TODO 
//void referee_system_info_init(){
//	referee_system_into.power_heat_data = &power_heat_data;
//}

extern car_t car;
void decode_referee_system_data(uint8_t *data){
	uint16_t data_len = (uint16_t)data[2]<<8 | data[1];
	uint16_t cmd_id = (uint16_t)data[6]<<8 | data[5];

	//TODO: you should add other information
	switch(cmd_id){
		case CMD_ID_RF_CHASSIS_POWER_AND_MUZZLE_HEAT:
			//TODO: how to process the order of bytes
			memcpy(car.pReferee_system_into->power_heat_data, &data[MSG_LEN_HEADER_REFEREE_SYS+2], data_len);
			break;
		default:
			break;
	}
}
