#include "imu_protocol.h"
#include <stdarg.h>
#include <stdio.h>
#include "string.h"

imu_info_t imu_info;

void imu_decode(uint8_t* buf){	
	memcpy(&imu_info.roll.bytes[0], buf+6+28, 4);
	memcpy(&imu_info.pitch.bytes[0], buf+6+32, 4);	
	memcpy(&imu_info.yaw.bytes[0], buf+6+36, 4);
	
	imu_info.roll.data =imu_info.roll.data/1000 + 180;
	imu_info.pitch.data =imu_info.pitch.data/1000 + 180;
	imu_info.yaw.data =imu_info.yaw.data/1000 + 180;
}