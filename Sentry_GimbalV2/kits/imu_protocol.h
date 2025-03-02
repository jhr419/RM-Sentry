#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H

#include "typedef.h"

typedef struct{
	union_int32 roll;
	union_int32 pitch;
	union_int32 yaw;
}imu_info_t;

void imu_decode(uint8_t* buf);

#endif
