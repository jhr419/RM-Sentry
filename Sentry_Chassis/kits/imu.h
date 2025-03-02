#ifndef IMU_H_
#define IMU_H_

#include "main.h"
#include "usart.h"
#include "string.h"
#include "communication.h"
#include "typedef.h"
#include "cmsis_os.h"

typedef struct{
	fp32 Ax;
	fp32 Ay;
	fp32 Az;
	uint16_t temperature;
}acc_speed_t; //14字节

typedef struct{
	fp32 Vx;
	fp32 Vy;
	fp32 Vz;
}ang_speed_t; //12字节

typedef struct{
	fp32 roll;
	fp32 pitch;
	fp32 yaw;
}angle_t; //12字节

void imu_decode(void);
void pkt_decode(uint8_t *rawData);

void StartImuTask(void const * argument);

#endif
