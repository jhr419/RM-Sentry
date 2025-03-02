#include "imu.h"

#define IMU_HEADER 0x55
#define IMU_FRAME_LEN 11

#define TYPE_TIME				0x50
#define TYPE_ACC_SPEED	0x51
#define TYPE_ANG_SPEED  0x52
#define TYPE_ANGLE			0x53
#define TYPE_MAG				0x54
#define TYPE_PORT_STATE 0x55 

extern uint8_t rx1Data[128];
extern uint8_t rx2Data[128];
extern uint16_t rx1_len;
extern uint16_t rx2_len;
extern uint8_t rx1_flag;
extern uint8_t rx2_flag;

uint8_t imuData_pkt[3][11];
acc_speed_t Av;
ang_speed_t ang_v;
angle_t angle;



void imu_decode(void){
	if(rx2Data[0] == IMU_HEADER && rx2Data[IMU_FRAME_LEN] == IMU_HEADER && rx2Data[2*IMU_FRAME_LEN] == IMU_HEADER){
		memcpy(imuData_pkt[0], &rx2Data[0], IMU_FRAME_LEN);
		memcpy(imuData_pkt[1], &rx2Data[IMU_FRAME_LEN], IMU_FRAME_LEN);
		memcpy(imuData_pkt[2], &rx2Data[2*IMU_FRAME_LEN], IMU_FRAME_LEN);
	}
}

void pkt_decode(uint8_t *rawData){
	if(rawData[0] == IMU_HEADER){
		fp32 Ax;
		fp32 Ay;
		fp32 Az;
		
		fp32 Vx;
		fp32 Vy;
		fp32 Vz;
		
		fp32 roll;
		fp32 pitch;
		fp32 yaw;
		
		switch(rawData[1]){
			case TYPE_ACC_SPEED:
					Ax = (fp32)((rawData[3]<<8)|rawData[2])*16/32768;
					Ay = (fp32)((rawData[5]<<8)|rawData[4])*16/32768;
					Az = (fp32)((rawData[7]<<8)|rawData[6])*16/32768;
			
					if(Ax >= 16){
						Av.Ax = Ax - 32;
					}else{
						Av.Ax = Ax;
					}
					
					if(Ay >= 16){
						Av.Ay = Ay - 32;
					}else{
						Av.Ay = Ay;
					}
					
					if(Az >= 16){
						Av.Az = Az - 32;
					}else{
						Av.Az = Az;
					}
					
					Av.temperature = ((rawData[9]<<8)|rawData[8])/100;
				break;
			case TYPE_ANG_SPEED:
					Vx = (fp32)((rawData[3]<<8)|rawData[2])*2000/32768;
					Vy = (fp32)((rawData[5]<<8)|rawData[4])*2000/32768;
					Vz = (fp32)((rawData[7]<<8)|rawData[6])*2000/32768;
					
					if(Vx >= 2000){
						ang_v.Vx = Vx - 4000;
					} else{
						ang_v.Vx = Vx;
					}
					
					if(Vy >= 2000){
						ang_v.Vy = Vy - 4000;
					} else{
						ang_v.Vy = Vy;
					}
					
					if(Vz >= 2000){
						ang_v.Vz = Vz - 4000;
					} else{
						ang_v.Vz = Vz;
					}
				break;
			case TYPE_ANGLE:
					roll 	= (fp32)((rawData[3]<<8)|rawData[2])*180/32768;
					pitch = (fp32)((rawData[5]<<8)|rawData[4])*180/32768;
					yaw 	= (fp32)((rawData[7]<<8)|rawData[6])*180/32768;
			
					if(roll >= 180){
						angle.roll = roll-360;
					}else{
						angle.roll = roll;
					}
					
					if(pitch >= 180){
						angle.pitch = pitch-360;
					}else{
						angle.pitch = pitch;
					}
					
					if(yaw >= 180){
						angle.yaw = yaw-360;
					}else{
						angle.yaw = yaw;
					}
		}
	}
}

void StartImuTask(void const * argument){
	while(1){
		imu_decode();
		pkt_decode(imuData_pkt[0]);
		pkt_decode(imuData_pkt[1]);
		pkt_decode(imuData_pkt[2]);
		uart_tx(&huart1, (uint8_t *)&angle, 12);
		osDelay(1);
	}
}

