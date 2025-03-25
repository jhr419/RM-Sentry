#ifndef MOTORS_H
#define MOTORS_H

#include "typedef.h"
#include "pid.h"

typedef struct
{
	uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
}motor_3508_measure_t;

typedef struct
{
	uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
}motor_6020_measure_t;

typedef struct
{
	uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
}motor_2006_measure_t;

typedef struct 
{
  int8_t temperate;
  int16_t iq;
  int16_t speed;
  uint16_t ecd;
}motor_9025_measure_t;

typedef struct 
{
  uint8_t anglePidKp;
  uint8_t anglePidKi;
  uint8_t speedPidKp;
  uint8_t speedPidKi;
  uint8_t iqPidKp;
  uint8_t iqPidKi;
}motor_9025_pid_t;

typedef struct
{
  uint16_t encoder;
  uint16_t encoderRaw;
  uint16_t encoderOffset;
}motor_9025_ecd_data_t;

typedef struct
{
	motor_3508_measure_t* hmotor_3508_measure;
	fp32 pid_coefficient[3];
	pid_t* pid;
	int16_t given_speed_rpm;
	int16_t given_current;
}motor_3508_t;

typedef struct{
	motor_6020_measure_t* hmotor_6020_measure;
	fp32 gyroPid_coefficient[3];
	fp32 ecdPid_coefficient[3];
	fp32 rpmPid_coefficient[3];
	
	pid_t* gyroPid;
	pid_t* ecdPid;
	pid_t* rpmPid;
	
	int16_t last_angle;
	//feedback
	int16_t fb_angle;
	
	//set value
	int16_t given_angle;
	uint16_t given_ecd;
	int16_t given_speed_rpm;
	
	uint16_t hold_ecd;
}motor_6020_t;

typedef struct{
	motor_2006_measure_t* hmotor_2006_measure;
	fp32 pid_coefficient[3];
	pid_t* pid;
	int16_t given_speed_rpm;
	int16_t given_current;
}motor_2006_t;

motor_3508_t* motors_3508_init(uint8_t mode, fp32 PID3508[3], fp32 max_out, fp32 max_iout);
motor_6020_t* motors_6020_init(	uint8_t mode, 
																fp32 YawGyroPID[3], fp32 PitchGyroPID[3], 
																fp32 YawEcdPID[3], fp32 PitchEcdPID[3], 
																fp32 YawRpmPID[3], fp32 PitchRpmPID[3],

																fp32 Yaw_gyro_max_out, fp32 Yaw_gyro_max_iout,
																fp32 Pitch_gyro_max_out, fp32 Pitch_gyro_max_iout,

																fp32 Yaw_ecd_max_out, fp32 Yaw_ecd_max_iout,
																fp32 Pitch_ecd_max_out, fp32 Pitch_ecd_max_iout,

																fp32 Yaw_Rpm_max_out, fp32 Yaw_Rpm_max_iout,
																fp32 Pitch_Rpm_max_out, fp32 Pitch_Rpm_max_iout);
motor_3508_t* motors_ammo_init(uint8_t mode, fp32 PID[3], fp32 max_out, fp32 max_iout);
motor_2006_t* motors_2006_init(uint8_t mode, fp32 PID[3], fp32 max_out, fp32 max_iout);


#endif
