#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "typedef.h"
#include "motors.h"
#include "pid.h"
#include "bsp_rc.h"

#define PI 	   3.14159265358979323846l
#define ROOT_2 1.41421356237309504880l
#define MOTOR_DISTANCE_TO_CENTER 274.86l
#define RIDIUS 72.0l
#define RAD_TO_DEGREE 2*PI/360

//TODO 确定一组合适的最大值
#define CHASSIS_MAX_V 7000.0f
#define CHASSIS_MAX_W 20.0f

#define MCNAMM 	 1
#define OMNI   	 2
#define STEERING 3

#define FOLLOW_CHASSIS 1
#define FOLLOW_GIMBAL  2

#define MODE_RC 	1
#define MODE_OFFSET 3
#define MODE_UPC	2

#define LIMIT(value, max, min) ((value) > (max) ? (max) : ((value) < (min) ? (min) : (value)))

typedef struct{
		motor_3508_t* LF;
		motor_3508_t* RF;
		motor_3508_t* RB;
		motor_3508_t* LB;
	
		pid_t* pid;
	
		fp32 given_chassis_v_vector[2];
		fp32 given_chassis_w;
		fp32 given_chassis_radians;
	
		uint16_t max_v;
		uint16_t max_w;
		uint16_t max_p;
		
		uint8_t type;
		uint8_t mode;
}chassis_t;

void chassis_init(uint8_t mode,fp32 PID3508[3], fp32 max_out, fp32 max_iout);

void chassis_v_to_wheels_rpm(chassis_t* pChassis);

void chassis_control(chassis_t* pChassis, rc_info_t* pRC);
#endif
