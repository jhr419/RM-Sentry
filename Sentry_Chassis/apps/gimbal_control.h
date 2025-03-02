#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "typedef.h"
#include "main.h"
#include "motors.h"
#include "pid.h"
#include "bsp_rc.h"

#define GIMBAL_MAX_V


typedef struct{
	uint8_t given_state;
	
	int16_t given_angle;
	int16_t given_increment_angle;
	uint8_t given_spin_direction;
	motor_9025_t motor_9025;
	
	uint8_t type;
	uint8_t mode;
}gimbal_t;

void gimbal_control(gimbal_t* pGimbal, rc_info_t* pRC);

#endif
