#ifndef AMMO_BOOSTER_CONTROL_H
#define AMMO_BOOSTER_CONTROL_H

#include "typedef.h"
#include "motors.h"
#include "pid.h"
#include "bsp_rc.h"

#define PI 	   3.14159265358979323846l
#define ROOT_2 1.41421356237309504880l
#define ECD_RANGE 8192

//TODO:
#define GIMBAL_MOTOR_MAX_RPM 1200

#define SINGLE_SHOT 2
#define TRIPE_SHOT 3
#define AUTO 1

#define OPEN_FIRE 	1
#define READY_FIRE 	3
#define CEASE_FIRE  2

#define LIMIT(value, max, min) ((value) > (max) ? (max) : ((value) < (min) ? (min) : (value)))

#define BOOSTER_RPM 10000
#define TRIGGER_RPM 16000

typedef struct{
	motor_3508_t* trigger_l;
	motor_3508_t* trigger_r;
	motor_2006_t* booster;
	
	uint8_t fire;
	uint8_t mode;//单发，三连发，全自动
}ammo_booster_t;

void ammo_booster_init(uint8_t mode, fp32 TriggerPID[3], fp32 BoosterPID[3], fp32 trigger_max_out, fp32 trigger_max_iout, fp32 booster_max_out, fp32 booster_max_iout);
void ammo_booster_control(ammo_booster_t* pAmmo_booster, rc_info_t* pRC);
#endif
