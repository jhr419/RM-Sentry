#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "bsp_rc.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "typedef.h"
#include "ammo_booster_control.h"
#include "car.h"
#include "math.h"
#include "imu_protocol.h"

#define GIMBAL_MODE_RC 1
#define GIMBAL_MODE_HOLD 3
#define GIMBAL_MODE_AUTO_AIM 2

#define CHANNEL_RANGE  660
#define CHANNEL_OFFSET 1024

void rc_to_vector(chassis_t*pChassis, rc_info_t* pRC);
void rc_to_angle(gimbal_t* pGimbal, rc_info_t* pRC);
void rc_to_fire_state(ammo_booster_t* pAmmo_booster, rc_info_t* pRC);
#endif
