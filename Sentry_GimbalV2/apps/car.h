#ifndef CAR_H
#define CAR_H

#include "typedef.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "bsp_rc.h"
#include "ammo_booster_control.h"
#include "auto_aim.h"
#include "imu_protocol.h"

typedef struct{
		rc_info_t* pRC;
		ammo_booster_t* pAmmoBooster;
		gimbal_t* pGimbal;
		chassis_t* pChassis;
		aim_info_t* pAim_info;
		imu_info_t* pImu_info;
}car_t;

void car_init(void);

#endif
