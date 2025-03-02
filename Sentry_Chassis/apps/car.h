#ifndef CAR_H
#define CAR_H

#include "typedef.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "auto_aim.h"
#include "bsp_rc.h"
#include "chassis_communication.h"
#include "communication.h"
#include "referee_system_communication.h"

typedef struct{
		rc_info_t* 		pRC;
		gimbal_t* 		pGimbal;
		chassis_t* 		pChassis;
		chassis_info_t* pChassis_info;
		aim_info_t*		pAim_info;
		referee_system_into_t* pReferee_system_into;
}car_t;

void car_init(void);

#endif
