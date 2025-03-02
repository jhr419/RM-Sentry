#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "bsp_rc.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "typedef.h"

#define CHANNEL_RANGE  660
#define CHANNEL_OFFSET 1024

void rc_to_vector(chassis_t*pChassis, rc_info_t* pRC);
void rc_to_main_gimbal(gimbal_t* pGimbal, rc_info_t* pRC);
#endif
