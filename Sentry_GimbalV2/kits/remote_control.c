#include "remote_control.h"
extern car_t car;
void rc_to_vector(chassis_t* pChassis, rc_info_t* pRC)
{
	//TODO: 如何确定mode
	pChassis->mode = pRC->rc.s[1];
	
	fp32 max_speed = pChassis->max_v;
	
	fp32 ch_offsety = -(pRC->rc.ch[2]);
	fp32 ch_offsetx =  (pRC->rc.ch[3]);
	fp32 ch_offsetw =  (pRC->rc.ch[0]);
	
	
	fp32 normalized_v = ((sqrt(ch_offsetx*ch_offsetx + ch_offsety*ch_offsety) / CHANNEL_RANGE) > 1) ?
						1 : (sqrt(ch_offsetx*ch_offsetx + ch_offsety*ch_offsety) / CHANNEL_RANGE);
	
	//0是比例向量长度，1是与y轴夹角，这里前方是y通道3 右方是x通道2
	pChassis->given_chassis_v_vector[0] = CHASSIS_MAX_V * normalized_v;
	pChassis->given_chassis_v_vector[1] = atan2(ch_offsety, ch_offsetx);
	pChassis->given_chassis_w			= CHASSIS_MAX_W * ch_offsetw;
}

void rc_to_angle(gimbal_t* pGimbal, rc_info_t* pRC)
{
	pGimbal->yaw->last_angle = car.pImu_info->yaw.data;
	pGimbal->pitch->last_angle = car.pImu_info->roll.data;
	
	aim_info_t* aim_info;
	imu_info_t* imu_info;
	
	aim_info = car.pAim_info;
	imu_info = car.pImu_info;
	
	//上1 中3 下2
	pGimbal->mode = pRC->rc.s[0];
	
	int16_t ch_offsetYaw 	= pRC->rc.ch[0];
	int16_t ch_offsetPitch = pRC->rc.ch[1];
	//todo pitch/roll
	switch(pGimbal->mode){
		case GIMBAL_MODE_RC:
			if(ch_offsetYaw > 0) 
				pGimbal->yaw->given_angle = pGimbal->yaw->last_angle - 10;
			else if(ch_offsetYaw < 0) 
				pGimbal->yaw->given_angle = pGimbal->yaw->last_angle + 10;
			
			//todo pitch roll
			if(ch_offsetPitch > 0) 
				pGimbal->pitch->given_angle = pGimbal->pitch->last_angle + 6;
			else if(ch_offsetPitch < 0) 
				pGimbal->pitch->given_angle = pGimbal->pitch->last_angle - 3;
			break;
		case GIMBAL_MODE_HOLD:
			pGimbal->yaw->given_angle = imu_info->yaw.data;
			pGimbal->pitch->given_angle = imu_info->roll.data;
			break;
		
		case GIMBAL_MODE_AUTO_AIM:
			pGimbal->yaw->given_angle = (int16_t)(aim_info->target_yaw.data);
			pGimbal->pitch->given_angle = (int16_t)(aim_info->target_pitch.data);
			break;
		default:
			break;
	}
}

void rc_to_fire_state(ammo_booster_t* pAmmo_booster, rc_info_t* pRC)
{
	pAmmo_booster->mode = pRC->rc.s[1];
}
