#include "remote_control.h"
#include "CAN_in_motors.h"
#include "car.h"
#include "gimbal_control.h"
#include "math.h"

extern car_t car;

void rc_to_vector(chassis_t* pChassis, rc_info_t* pRC)
{
	//TODO: 如何确定mode
	pChassis->mode = pRC->sw2;
	
	fp32 max_speed = pChassis->max_v;
	
	fp32 ch_offsety;  
	fp32 ch_offsetx; 
	fp32 ch_offsetw; 
	fp32 normalized_v;
	
	if(pChassis->mode == MODE_RC){
		ch_offsety  = -(pRC->ch3);
		ch_offsetx  =  (pRC->ch4);
		ch_offsetw  =  (pRC->ch1);
		
	}
	else if(pChassis->mode == MODE_UPC){
		ch_offsety	= 100*car.pChassis_info->data.speedY.data;
		ch_offsetx	= 100*car.pChassis_info->data.speedX.data;//Y
		ch_offsetw	= 20*car.pChassis_info->data.speedZ.data;
	}
	
	normalized_v = ((sqrt(ch_offsetx*ch_offsetx + ch_offsety*ch_offsety) / CHANNEL_RANGE) > 1) ?
					1 : (sqrt(ch_offsetx*ch_offsetx + ch_offsety*ch_offsety) / CHANNEL_RANGE);
	
	//0是比例向量长度，1是与y轴夹角，这里前方是y通道3 右方是x通道2
	pChassis->given_chassis_v_vector[0] = CHASSIS_MAX_V * normalized_v;
	pChassis->given_chassis_v_vector[1] = atan2(ch_offsety, ch_offsetx);//角度
	pChassis->given_chassis_w			= CHASSIS_MAX_W * ch_offsetw;
}

void rc_to_main_gimbal(gimbal_t* pGimbal, rc_info_t* pRC)
{
	pGimbal->mode = pRC->sw1;
	fp32 ch_offset; 
	ch_offset  =  -(pRC->ch5)/50;
	pGimbal->given_increment_angle = ch_offset;
	if(pGimbal->mode == MODE_RC){
		pGimbal->given_state = CMD_9025_START;	
	}
	else{
		pGimbal->given_state = CMD_9025_STOP;
	}
}
