#include "gimbal_control.h"
#include "motors.h"
#include "remote_control.h"
#include "CAN_in_motors.h"
#include "math.h"
#include "cmsis_os.h"
#include "car.h"
#include "communication.h"
#include "stdlib.h"
#include "filter.h"

#ifdef VICE_GIMBAL_LEFT
	#define YAW_LOW_LIMIT1	 10
	#define YAW_LOW_LIMIT2	 25
	#define YAW_HIGH_LIMIT1	 220
	#define YAW_HIGH_LIMIT2	 240
	
	#define PITCH_LOW_LIMIT  250
	#define PITCH_HIGH_LIMIT 280
#endif
#ifdef VICE_GIMBAL_RIGHT
	#define YAW_LOW_LIMIT1	 100
	#define YAW_LOW_LIMIT2	 120
	#define YAW_HIGH_LIMIT1	 240
	#define YAW_HIGH_LIMIT2	 250
	
	#define PITCH_LOW_LIMIT		153
	#define PITCH_HIGH_LIMIT 	200
#endif
//todo

MotorVoltageFilter angle_filter[2];
MotorVoltageFilter rpm_filter[2];

extern motor_6020_measure_t 	motors_6020_measure[3];
extern car_t car;
gimbal_t gimbal;

void gimbal_init(	uint8_t mode, 
									fp32 YawEcdPID[3], fp32 PitchEcdPID[3], 
									fp32 YawRpmPID[3], fp32 PitchRpmPID[3], 

									fp32 Yaw_ecd_max_out, fp32 Yaw_ecd_max_iout,
									fp32 Pitch_ecd_max_out, fp32 Pitch_ecd_max_iout,
									
									fp32 Yaw_Rpm_max_out, fp32 Yaw_Rpm_max_iout,
									fp32 Pitch_Rpm_max_out, fp32 Pitch_Rpm_max_iout){
	//TODO: type mode
	motor_6020_t* motor_6020_arr 
	= motors_6020_init(mode, 	YawEcdPID, PitchEcdPID, YawRpmPID, PitchRpmPID, 
										
														Yaw_ecd_max_out, Yaw_ecd_max_iout,
														Pitch_ecd_max_out, Pitch_ecd_max_iout,
										
														Yaw_Rpm_max_out, Yaw_Rpm_max_iout,
														Pitch_Rpm_max_out, Pitch_Rpm_max_iout);
	
	gimbal.yaw 	 = &motor_6020_arr[0];
	gimbal.pitch = &motor_6020_arr[1];
	gimbal.roll  = &motor_6020_arr[2];
	
	gimbal.yaw->hmotor_6020_measure  = &motors_6020_measure[0];
	gimbal.pitch->hmotor_6020_measure= &motors_6020_measure[1];
	gimbal.roll->hmotor_6020_measure = &motors_6020_measure[2];
										
	Filter_Init(&angle_filter[0], 0);
	Filter_Init(&angle_filter[1], 0);
										
	Filter_Init(&rpm_filter[0], 0);
	Filter_Init(&rpm_filter[1], 0);
										
}

void gimbal_angle_limit(gimbal_t* pGimbal){
	#ifdef VICE_GIMBAL_LEFT
		if(pGimbal->yaw->given_angle >= YAW_LOW_LIMIT1 && pGimbal->yaw->given_angle <= YAW_LOW_LIMIT2) 
			pGimbal->yaw->given_angle = YAW_LOW_LIMIT2;
		else if(pGimbal->yaw->given_angle >= YAW_HIGH_LIMIT1 && pGimbal->yaw->given_angle <= YAW_HIGH_LIMIT2) 
			pGimbal->yaw->given_angle = YAW_HIGH_LIMIT1;
	#endif
		
	#ifdef VICE_GIMBAL_RIGHT
		if(pGimbal->yaw->given_angle >= YAW_LOW_LIMIT1 && pGimbal->yaw->given_angle <= YAW_LOW_LIMIT2) 
			pGimbal->yaw->given_angle = YAW_LOW_LIMIT1;
		else if(pGimbal->yaw->given_angle >= YAW_HIGH_LIMIT1 && pGimbal->yaw->given_angle <= YAW_HIGH_LIMIT2) 
			pGimbal->yaw->given_angle = YAW_HIGH_LIMIT2;
	#endif
	if(pGimbal->pitch->given_angle <= PITCH_LOW_LIMIT)
		pGimbal->pitch->given_angle = PITCH_LOW_LIMIT;
	else if(pGimbal->pitch->given_angle >= PITCH_HIGH_LIMIT)
		pGimbal->pitch->given_angle = PITCH_HIGH_LIMIT;
}

void gimbal_angle_to_ecd(gimbal_t* pGimbal)
{
	uint8_t type = pGimbal->type;
	uint8_t mode = pGimbal->mode;
	
	int16_t yaw_angle;
	int16_t pitch_angle;
	
	yaw_angle 	= pGimbal->yaw->given_angle;
	pitch_angle = pGimbal->pitch->given_angle;
	
	pGimbal->yaw->given_ecd = ANGLE_TO_ECD(yaw_angle);
	pGimbal->pitch->given_ecd = ANGLE_TO_ECD(pitch_angle);
}

void gimbal_pid_calc(gimbal_t* pGimbal, rc_info_t* pRC)
{
	uint8_t mode = pGimbal->mode;
		
	//todo pitch roll
	int16_t yaw_ref_angle = car.pImu_info->yaw.data;
	int16_t yaw_set_angle = pGimbal->yaw->given_angle;
	int16_t pitch_ref_angle = car.pImu_info->roll.data;
	int16_t pitch_set_angle = pGimbal->pitch->given_angle;
	
	PID_calc(pGimbal->yaw->	 ecdPid, 	yaw_ref_angle,		ANGLE_LIMIT(yaw_ref_angle,yaw_set_angle 	 ));
	PID_calc(pGimbal->pitch->ecdPid, 	pitch_ref_angle,	ANGLE_LIMIT(pitch_ref_angle,pitch_set_angle));
	
	pGimbal->yaw->given_speed_rpm		= pGimbal->yaw->ecdPid->out;
	pGimbal->pitch->given_speed_rpm	= -pGimbal->pitch->ecdPid->out;
		
	Filter_Process(&angle_filter[0], pGimbal->yaw->given_speed_rpm);
	Filter_Process(&angle_filter[1], pGimbal->pitch->given_speed_rpm);
	
	//速度环
//	PID_calc(pGimbal->yaw->rpmPid, 		pGimbal->yaw->hmotor_6020_measure->speed_rpm,		pGimbal->yaw->given_speed_rpm	 );
//	PID_calc(pGimbal->pitch->rpmPid, 	pGimbal->pitch->hmotor_6020_measure->speed_rpm,	pGimbal->pitch->given_speed_rpm);

	PID_calc(pGimbal->yaw->rpmPid, 		pGimbal->yaw->hmotor_6020_measure->speed_rpm,		angle_filter[0].filtered_value);
	PID_calc(pGimbal->pitch->rpmPid, 	pGimbal->pitch->hmotor_6020_measure->speed_rpm,	angle_filter[1].filtered_value);
	
//	Filter_Process(&rpm_filter[0],pGimbal->yaw->rpmPid->out);
//	Filter_Process(&rpm_filter[1],pGimbal->pitch->rpmPid->out);
}	

void gimbal_control(gimbal_t* pGimbal, rc_info_t* pRC)
{
	//解析遥控器指令，将目标角度给到结构体变量
	rc_to_angle(pGimbal, pRC);

//	gimbal_angle_to_ecd(pGimbal);
	
	//限制输出角度
	gimbal_angle_limit(pGimbal);
	
	gimbal_pid_calc(pGimbal, pRC);

	if( pGimbal->mode == GIMBAL_MODE_AUTO_AIM ){
		CAN_Control6020Voltage(0,0,0,0);
	}else{
//		CAN_Control6020Voltage(rpm_filter[0].filtered_value, rpm_filter[1].filtered_value, car.pAmmoBooster->booster->pid->out,0);
		CAN_Control6020Voltage(pGimbal->yaw->rpmPid->out, pGimbal->pitch->rpmPid->out, car.pAmmoBooster->booster->pid->out,0);
	}

//	uart1_printf("%d,%d,%d,%.2f,%d\n", car.pImu_info->roll.data, pGimbal->pitch->given_angle,
//																			pGimbal->pitch->hmotor_6020_measure->speed_rpm,pGimbal->pitch->given_speed_rpm,
//																			pGimbal->pitch->hmotor_6020_measure->ecd);
		uart1_printf("%d,%d,%d,%d\n", car.pImu_info->yaw.data,pGimbal->yaw->given_angle,
																	car.pImu_info->roll.data,pGimbal->pitch->given_angle);
}
