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
	#define YAW_LOW_LIMIT	 	 340
	#define YAW_HIGH_LIMIT	 5100
	
	#define PITCH_LOW_LIMIT  3600
	#define PITCH_HIGH_LIMIT 4300
#endif
#ifdef VICE_GIMBAL_RIGHT
	#define YAW_LOW_LIMIT	 	 142
	#define YAW_HIGH_LIMIT	 5350
	
	#define PITCH_LOW_LIMIT	 4860
	#define PITCH_HIGH_LIMIT 5700
#endif

MotorVoltageFilter ecd_filter[2];
MotorVoltageFilter angle_filter[2];

extern motor_6020_measure_t 	motors_6020_measure[3];
extern car_t car;
gimbal_t gimbal;

void gimbal_init(	uint8_t mode, 
									fp32 YawGyroPID[3], fp32 PitchGyroPID[3],
									fp32 YawEcdPID[3], fp32 PitchEcdPID[3], 
									fp32 YawRpmPID[3], fp32 PitchRpmPID[3], 
									
									fp32 Yaw_gyro_max_out, fp32 Yaw_gyro_max_iout,
									fp32 Pitch_gyro_max_out, fp32 Pitch_gyro_max_iout,

									fp32 Yaw_ecd_max_out, fp32 Yaw_ecd_max_iout,
									fp32 Pitch_ecd_max_out, fp32 Pitch_ecd_max_iout,
									
									fp32 Yaw_Rpm_max_out, fp32 Yaw_Rpm_max_iout,
									fp32 Pitch_Rpm_max_out, fp32 Pitch_Rpm_max_iout
){
	//TODO: type mode
	motor_6020_t* motor_6020_arr 
	= motors_6020_init(mode, 	
										 YawGyroPID, PitchGyroPID,
										 YawEcdPID,  PitchEcdPID,
										 YawRpmPID,  PitchRpmPID,
	
										 Yaw_gyro_max_out, Yaw_gyro_max_iout,
										 Pitch_gyro_max_out, Pitch_gyro_max_iout,
	
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

	Filter_Init(&ecd_filter[0], 0);
	Filter_Init(&ecd_filter[1], 0);
}

void gimbal_ecd_limit(gimbal_t* pGimbal){
	if(pGimbal->yaw->given_ecd <= YAW_LOW_LIMIT) 
		pGimbal->yaw->given_ecd = YAW_LOW_LIMIT;
	else if(pGimbal->yaw->given_ecd >= YAW_HIGH_LIMIT) 
		pGimbal->yaw->given_ecd = YAW_HIGH_LIMIT;
		
	if(pGimbal->pitch->given_ecd <= PITCH_LOW_LIMIT)
		pGimbal->pitch->given_ecd = PITCH_LOW_LIMIT;
	else if(pGimbal->pitch->given_ecd >= PITCH_HIGH_LIMIT)
		pGimbal->pitch->given_ecd = PITCH_HIGH_LIMIT;
}

void gimbal_angle_to_ecd(gimbal_t* pGimbal)
{
//	uint8_t type = pGimbal->type;
//	uint8_t mode = pGimbal->mode;
	
//	int16_t yaw_angle;
//	int16_t pitch_angle;
	
//	yaw_angle 	= pGimbal->yaw->given_angle;
//	pitch_angle = pGimbal->pitch->given_angle;
	
//	pGimbal->yaw->given_ecd = ANGLE_TO_ECD(yaw_angle);
//	pGimbal->pitch->given_ecd = ANGLE_TO_ECD(pitch_angle);
}

void gimbal_pid_calc(gimbal_t* pGimbal, rc_info_t* pRC)
{
	uint8_t mode = pGimbal->mode;
		
	int16_t yaw_ref	  = 0;
	int16_t yaw_set 	= 0;
	int16_t pitch_ref = 0;
	int16_t pitch_set = 0;
	
	if(mode == GIMBAL_MODE_AUTO_AIM){
		yaw_ref 	= car.pImu_info->yaw.data;
		yaw_set 	= pGimbal->yaw->given_angle;
		pitch_ref = car.pImu_info->roll.data;
		pitch_set = pGimbal->pitch->given_angle;
		
		PID_calc(pGimbal->yaw->	 gyroPid, 	yaw_ref,		ANGLE_LIMIT(yaw_ref,yaw_set 	 ));
		PID_calc(pGimbal->pitch->gyroPid, 	pitch_ref,	ANGLE_LIMIT(pitch_ref,pitch_set));
		
		pGimbal->yaw->given_speed_rpm		= pGimbal->yaw->gyroPid->out;
		pGimbal->pitch->given_speed_rpm	= -pGimbal->pitch->gyroPid->out;
			
		Filter_Process(&angle_filter[0], pGimbal->yaw->given_speed_rpm);
		Filter_Process(&angle_filter[1], pGimbal->pitch->given_speed_rpm);
		
		PID_calc(pGimbal->yaw->rpmPid, 		pGimbal->yaw->hmotor_6020_measure->speed_rpm,		angle_filter[0].filtered_value);
		PID_calc(pGimbal->pitch->rpmPid, 	pGimbal->pitch->hmotor_6020_measure->speed_rpm,	angle_filter[1].filtered_value);
	}
	else{
		if(mode == GIMBAL_MODE_RC){
			yaw_ref		= pGimbal->yaw->	hmotor_6020_measure->ecd;
			yaw_set		= pGimbal->yaw->	given_ecd;
			pitch_ref = pGimbal->pitch->hmotor_6020_measure->ecd;
			pitch_set = pGimbal->pitch->given_ecd;
		}
		else if(mode == GIMBAL_MODE_HOLD){
			yaw_ref		= pGimbal->yaw->hmotor_6020_measure->ecd;
			yaw_set		= pGimbal->yaw->hold_ecd;
			pitch_ref = pGimbal->pitch->hmotor_6020_measure->ecd;
			pitch_set = pGimbal->pitch->hold_ecd;
		}
		
		PID_calc(pGimbal->yaw->	 ecdPid, 	yaw_ref,		ECD_LIMIT(yaw_ref,yaw_set));
		PID_calc(pGimbal->pitch->ecdPid, 	pitch_ref,	ECD_LIMIT(pitch_ref,pitch_set));
		
		pGimbal->yaw->given_speed_rpm		= pGimbal->yaw->ecdPid->out;
		pGimbal->pitch->given_speed_rpm	= -pGimbal->pitch->ecdPid->out;
			
		Filter_Process(&ecd_filter[0], pGimbal->yaw->given_speed_rpm);
		Filter_Process(&ecd_filter[1], pGimbal->pitch->given_speed_rpm);
		
		PID_calc(pGimbal->yaw->rpmPid, 		pGimbal->yaw->hmotor_6020_measure->speed_rpm,		ecd_filter[0].filtered_value);
		PID_calc(pGimbal->pitch->rpmPid, 	pGimbal->pitch->hmotor_6020_measure->speed_rpm,	ecd_filter[1].filtered_value);
	}
}	

void gimbal_control(gimbal_t* pGimbal, rc_info_t* pRC)
{
	//解析遥控器指令，将目标角度给到结构体变量
	rc_to_angle(pGimbal, pRC);

//	gimbal_angle_to_ecd(pGimbal);
	
	//限制输出ecd
	gimbal_ecd_limit(pGimbal);
	
	gimbal_pid_calc(pGimbal, pRC);

	if( pGimbal->mode == GIMBAL_MODE_AUTO_AIM || pGimbal->mode == 0){
		//暂时将自瞄档用于断控模式；为0时说明遥控器没有连接
		CAN_Control6020Voltage(0,0,0,0);
	}else{
//		CAN_Control6020Voltage(pGimbal->yaw->rpmPid->out, pGimbal->pitch->rpmPid->out, car.pAmmoBooster->booster->pid->out,0);
		CAN_Control6020Voltage(pGimbal->yaw->rpmPid->out, 0, car.pAmmoBooster->booster->pid->out,0);
	}
	uart1_printf("%d,%d,%d,%.2f\n", pGimbal->yaw->hmotor_6020_measure->ecd,
															pGimbal->yaw->given_ecd,
															pGimbal->yaw->hold_ecd,
															pGimbal->yaw->rpmPid->out);
}
