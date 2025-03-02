#include "gimbal_control.h"
#include "motors.h"
#include "remote_control.h"
#include "CAN_in_motors.h"
#include "math.h"
#include "cmsis_os.h"
#include "car.h"
extern motor_6020_measure_t 	motors_6020_measure[3];
extern car_t car;
gimbal_t gimbal;

void gimbal_init(uint8_t mode, fp32 YawEcdPID[3], fp32 PitchEcdPID[3], fp32 rpmPID[3] , fp32 ecd_max_out, fp32 ecd_max_iout, fp32 rpm_max_out, fp32 rpm_max_iout)
{
	//TODO: type mode
	motor_6020_t* motor_6020_arr = motors_6020_init(mode, YawEcdPID, PitchEcdPID, rpmPID, ecd_max_out, ecd_max_iout, rpm_max_out, rpm_max_iout);
	
	gimbal.yaw 	 = &motor_6020_arr[0];
	gimbal.pitch = &motor_6020_arr[1];
	gimbal.roll  = &motor_6020_arr[2];

	gimbal.yaw->hmotor_6020_measure  = &motors_6020_measure[0];
	gimbal.pitch->hmotor_6020_measure= &motors_6020_measure[1];
	gimbal.roll->hmotor_6020_measure = &motors_6020_measure[2];
}

void gimbal_angle_to_ecd(gimbal_t* pGimbal)
{
	uint8_t type = pGimbal->type;
	uint8_t mode = pGimbal->mode;
	
	int16_t yaw_angle;
	int16_t pitch_angle;
	int16_t roll_angle;
	
	yaw_angle 	= pGimbal->yaw->given_angle;
	pitch_angle = pGimbal->pitch->given_angle;
	roll_angle 	= pGimbal->roll->given_angle;
	
	pGimbal->yaw->given_ecd = ANGLE_TO_ECD(yaw_angle);
	pGimbal->pitch->given_ecd = ANGLE_TO_ECD(pitch_angle);
	pGimbal->roll->given_ecd = ANGLE_TO_ECD(roll_angle);
}

void gimbal_pid_calc(gimbal_t* pGimbal, rc_info_t* pRC)
{
	uint8_t mode = pGimbal->mode;
		
	//todo pitch roll
	int16_t yaw_ref_angle = car.pImu_info->yaw.data;
	int16_t yaw_set_angle = pGimbal->given_pose_angle_yaw;
	int16_t pitch_ref_angle = car.pImu_info->roll.data;
	int16_t pitch_set_angle = pGimbal->given_pose_angle_pitch;
	
	PID_calc(pGimbal->yaw->ecdPid, 		yaw_ref_angle,		ANGLE_LIMIT(yaw_ref_angle,yaw_set_angle ));
	PID_calc(pGimbal->pitch->ecdPid, 	pitch_ref_angle,	ANGLE_LIMIT(pitch_ref_angle,pitch_set_angle));
	
	pGimbal->yaw->given_speed_rpm	= pGimbal->yaw->ecdPid->out;
	pGimbal->pitch->given_speed_rpm	= pGimbal->pitch->ecdPid->out;
	
	//速度环
	PID_calc(pGimbal->yaw->rpmPid, 		pGimbal->yaw->hmotor_6020_measure->speed_rpm,	pGimbal->yaw->given_speed_rpm	);
	PID_calc(pGimbal->pitch->rpmPid, 	pGimbal->pitch->hmotor_6020_measure->speed_rpm,	pGimbal->pitch->given_speed_rpm	);
}	

void gimbal_control(gimbal_t* pGimbal, rc_info_t* pRC)
{
	rc_to_angle(pGimbal, pRC);
	
	gimbal_angle_to_ecd(pGimbal);
	
	gimbal_pid_calc(pGimbal, pRC);
	
	CAN_Control6020Voltage(	pGimbal->yaw->rpmPid->out,
							pGimbal->pitch->rpmPid->out,
							car.pAmmoBooster->booster->pid->out,
							0);
//	CAN_Control6020Voltage(	pGimbal->yaw->rpmPid->out,
//							0,
//							0,
//							0);
	if(pGimbal->mode == GIMBAL_MODE_RC){
		pGimbal->last_pose_angle_yaw = (int16_t)(car.pImu_info->yaw.data);
		pGimbal->last_pose_angle_pitch = (int16_t)(car.pImu_info->roll.data);
	}
}
