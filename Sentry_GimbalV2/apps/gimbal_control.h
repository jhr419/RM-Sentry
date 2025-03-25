#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "typedef.h"
#include "motors.h"
#include "pid.h"
#include "bsp_rc.h"
#include "auto_aim.h"

#define ECD_RANGE 8192

//TODO：
#define GIMBAL_MOTOR_MAX_RPM 1200

#define ONE_FREEDOM   1
#define TWO_FREEDOM   2
#define THREE_FREEDOM 3

#define ZERO_SPEED 1
#define FOLLOW_RC  2
#define ANGLE_CMD  3
#define GUARD	   	 4
#define TRACKING   5

#define RC_DEADZONE 20

#define ANGLE_TO_ECD(anlge) (uint16_t)(angle*8192)/360
#define VICE_GIMBAL_MAX_RPM 40
#define MAX_MOTOR_ECD 8192
#define MAX_ANGLE 360

#define LIMIT(value, max, min) ((value) > (max) ? (max) : ((value) < (min) ? (min) : (value)))
#define ECD_LIMIT(ref, set) \
    ((set) > (ref) ? \
        (((set) - (ref)) > ((ref) - (set) + MAX_MOTOR_ECD) ? \
            ((set) - MAX_MOTOR_ECD) : (set)) : \
        ((ref) > (set) ? \
            (((ref) - (set)) > ((set) - (ref) + MAX_MOTOR_ECD) ? \
                ((set) + MAX_MOTOR_ECD) : (set)) : \
            (set)))

#define ANGLE_LIMIT(ref, set) \
    ((set) > (ref) ? \
        (((set) - (ref)) > ((ref) - (set) + MAX_ANGLE) ? \
            ((set) - MAX_ANGLE) : (set)) : \
        ((ref) > (set) ? \
            (((ref) - (set)) > ((set) - (ref) + MAX_ANGLE) ? \
                ((set) + MAX_ANGLE) : (set)) : \
            (set)))

typedef struct{
	motor_6020_t* yaw;
	motor_6020_t* pitch;
	motor_6020_t* roll;

	uint8_t type;//针对不同构型的云台，主要是自由度不同
	uint8_t mode;//警戒，敌跟踪，角度控制，遥控，零速自由

}gimbal_t;

void gimbal_init(	uint8_t mode, 
									fp32 YawGyroPID[3], fp32 PitchPID[3], 
									fp32 YawEcdPID[3], fp32 PitchEcdPID[3], 
									fp32 YawRpmPID[3], fp32 PitchRpmPID[3], 

									fp32 Yaw_gyro_max_out, fp32 Yaw_gyro_max_iout,
									fp32 Pitch_gyro_max_out, fp32 Pitch_gyro_max_iout,

									fp32 Yaw_ecd_max_out, fp32 Yaw_ecd_max_iout,
									fp32 Pitch_ecd_max_out, fp32 Pitch_ecd_max_iout,

									fp32 Yaw_Rpm_max_out, fp32 Yaw_Rpm_max_iout,
									fp32 Pitch_Rpm_max_out, fp32 Pitch_Rpm_max_iout);
void gimbal_control(gimbal_t* pGimbal, rc_info_t* pRC);

#endif
