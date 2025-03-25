#include "car.h"
//TODO: 确定pid参数

//3508
#define MAX_MOTOR_CAN_CURRENT 16000.0f
#define M3508_MOTOR_SPEED_PID_KP 10.0f
#define M3508_MOTOR_SPEED_PID_KI 0.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 0.0f

//6020
//ecd
#define M6020_YAW_ECD_PID_KP 1.0f
#define M6020_YAW_ECD_PID_KI 0.0f
#define M6020_YAW_ECD_PID_KD 1.0f
#define M6020_YAW_ECD_PID_MAX_OUT 8000.0f
#define M6020_YAW_ECD_PID_MAX_IOUT 800.0f

#define M6020_PITCH_ECD_PID_KP 				0.8f		//5.0f
#define M6020_PITCH_ECD_PID_KI 				0.0f		//0.0f
#define M6020_PITCH_ECD_PID_KD 				0.4f		//0.5f
#define M6020_PITCH_ECD_PID_MAX_OUT 	8000.0f	//8000.0f
#define M6020_PITCH_ECD_PID_MAX_IOUT 	8000.0f	//800.0f

//rpm
#define M6020_MAX_VOLTAGE 20000.0f

#define M6020_YAW_RPM_PID_KP 100.0f
#define M6020_YAW_RPM_PID_KI 20.0f
#define M6020_YAW_RPM_PID_KD 0.0f
#define M6020_YAW_RPM_PID_MAX_OUT M6020_MAX_VOLTAGE
#define M6020_YAW_RPM_PID_MAX_IOUT 30000.0f

#define M6020_PITCH_RPM_PID_KP 105.0f
#define M6020_PITCH_RPM_PID_KI 22.0f
#define M6020_PITCH_RPM_PID_KD 0.0f
#define M6020_PITCH_RPM_PID_MAX_OUT 30000.0f
#define M6020_PITCH_RPM_PID_MAX_IOUT 30000.0f

//trigger done
#define MAX_TRIGGER_CAN_CURRENT 14000.0f
#define TRIGGER_SPEED_PID_KP 8.1f
#define TRIGGER_SPEED_PID_KI 0.03f
#define TRIGGER_SPEED_PID_KD 0.73f
#define TRIGGER_SPEED_PID_MAX_OUT MAX_TRIGGER_CAN_CURRENT
#define TRIGGER_SPEED_PID_MAX_IOUT 5000.0f

//booster done
#define MAX_2006_CAN_CURRENT 30000.0f
#define M2006_MOTOR_RPM_PID_KP 10.0f
#define M2006_MOTOR_RPM_PID_KI 0.0f
#define M2006_MOTOR_RPM_PID_KD 0.0f
#define M2006_MOTOR_RPM_PID_MAX_OUT MAX_2006_CAN_CURRENT
#define M2006_MOTOR_RPM_PID_MAX_IOUT 0.0f

fp32 M3508_pid_k[3]={M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};

fp32 M6020_yaw_ecd_pid_k[3] = {M6020_YAW_ECD_PID_KP, M6020_YAW_ECD_PID_KI, M6020_YAW_ECD_PID_KD};
fp32 M6020_pitch_ecd_pid_k[3] = {M6020_PITCH_ECD_PID_KP, M6020_PITCH_ECD_PID_KI, M6020_PITCH_ECD_PID_KD};

fp32 M6020_yaw_rpm_pid_k[3] = {M6020_YAW_RPM_PID_KP, M6020_YAW_RPM_PID_KI, M6020_YAW_RPM_PID_KD};
fp32 M6020_pitch_rpm_pid_k[3] = {M6020_PITCH_RPM_PID_KP, M6020_PITCH_RPM_PID_KI, M6020_PITCH_RPM_PID_KD};

fp32 M3508_trigger_pid_k[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
fp32 M2006_booster_pid_k[3] = {M2006_MOTOR_RPM_PID_KP, M2006_MOTOR_RPM_PID_KI, M2006_MOTOR_RPM_PID_KD};

extern rc_info_t rc;
extern ammo_booster_t ammo_booster;
extern aim_info_t aim_info;
extern gimbal_t gimbal;
extern chassis_t chassis;
extern imu_info_t imu_info;

extern motor_3508_t motors_3508[4];
extern motor_6020_t motors_6020[4];
extern motor_2006_t motors_2006[4];

extern pid_t motor_3508_pid[4];
extern pid_t motor_6020_pid[4];
extern pid_t motor_2006_pid[4];

extern motor_9025_measure_t 	motor_9025;
extern motor_9025_pid_t 			motor_9025_pid;
extern motor_9025_ecd_data_t motor_9025_ecd_data;

car_t car;

void car_init(void)//TODO:如何制定car类型及其参数
{
	car.pRC 		 = &rc;
	
	car.pAmmoBooster = &ammo_booster;
	car.pAim_info		= &aim_info;
	ammo_booster_init(	PID_POSITION,
						M3508_trigger_pid_k,
						M2006_booster_pid_k,
						TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT,
						M2006_MOTOR_RPM_PID_MAX_OUT,M2006_MOTOR_RPM_PID_MAX_IOUT);
	
	car.pGimbal = &gimbal;
	car.pImu_info = &imu_info;
	gimbal_init(PID_POSITION,
				M6020_yaw_ecd_pid_k, M6020_pitch_ecd_pid_k,
				M6020_yaw_rpm_pid_k, M6020_pitch_rpm_pid_k,

				M6020_YAW_ECD_PID_MAX_OUT,
				M6020_YAW_ECD_PID_MAX_IOUT,
				M6020_PITCH_ECD_PID_MAX_OUT,
				M6020_PITCH_ECD_PID_MAX_IOUT,
	
				M6020_YAW_RPM_PID_MAX_OUT, 
				M6020_YAW_RPM_PID_MAX_IOUT,
				M6020_PITCH_RPM_PID_MAX_OUT, 
				M6020_PITCH_RPM_PID_MAX_IOUT);
}
