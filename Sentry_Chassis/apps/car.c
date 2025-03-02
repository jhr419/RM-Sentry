#include "car.h"
//TODO: 确定pid参数

#define MAX_MOTOR_CAN_CURRENT 16000.0f
#define M3505_MOTOR_SPEED_PID_KP 10.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 0.0f

fp32 pid_k[3]={M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

extern chassis_info_t chassis_info;
extern rc_info_t rc;
extern gimbal_t gimbal;
extern chassis_t chassis;
extern aim_info_t aim_info;
extern referee_system_into_t referee_system_into;
extern power_heat_data_t power_heat_data;

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

void car_init(void)//TODO:如何制定car类型、部位及其参数
{
//	referee_system_info_init();
	
	car.pRC 			= &rc;
	car.pGimbal 		= &gimbal;
	car.pChassis 		= &chassis;	
	car.pAim_info		= &aim_info;
	car.pReferee_system_into = &referee_system_into;
	car.pReferee_system_into->power_heat_data = &power_heat_data;
	car.pChassis_info = &chassis_info;
	
	car.pChassis->max_v = 1000.0f;//TODO:如何确定最大值
	
	
	
	chassis_init(PID_POSITION,
				 pid_k,
				 M3505_MOTOR_SPEED_PID_MAX_OUT,
				 M3505_MOTOR_SPEED_PID_MAX_IOUT);
	
	
	
	HAL_GPIO_WritePin(GPIOG, LED_8_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOG, LED_8_Pin, GPIO_PIN_SET);
}
