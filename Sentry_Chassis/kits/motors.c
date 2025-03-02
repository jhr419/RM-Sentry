#include "motors.h"
#include "pid.h"

motor_3508_measure_t	motors_3508_measure[4];
motor_6020_measure_t 	motors_6020_measure[2];
motor_2006_measure_t	motors_2006_measure[2];

motor_9025_measure_t 	motor_9025;
motor_9025_pid_t 		motor_9025_pid;
motor_9025_ecd_data_t 	motor_9025_ecd_data;

pid_t motor_3508_pid[4];
pid_t motor_6020_pid[4];
pid_t motor_2006_pid[4];

motor_3508_t motors_3508[4];
motor_6020_t motors_6020[4];
motor_2006_t motors_2006[4];
motor_9025_t motors_9025;
//初始化所有电机成员变量
motor_3508_t* motors_3508_init(uint8_t mode, fp32 PID3508[3], fp32 max_out, fp32 max_iout)
{
	for(int i=0;i<__COUNT(motors_3508);i++)
	{
		motors_3508[i].hmotor_3508_measure = motors_3508_measure + i;
		motors_3508[i].pid				   = motor_3508_pid	+ i;
		motors_3508[i].pid_coefficient[0]  = PID3508[0];
		motors_3508[i].pid_coefficient[1]  = PID3508[1];
		motors_3508[i].pid_coefficient[2]  = PID3508[2];
		
		PID_init(motors_3508[i].pid,mode, motors_3508[i].pid_coefficient, max_out, max_iout);
		//6020 2006
	}
	
	return motors_3508;
}

motor_9025_t* motors_9025_init(){
	return &motors_9025;
}
