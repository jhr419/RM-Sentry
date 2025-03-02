#include "motors.h"
#include "pid.h"

motor_3508_measure_t	motors_3508_measure[4];
motor_6020_measure_t 	motors_6020_measure[3];



motor_9025_measure_t 	motor_9025;
motor_9025_pid_t 		motor_9025_pid;
motor_9025_ecd_data_t 	motor_9025_ecd_data;

pid_t motor_3508_pid[4];
pid_t motor_6020_ecdPid[3];
pid_t motor_6020_rpmPid[3];


motor_3508_t motors_3508[4];
motor_6020_t motors_6020[3];

//发射机构
motor_3508_measure_t 	motors_ammo_measure[2];
motor_2006_measure_t	motors_2006_measure;

pid_t motor_ammo_pid[2];
pid_t motor_2006_pid;

motor_3508_t motors_ammo[2];
motor_2006_t motors_2006;

motor_3508_t* motors_ammo_init(uint8_t mode, fp32 PID[3], fp32 max_out, fp32 max_iout)
{
	for(int i=0;i<__COUNT(motors_ammo);i++)
	{
		motors_ammo[i].hmotor_3508_measure = motors_ammo_measure + i;
		motors_ammo[i].pid				   = motor_ammo_pid	+ i;
		motors_ammo[i].pid_coefficient[0]  = PID[0];
		motors_ammo[i].pid_coefficient[1]  = PID[1];
		motors_ammo[i].pid_coefficient[2]  = PID[2];
		
		PID_init(motors_ammo[i].pid,mode, motors_ammo[i].pid_coefficient, max_out, max_iout);
		//6020 2006
	}
	
	return motors_ammo;
}

motor_2006_t* motors_2006_init(uint8_t mode, fp32 PID[3], fp32 max_out, fp32 max_iout)
{
	motors_2006.hmotor_2006_measure = &motors_2006_measure;
	motors_2006.pid = &motor_2006_pid;
	motors_2006.pid_coefficient[0] = PID[0];
	motors_2006.pid_coefficient[1] = PID[1];
	motors_2006.pid_coefficient[2] = PID[2];
	
	PID_init(motors_2006.pid, mode, motors_2006.pid_coefficient, max_out, max_iout);
	
	return &motors_2006;
}

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

motor_6020_t* motors_6020_init(uint8_t mode, fp32 YawEcdPID[3], fp32 PitchEcdPID[3], fp32 rpmPID[3] , fp32 ecd_max_out, fp32 ecd_max_iout, fp32 rpm_max_out, fp32 rpm_max_iout)
{
	motors_6020[0].ecdPid_coefficient[0]  = YawEcdPID[0];
	motors_6020[0].ecdPid_coefficient[1]  = YawEcdPID[1];
	motors_6020[0].ecdPid_coefficient[2]  = YawEcdPID[2];
	
	motors_6020[1].ecdPid_coefficient[0]  = PitchEcdPID[0];
	motors_6020[1].ecdPid_coefficient[1]  = PitchEcdPID[1];
	motors_6020[1].ecdPid_coefficient[2]  = PitchEcdPID[2];
	
	
	for(int i=0;i<__COUNT(motors_6020);i++)
	{
		motors_6020[i].hmotor_6020_measure = motors_6020_measure + i;
		
		motors_6020[i].ecdPid			   = motor_6020_ecdPid	+ i;
		PID_init(motors_6020[i].ecdPid, mode, motors_6020[i].ecdPid_coefficient, ecd_max_out, ecd_max_iout);
		
		motors_6020[i].rpmPid			   = motor_6020_rpmPid	+ i;
		motors_6020[i].rpmPid_coefficient[0]  = rpmPID[0];
		motors_6020[i].rpmPid_coefficient[1]  = rpmPID[1];
		motors_6020[i].rpmPid_coefficient[2]  = rpmPID[2];
		
		PID_init(motors_6020[i].rpmPid, mode, motors_6020[i].rpmPid_coefficient, rpm_max_out, rpm_max_iout);
	}
	
	return motors_6020;
}


