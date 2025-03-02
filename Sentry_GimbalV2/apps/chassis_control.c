#include "chassis_control.h"
#include "math.h"
#include "remote_control.h"
#include "CAN_in_motors.h"
extern motor_3508_measure_t		motors_3508_measure[4];

chassis_t chassis;

void chassis_init(uint8_t mode,fp32 PID[3], fp32 max_out, fp32 max_iout)
{
	chassis.type = OMNI;//TODO:如何底盘类型
	motor_3508_t* motor_3508_arr = motors_3508_init(mode, PID, max_out, max_iout);
		
	chassis.LF = &motor_3508_arr[0];
	chassis.RF = &motor_3508_arr[1];
	chassis.RB = &motor_3508_arr[2];
	chassis.LB = &motor_3508_arr[3];
		
	chassis.LF->hmotor_3508_measure = &motors_3508_measure[0];
	chassis.RF->hmotor_3508_measure = &motors_3508_measure[1];
	chassis.RB->hmotor_3508_measure = &motors_3508_measure[2];
	chassis.LB->hmotor_3508_measure = &motors_3508_measure[3];
}

void chassis_v_to_wheels_rpm(chassis_t* pChassis)
{
	uint8_t type 		= pChassis->type;
	uint8_t mode 		= pChassis->mode;
	fp32 	chassis_v 	= pChassis->given_chassis_v_vector[0];
	fp32 	theta   	= pChassis->given_chassis_v_vector[1];
	fp32 	chassis_w 	= pChassis->given_chassis_w;
	
	switch(type){
	/*********************************************************************/
		case MCNAMM:
			if(mode == FOLLOW_CHASSIS){
				
			}
			else if(mode == FOLLOW_GIMBAL){
					
			}
			break;
			/*********************************************************************/
		case OMNI:
			if(mode == FOLLOW_CHASSIS){
				chassis.LF->given_speed_rpm =   chassis_v * cos(PI/4+theta) + chassis_w;
				chassis.RF->given_speed_rpm = - chassis_v * cos(PI/4-theta) + chassis_w;
				chassis.RB->given_speed_rpm = - chassis_v * cos(PI/4+theta) + chassis_w;
				chassis.LB->given_speed_rpm =   chassis_v * cos(PI/4-theta) + chassis_w;
			}
			else if(mode != FOLLOW_CHASSIS){
				chassis.LF->given_speed_rpm = 0;
				chassis.RF->given_speed_rpm = 0;
				chassis.RB->given_speed_rpm = 0;
				chassis.LB->given_speed_rpm = 0;
			}
			break;
		/*********************************************************************/
		case STEERING:
			if(mode == FOLLOW_CHASSIS){
					
			}
			else if(mode == FOLLOW_GIMBAL){
					
			}
			break;
		 /*********************************************************************/
		default:
			break;
	}
}

void chassis_pid_calc(chassis_t* pChassis)
{
	PID_calc(pChassis->LF->pid, pChassis->LF->hmotor_3508_measure->speed_rpm, pChassis->LF->given_speed_rpm);
	PID_calc(pChassis->RF->pid, pChassis->RF->hmotor_3508_measure->speed_rpm, pChassis->RF->given_speed_rpm);
	PID_calc(pChassis->RB->pid, pChassis->RB->hmotor_3508_measure->speed_rpm, pChassis->RB->given_speed_rpm);
	PID_calc(pChassis->LB->pid, pChassis->LB->hmotor_3508_measure->speed_rpm, pChassis->LB->given_speed_rpm);	
}

void chassis_control(chassis_t* pChassis, rc_info_t* pRC)//TODO: 如何确定控制方式，遥控器or上位机，接口如何定义
{
	rc_to_vector(pChassis, pRC);
	
	chassis_v_to_wheels_rpm(pChassis);
	
	chassis_pid_calc(pChassis);
	
	CAN_Control3508Current(	pChassis->LF->pid->out,
							pChassis->RF->pid->out,
	                        pChassis->RB->pid->out,
	                        pChassis->LB->pid->out);
}
