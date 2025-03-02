#include "ammo_booster_control.h"
#include "motors.h"
#include "remote_control.h"
#include "CAN_in_motors.h"
#include "math.h"

extern motor_3508_measure_t 	motors_3508_measure[4];
extern motor_2006_measure_t		motors_2006_measure;

ammo_booster_t ammo_booster;

void ammo_booster_init(uint8_t mode, fp32 TriggerPID[3], fp32 BoosterPID[3], 
						fp32 trigger_max_out, fp32 trigger_max_iout, 
						fp32 booster_max_out, fp32 booster_max_iout)
{
	motor_3508_t* motor_trigger_arr = motors_ammo_init(mode,TriggerPID, trigger_max_out, trigger_max_iout);
	
	ammo_booster.trigger_l = motor_trigger_arr;
	ammo_booster.trigger_r = motor_trigger_arr + 1;
	
	//TODO ammo motor id 
//	ammo_booster.trigger_l->hmotor_3508_measure = &motors_ammo_measure[0];
//	ammo_booster.trigger_r->hmotor_3508_measure = &motors_ammo_measure[1];
	ammo_booster.trigger_l->hmotor_3508_measure = &motors_3508_measure[0];
	ammo_booster.trigger_r->hmotor_3508_measure = &motors_3508_measure[1];

	/**************/
	motor_2006_t* motor_booster = motors_2006_init(mode, BoosterPID, booster_max_out, booster_max_iout);
	ammo_booster.booster = motor_booster;
	
	ammo_booster.booster->hmotor_2006_measure = &motors_2006_measure;
}

void ammo_booster_pid_calc(ammo_booster_t* pAmmo_booster, rc_info_t* pRC)
{
	uint8_t mode = pAmmo_booster->mode;
	
	switch(mode)
	{
		case CEASE_FIRE:
			pAmmo_booster->booster->given_speed_rpm = 0;
			pAmmo_booster->trigger_l->given_speed_rpm = 0;
			pAmmo_booster->trigger_r->given_speed_rpm = 0;
			break;
		case READY_FIRE:
			pAmmo_booster->booster->given_speed_rpm = 0;
			pAmmo_booster->trigger_l->given_speed_rpm = TRIGGER_RPM;
			pAmmo_booster->trigger_r->given_speed_rpm = TRIGGER_RPM;
			break;
		case OPEN_FIRE:
			pAmmo_booster->booster->given_speed_rpm = BOOSTER_RPM;
			pAmmo_booster->trigger_l->given_speed_rpm = TRIGGER_RPM;
			pAmmo_booster->trigger_r->given_speed_rpm = TRIGGER_RPM;
			break;
		default: break;
	}
	
	PID_calc(pAmmo_booster->booster->pid, pAmmo_booster->booster->hmotor_2006_measure->speed_rpm, pAmmo_booster->booster->given_speed_rpm);
	
	PID_calc(pAmmo_booster->trigger_l->pid, pAmmo_booster->trigger_l->hmotor_3508_measure->speed_rpm, pAmmo_booster->trigger_l->given_speed_rpm);
	PID_calc(pAmmo_booster->trigger_r->pid, -pAmmo_booster->trigger_r->hmotor_3508_measure->speed_rpm, pAmmo_booster->trigger_r->given_speed_rpm);
	
}

void ammo_booster_control(ammo_booster_t* pAmmo_booster, rc_info_t* pRC)
{
	rc_to_fire_state(pAmmo_booster, pRC);
	
	ammo_booster_pid_calc(pAmmo_booster, pRC);

	CAN_Control3508Current(	pAmmo_booster->trigger_l->pid->out,
							-pAmmo_booster->trigger_r->pid->out,
							0,0);

//	CAN_Control2006Current(0,0,,0);
}