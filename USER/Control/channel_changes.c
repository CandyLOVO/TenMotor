#include "channel_changes.h"
#include "rc_potocal.h"
#include "chassis.h"
#include "motion_overlay.h"
#include "handle_value.h"
#include "user_can.h"
#include "user_pid.h"

extern RC_ctrl_t rc_ctrl;
extern uint16_t initial_angle[4];
extern motor_info motor[8];
extern pidTypeDef PID_angle[4];
extern pidTypeDef PID_speed_3508[4];
extern pidTypeDef PID_speed_6020[4];
extern int16_t motor_angle[4]; //6020angle Calculated as a global variable in "motion_overlay.c"
extern int16_t motor_speed[4]; //3508speed
extern int16_t Max_out_a;
extern int16_t Max_iout_a;
extern int16_t Max_out_s;
extern int16_t Max_iout_s;

extern motor_info motor_2[8];

int16_t get_6020[4];
int16_t speed_6020[4];
int16_t output_6020[4];
int16_t output_3508[4];

void translational_control() //Chassis translational control
{
	translate_6020(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]);
	for(int i=0;i<4;i++){
		get_6020[i] = - motor_value(initial_angle[i],motor[i+4].angle);  //Motor angle - get;the current motor angle projected from 0 to 180/0 to -180 // Counterclockwise rotation - add a negative sign
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	
	translate_3508(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]);
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s); //3508????
	}
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
	can_cmd_send_6020_2(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
}

void compound_control()
{
	compound_movement_6020(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //Setting the angle of rotation and translation for 6020, calculated as motor_angle[4]
	for(int i=0;i<4;i++){
		get_6020[i] = - motor_value(initial_angle[i],motor[i+4].angle);
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	compound_movement_3508(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]); //Setting the speed of rotation and translation for 3508, calculated as motor_speed[4]
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s); //3508????
	}
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
	can_cmd_send_6020_2(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
}

void rotate_control()
{
	rotate_6020(); 
	for(int i=0;i<4;i++){
		get_6020[i] = - motor_value(initial_angle[i],motor[i+4].angle);
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	translate_3508(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1]);
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s); //3508????
	}
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
	can_cmd_send_6020_2(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
}
