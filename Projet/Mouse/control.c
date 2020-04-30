#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "sensors/proximity.h"
#include "motors.h"

#include <main.h>
#include "tof.h"
#include "IR_sensors.h"
#include "audio_processing.h"

#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define TURN_90DEG			PERIMETER_EPUCK/4
#define TURN_180DEG			PERIMETER_EPUCK/2
#define TURN_LEFT			-1
#define TURN_RIGHT			1
#define TURNING_SPEED		300
#define SPEED_0				0

static bool pause_flag=FALSE;

void init_threads(void){
	init_IR_thread();		//lance thread des capteurs IR
	init_tof_thread();	//lance thread du tof
    //init_sound_thread();	//lance thread d'analyse du son
}

void turn(float position, int sense){
	int32_t final_l_pos=0;
	int32_t final_r_pos=0;
	int32_t l_pos=0;
	int32_t r_pos=0;

	left_motor_set_pos(0);
	right_motor_set_pos(0);

	final_l_pos=position * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	final_r_pos=-position * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	left_motor_set_speed(sense*TURNING_SPEED);
	right_motor_set_speed(-sense*TURNING_SPEED);

	l_pos=sense*left_motor_get_pos();
	r_pos=sense*right_motor_get_pos();

	while(l_pos<final_l_pos || r_pos>final_r_pos ){
		l_pos=sense*left_motor_get_pos();
		r_pos=sense*right_motor_get_pos();
	}

	left_motor_set_speed(SPEED_0);
	right_motor_set_speed(SPEED_0);
}

void user_direction_input(void){
	set_pause();
	//TODO ask user for direction
}

void turn_left(void){
	turn(TURN_90DEG, TURN_LEFT);
}

void turn_right(void){
	turn(TURN_90DEG, TURN_RIGHT);
}

void dead_end(void){
	//TODO dead end...  then what?
	turn(TURN_180DEG, TURN_RIGHT);
}

void set_pause(void){
	pause_flag=TRUE;
}

void set_play(void){
	pause_flag=FALSE;
}

bool pause_state(void){
	return pause_flag;
}
