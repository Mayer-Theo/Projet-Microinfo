#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "IR_sensors.h"
#include <main.h>
#include "sensors/proximity.h"
#include "motors.h"
#include "control.h"

#define WALL_THRESHOLD 100 //valeur expérimentale
#define CRUISE_SPEED		500

static THD_WORKING_AREA(proximity_scan_wa, 2048);
static THD_FUNCTION(proximity_scan, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	uint8_t stop_loop = 0;
	systime_t time;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	int16_t leftSpeed = 0, rightSpeed = 0;

	msg_t msg=0; //TODO check if used correctly to end thread

	while(stop_loop == 0) {
		time = chVTGetSystemTime();

		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

		leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
		rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
		right_motor_set_speed(rightSpeed);
		left_motor_set_speed(leftSpeed);
		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.

		if(prox_values.delta[1]<WALL_THRESHOLD || prox_values.delta[6]< WALL_THRESHOLD){
			right_motor_set_speed(CRUISE_SPEED);
			left_motor_set_speed(CRUISE_SPEED);
			chThdExit(msg);		//TODO mise en pause de la thread plutôt? Ou variable globale de contrôle? différence avec chThdExits? Que doit contenir msg ?=====================================================================================================
		}
	}
}

void proximity_check(void){
	chThdCreateStatic(proximity_scan_wa, sizeof(proximity_scan_wa), NORMALPRIO, proximity_scan, NULL);
}

void junction_scan(void){
	proximity_msg_t prox_values;
	unsigned int left=0;
	unsigned int right=0;

	left=prox_values.delta[5];
	right=prox_values.delta[2];

	if(left<WALL_THRESHOLD && right<WALL_THRESHOLD){
		user_direction_input();
	}
	else if(left>WALL_THRESHOLD && right<WALL_THRESHOLD){
		turn_right();
	}
	else if(left<WALL_THRESHOLD && right>WALL_THRESHOLD){
		turn_left();
	}
	else if(left>WALL_THRESHOLD && right>WALL_THRESHOLD){
		dead_end();
	}
}
