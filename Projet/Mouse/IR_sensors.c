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
#include "stdbool.h"
#include "leds.h"

#define WALL_THRESHOLD 	100
#define SPEED_0			0
#define TRUE			1
#define FALSE			0

int speed=MOTOR_SPEED_LIMIT-200;

static THD_WORKING_AREA(proximity_scan_wa, 2048);
static THD_FUNCTION(proximity_scan, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	systime_t time;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	int16_t leftSpeed = 0, rightSpeed = 0;

	while(1) {
		time = chVTGetSystemTime();

		if(pause_state()==FALSE){
			messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

			//autopilote seulement si dans un couloir
			if(tof_wall_too_close()==FALSE){
				chprintf((BaseSequentialStream *)&SD3, "autopilote\r\n");
				leftSpeed = speed - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = speed - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);
			}
		}
		else if(pause_state()==TRUE){
			right_motor_set_speed(SPEED_0);
			left_motor_set_speed(SPEED_0);
		}

		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}
}

void init_IR_thread(void){
	chThdCreateStatic(proximity_scan_wa, sizeof(proximity_scan_wa), NORMALPRIO, proximity_scan, NULL);
}

void maze_end_check(void){
	unsigned int front_scan_var=0;
	unsigned int lateral_scan_var=0;

	front_scan_var=abs(get_calibrated_prox(0)-get_calibrated_prox(7));
	lateral_scan_var=abs(get_calibrated_prox(1)-get_calibrated_prox(6));

	if(abs(front_scan_var-lateral_scan_var)<30){
		set_rgb_led(0, 10, 0, 0);
	}
}

void junction_scan(void){
	unsigned int left=0;
	unsigned int right=0;

	left=get_calibrated_prox(5);
	right=get_calibrated_prox(2);

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
		maze_end_check();
		dead_end();
	}
}

void change_speed(void){//TODO completer
	static int speed_modifier=0;
	static int counter=0;

	counter++;
	speed_modifier=200*counter;
	speed=MOTOR_SPEED_LIMIT-speed_modifier;

	if(counter==4){
		counter=0;
	}
}
