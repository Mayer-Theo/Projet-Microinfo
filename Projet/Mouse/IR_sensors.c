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
#include "tof.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include"audio/play_melody.h"

#define WALL_THRESHOLD 	100
#define SPEED_0			0
#define TRUE			1
#define FALSE			0
static bool inside_maze = FALSE;

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

			//autopilote seulement si pas d'obstacle
			if(tof_wall_too_close()==FALSE){
				leftSpeed = speed - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = speed - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);

				if(prox_values.delta[2]<WALL_THRESHOLD && prox_values.delta[5]<WALL_THRESHOLD && inside_maze==TRUE){
					inside_maze = FALSE;
					stopCurrentMelody();
					set_pause();
					set_rgb_led(0, 10, 0, 0);
					set_rgb_led(1, 10, 0, 0);
					set_rgb_led(2, 10, 0, 0);
					set_rgb_led(3, 10, 0, 0);
					end_game_animation();
				}
				else if(prox_values.delta[2]>WALL_THRESHOLD && prox_values.delta[5]>WALL_THRESHOLD){
					inside_maze = TRUE;
					set_rgb_led(0, 0, 0, 0);
					set_rgb_led(1, 0, 0, 0);
					set_rgb_led(2, 0, 0, 0);
					set_rgb_led(3, 0, 0, 0);
				}
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

void junction_scan(void){
	unsigned int left=0;
	unsigned int right=0;

	stopCurrentMelody();

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
		dead_end();
	}
}

void slower(void){//TODO completer
	speed=500;

}

void faster(void){//TODO completer
	speed=MOTOR_SPEED_LIMIT;
}

void end_game_animation(void){
	int i=0;
	bool toggle_led=FALSE;

	for(i=0; i<17; i++){
		set_body_led(toggle_led);
		toggle_led=!toggle_led;

		chThdSleepMilliseconds(125);
	}
}

bool maze_state(void){
	return inside_maze;
}
