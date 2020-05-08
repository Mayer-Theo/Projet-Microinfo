#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "sensors/proximity.h"
#include "stdbool.h"
#include "leds.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"
#include "audio/play_melody.h"

//nos librairies
#include <main.h>
#include "IR_sensors.h"
#include "control.h"
#include "tof.h"

#define WALL_THRESHOLD 			80
#define SPEED_0					0
#define TRUE					1
#define FALSE					0
#define ON						10
#define OFF						0
#define INTERSECTION_DISTANCE	1	//estimation du rayon de l'epuck
#define FORWARD					1
#define BACKWARD				-1

//variable indicatrice, Vrai si le robot est dans le labyrinthe
static bool inside_maze = FALSE;

static bool ignore_junction = TRUE;

static THD_WORKING_AREA(proximity_scan_wa, 2048);
static THD_FUNCTION(proximity_scan, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	int16_t left_IR=0;
	int16_t right_IR=0;

	systime_t time;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	int16_t leftSpeed = 0, rightSpeed = 0;

	while(1) {
		time = chVTGetSystemTime();
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

		if(pause_state()==FALSE){

			//autopilote seulement si pas d'obstacle
			if(tof_wall_too_close()==FALSE){
				leftSpeed = speed_value() - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = speed_value() - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);

				left_IR=prox_values.delta[5];
				right_IR=prox_values.delta[2];

				if(right_IR<WALL_THRESHOLD && left_IR<WALL_THRESHOLD && inside_maze==TRUE){
					inside_maze = FALSE;
					end_game_animation();
				}
				else if(right_IR>WALL_THRESHOLD && left_IR>WALL_THRESHOLD){
					inside_maze = TRUE;
					no_maze_led_indicator(OFF);
					set_ignore_junction(FALSE);
				}
				//================================================
				//================================================
				else if(ignore_junction==FALSE && ((right_IR<WALL_THRESHOLD && left_IR>WALL_THRESHOLD) || (right_IR>WALL_THRESHOLD && left_IR<WALL_THRESHOLD))){
				//else if(inside_maze==TRUE && ((prox_values.delta[2]>WALL_THRESHOLD) != (prox_values.delta[5]>WALL_THRESHOLD))){
					advance(INTERSECTION_DISTANCE, FORWARD);
					user_direction_input();
				}
				//================================================
				//================================================
			}
		}

		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}
}

void init_IR_thread(void){
	chThdCreateStatic(proximity_scan_wa, sizeof(proximity_scan_wa), NORMALPRIO, proximity_scan, NULL);
}

bool maze_state(void){
	return inside_maze;
}

//================================================
//================================================
void set_ignore_junction(bool value){
	ignore_junction=value;
}

bool ignore_junction_value(void){
	return ignore_junction;
}
