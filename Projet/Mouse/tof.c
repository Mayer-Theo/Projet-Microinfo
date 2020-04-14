#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "tof.h"
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"

#define WALL_STOP_DIST 		80
#define SPEED_0				0
#define CRUISING_SPEED		500

//valeurs obtenue par expérience pour fonctionnement du tof
#define ABERRATION_CONTROL	470
#define MIN_TOF_VALUE		30
#define MAX_TOF_VALUE		500

static THD_WORKING_AREA(tof_sensor_wa, 2048);
static THD_FUNCTION(tof_sensor, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	static uint16_t wall_distance=0;
	static uint16_t temp_wall_distance=0;

	left_motor_set_speed(CRUISING_SPEED);
	right_motor_set_speed(CRUISING_SPEED);

	while(wall_distance<MIN_TOF_VALUE || wall_distance>MAX_TOF_VALUE){
		wall_distance=VL53L0X_get_dist_mm();
	}

	while(1){
		temp_wall_distance=VL53L0X_get_dist_mm();

		chprintf((BaseSequentialStream *)&SD3, "check 1\r\n");
		chprintf((BaseSequentialStream *)&SD3, "%d-%d\r\n\n", wall_distance, temp_wall_distance);

		if(abs(temp_wall_distance-wall_distance)<ABERRATION_CONTROL && temp_wall_distance>MIN_TOF_VALUE && temp_wall_distance<MAX_TOF_VALUE){
			wall_distance=temp_wall_distance;
		}

		if(wall_distance<WALL_STOP_DIST){
			left_motor_set_speed(SPEED_0);
			right_motor_set_speed(SPEED_0);
		}
		else{
			left_motor_set_speed(CRUISING_SPEED);
			right_motor_set_speed(CRUISING_SPEED);
		}
	}
}

void tof_scan(void){
	chThdCreateStatic(tof_sensor_wa, sizeof(tof_sensor_wa), NORMALPRIO, tof_sensor, NULL);
}
