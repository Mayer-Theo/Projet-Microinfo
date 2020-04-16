#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "tof.h"
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"
#include "IR_sensors.h"

#define WALL_STOP_DIST 		80
#define SPEED_0				0

//valeurs obtenue par expérience pour fonctionnement du tof
#define ABERRATION_CONTROL	470
#define MIN_TOF_VALUE		30
#define MAX_TOF_VALUE		500

static THD_WORKING_AREA(tof_sensor_wa, 2048);
static THD_FUNCTION(tof_sensor, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	msg_t msg=0; //TODO check if used correctly to end thread

	static uint16_t wall_distance=0;
	static uint16_t temp_wall_distance=0;

	systime_t time;

	while(wall_distance<MIN_TOF_VALUE || wall_distance>MAX_TOF_VALUE){
		wall_distance=VL53L0X_get_dist_mm();
		chThdSleepUntilWindowed(time, time + MS2ST(100));
	}

	while(1){
		temp_wall_distance=VL53L0X_get_dist_mm();

		if(abs(temp_wall_distance-wall_distance)<ABERRATION_CONTROL && temp_wall_distance>MIN_TOF_VALUE && temp_wall_distance<MAX_TOF_VALUE){
			wall_distance=temp_wall_distance;
		}

		if(wall_distance<WALL_STOP_DIST){
			left_motor_set_speed(SPEED_0);
			right_motor_set_speed(SPEED_0);

			junction_scan();
			chThdExit(msg);		//TODO mise en pause de la thread plutôt? Ou variable globale de contrôle? différence avec chThdExits? Que doit contenir msg ?=====================================================================================================
		}

		chThdSleepUntilWindowed(time, time + MS2ST(100));
	}
}

void tof_scan(void){
	chThdCreateStatic(tof_sensor_wa, sizeof(tof_sensor_wa), NORMALPRIO, tof_sensor, NULL);
}
