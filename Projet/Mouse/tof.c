#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "ch.h"
#include "hal.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"

#include "tof.h"
#include <main.h>
#include "control.h"

#define WALL_STOP_DIST 		60
#define SPEED_0				0
#define ABERRATION_CONTROL	80
#define MIN_TOF_VALUE		20
#define MAX_TOF_VALUE		100
#define TRUE				1
#define FALSE				0

static 	bool wall_close=FALSE;

static THD_WORKING_AREA(tof_sensor_wa, 2048);
static THD_FUNCTION(tof_sensor, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	static uint16_t wall_distance=0;
	static uint16_t temp_wall_distance=0;

	systime_t time;

	while(1){
		wall_distance=VL53L0X_get_dist_mm();

		if(pause_state()==FALSE && wall_distance>MAX_TOF_VALUE){
			wall_close=FALSE;
		}

		//lancement du contrôle de distance uniquement dans une plage de valeurs et pas en pause
		if(pause_state()==FALSE && wall_distance>MIN_TOF_VALUE && wall_distance<MAX_TOF_VALUE){
			right_motor_set_speed(speed_value());
			left_motor_set_speed(speed_value());

			wall_close=TRUE;

			temp_wall_distance=VL53L0X_get_dist_mm();

				//ignorer les valeurs aberrantes (beaucoups plus petite/grande que la valeur précédente)
				if(abs(temp_wall_distance-wall_distance)<ABERRATION_CONTROL && temp_wall_distance>MIN_TOF_VALUE && temp_wall_distance<MAX_TOF_VALUE){
					wall_distance=temp_wall_distance;
				}

				//arreter le robot s'il est plus proche que WALL_STOP_DIST d'un mur
				if(wall_distance<WALL_STOP_DIST){
					left_motor_set_speed(SPEED_0);
					right_motor_set_speed(SPEED_0);

					junction_scan();
				}
		}

		chThdSleepUntilWindowed(time, time + MS2ST(100));
	}
}

void init_tof_thread(void){
	chThdCreateStatic(tof_sensor_wa, sizeof(tof_sensor_wa), NORMALPRIO, tof_sensor, NULL);
}

bool tof_wall_too_close(void){
	return wall_close;
}
