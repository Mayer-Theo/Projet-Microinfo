#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "ch.h"
#include "hal.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"

//nos librairies
#include "tof.h"
#include <main.h>
#include "control.h"

#define WALL_STOP_DIST 		50		//distance d'arret au mur
#define SPEED_0				0
#define MAX_TOF_VALUE		110		//distance max de travail de notre tof
#define TRUE				1
#define FALSE				0

//Variable permettant de savoir si un mur est trop proche
static 	bool wall_close=FALSE;

static THD_WORKING_AREA(tof_thread_wa, 256);
static THD_FUNCTION(tof_thread, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	uint16_t wall_distance=0;

	systime_t time;

	while(1){
		wall_distance=VL53L0X_get_dist_mm();

		//effectuer le controle du tof que si le robot n'est pas en pause
		if(pause_state()==FALSE){
			//Permet de signaler qu'il n'y a pas d'obstacle
			if(wall_distance>MAX_TOF_VALUE){
				wall_close=FALSE;
			}

			//Mur dans la zone de travail (wall_distance<MAX_TOF_VALUE)
			else{
				//avancer a vitesse constante
				left_motor_set_speed(speed_value());
				right_motor_set_speed(speed_value());

				wall_close=TRUE;

					//arreter le robot s'il est plus proche que WALL_STOP_DIST d'un mur
					if(wall_distance<WALL_STOP_DIST){
						left_motor_set_speed(SPEED_0);
						right_motor_set_speed(SPEED_0);

						//scan de la jonction pour savoir quoi faire
						junction_scan();
					}
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}
}

//initialisation de la thread du tof
void init_tof_thread(void){
	chThdCreateStatic(tof_thread_wa, sizeof(tof_thread_wa), NORMALPRIO, tof_thread, NULL);
}

//ppermet de savoir si il y a un mur proche ou pas
bool tof_wall_too_close(void){
	return wall_close;
}
