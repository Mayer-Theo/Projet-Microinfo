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
#include "chprintf.h"

//nos librairies
#include <main.h>
#include "IR_sensors.h"
#include "control.h"
#include "tof.h"

#define WALL_THRESHOLD 			80
#define SPEED_0					0
#define TRUE					1
#define FALSE					0
#define ON						10	//utiliser pour allumer les leds indicatrices avec l'intensité assignée ici
#define OFF						0
#define MAZE_HALF_WIDTH			1.5	//utilisé pour avancer jusqu'au centre des jonctions en T
#define FORWARD					1
#define BACKWARD				-1
#define IR1						0
#define IR2						1
#define IR3						2
#define IR6						5
#define IR7						6
#define IR8						7
#define TURN					-1	//utilisé pour faire tourner le robot avec la fonction move()
#define TURN_LEFT				-1	//permet de modifier le sens de rotation des moteurs pour tourner dans le bon sens dans la fonction move()
#define TURN_RIGHT				1	//idem que pour TURN_LEFT
#define CORRECTION_ANGLE		0.2	//Permettet de corriger la dérive angulaire causée par l'autopilote dans le jonction en T par le manque de mur

//thread gérant les capteurs IR
static THD_WORKING_AREA(IR_thread_wa, 2048);
static THD_FUNCTION(IR_thread, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	int16_t left_IR=0;
	int16_t right_IR=0;

	//variable permettant d'allumer/éteindre le jeu, utile par exemple pour ne pas faire une end_game alors qu'on est pas encore rentré dans le labyrinthe
	bool game_on = FALSE;

	systime_t time;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	int16_t leftSpeed = 0, rightSpeed = 0;

	while(1) {
		time = chVTGetSystemTime();
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

		//Seulement tester les capteurs IR si le robot n'est pas en pause
		if(pause_state()==FALSE){

			//autopilote seulement si pas d'obstacle
			if(tof_wall_too_close()==FALSE){
				//pilote automatique utilisant l'intensité réfléchie pour dévier le robot
				leftSpeed = speed_value() - prox_values.delta[IR1]*2 - prox_values.delta[IR2];
				rightSpeed = speed_value() - prox_values.delta[IR8]*2 - prox_values.delta[IR7];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);

				//Capteur IR gauche et droite
				left_IR=prox_values.delta[IR6];
				right_IR=prox_values.delta[IR3];

				//Si on est entré dans un labyrinthe et qu'on ne voit plus de murs-->on a terminé le labyrinthe
				if(right_IR<WALL_THRESHOLD && left_IR<WALL_THRESHOLD && game_on==TRUE){
					game_on = FALSE;		//fin du labyrinthe
					end_game_animation();	//animation de fin
				}
				//Si on voit des murs a gauche et a droite on entre dans le labyrinthe ou on a fini de traverser une jonction
				else if(right_IR>WALL_THRESHOLD && left_IR>WALL_THRESHOLD){
					game_on = TRUE;					//rentre dans un labyrinthe
					out_of_maze_indicator(OFF);		//Pas de leds allumées dans le labyrinthe
					set_ignore_junction(FALSE);		//on a fini de traversé la jonction--> ne plus ignorer le prochain "manque" de mur
				}
				//Si on ne doit pas ignorer de jonction et qu'il manque un mur a gauche-->c'est une jonction en T depuis la droite
				else if(ignore_junction_value()==FALSE && (right_IR<WALL_THRESHOLD && left_IR>WALL_THRESHOLD)){
					move(FORWARD, MAZE_HALF_WIDTH, FORWARD);	//avancer jusqu'au milieu de la jonction
					move(TURN, CORRECTION_ANGLE, TURN_LEFT);					//correction de la dérive angulaire
					user_direction_input();						//attendre une instruction utilisateur
				}
				//Si on ne doit pas ignorer de jonction et qu'il manque un mur a droite-->c'est une jonction en T depuis la gauche
				else if(ignore_junction_value()==FALSE && (right_IR>WALL_THRESHOLD && left_IR<WALL_THRESHOLD)){
					move(FORWARD, MAZE_HALF_WIDTH, FORWARD);	//avancer jusqu'au milieu de la jonction
					move(TURN, CORRECTION_ANGLE, TURN_RIGHT);				//correction de la dérive angulaire
					user_direction_input();						//attendre une instruction utilisateur
				}
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}
}

//création de la thread capteur IR
void init_IR_thread(void){
	chThdCreateStatic(IR_thread_wa, sizeof(IR_thread_wa), NORMALPRIO, IR_thread, NULL);
}
