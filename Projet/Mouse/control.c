#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "ch.h"
#include "hal.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "leds.h"
#include "sensors/proximity.h"

//nos librairies
#include <main.h>
#include "tof.h"
#include "IR_sensors.h"
#include "audio_processing.h"
#include"audio/play_melody.h"

#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define TURN_90DEG			3.8	//valeur expérimentale
#define TURN_180DEG			PERIMETER_EPUCK/2
#define TURN				-1	//Permet dans la fonction move de faire une rotation
#define FORWARD				1	//Permet dans la fonction move de faire avancer le robot
#define TURN_LEFT			-1	//permet de modifier le sens de rotation des moteurs pour tourner dans le bon sens
#define TURN_RIGHT			1	//idem que pour TURN_LEFT
#define TURNING_SPEED		500	//vitesse de rotation
#define SPEED_0				0	//vitesse nulle
#define SPEED_FAST			800
#define WALL_THRESHOLD 		80	//seuil de détection des murs par les IR
#define LED2				0
#define LED4				1
#define LED6				2
#define ON					10	//utilisé pour allumer les leds indicatrices hors du labyrinthe avec l'intensité donnée
#define OFF					0	//utilisé pour éteindre les leds indicatrices dans le labyrinthe
#define NB_BLINK			17	//utilisé pour faire clignoter les body leds 8x
#define MAZE_HALF_WIDTH		6	//permet au robot d'avancer dans les jonctions en T (en venant de la barre horizontale) jusqu'à ce qu'il revoit des murs

//drapeau permettant la mise en pause du robot et de ces capteurs à l'exception du micro si ==TRUE, se change avec set_pause(TRUE/FALSE)
static bool pause_flag=FALSE;

//variable permettant de controller la vitesse du robot avec la fonction change_speed()
static int speed=SPEED_FAST;

//variable permettant d'ignorer les jonctions (en T si on vient de la gauche/droite) pour ne pas s'arreter après avoir tourné dans celle-ci
//initialement à vrai pour pas que le robot confonde ça position initiale hors du labyrinthe avec une jonction
static bool ignore_junction = TRUE;

//Permet de faire avancer ou tourner le robot
void move(int direction,float position, int sense){
	int32_t final_l_pos=0;
	int32_t final_r_pos=0;
	int32_t l_pos=0;
	int32_t r_pos=0;

	left_motor_set_pos(0);
	right_motor_set_pos(0);

	final_l_pos=position * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	final_r_pos=direction*position * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	left_motor_set_speed(sense*TURNING_SPEED);
	right_motor_set_speed(direction*sense*TURNING_SPEED);

	l_pos=sense*left_motor_get_pos();
	r_pos=sense*right_motor_get_pos();

	while(l_pos<final_l_pos || r_pos>final_r_pos ){
		l_pos=sense*left_motor_get_pos();
		r_pos=sense*right_motor_get_pos();
	}

	left_motor_set_speed(SPEED_0);
	right_motor_set_speed(SPEED_0);
}

/*///////////////////////////////////
 * 									*
 *			NOS FONCTIONS			*
 * 									*
*////////////////////////////////////

//Permet d'ignorer ou non la prochaine jonction
void set_ignore_junction(bool value){
	ignore_junction=value;
}

//Permet de savoir si la prochaine jonction est a ignorer
bool ignore_junction_value(void){
	return ignore_junction;
}

//Permet de mettre en pause le robot et ses capteurs à l'exception du micro
//Status est la nouvelle valeurs de la pause (TRUE/FALSE)
void set_pause(bool new_status){
	pause_flag=new_status;

	//si on met le robot en pause, arreter les moteurs
	if(new_status==TRUE){
		right_motor_set_speed(SPEED_0);
		left_motor_set_speed(SPEED_0);
	}
}

//Allume ou éteint les led 2 à 4 en rouge. La valeurs on_off définit l'intensité des leds
void out_of_maze_indicator(int on_off){
	set_rgb_led(LED2, on_off, 0, 0);
	set_rgb_led(LED4, on_off, 0, 0);
	set_rgb_led(LED6, on_off, 0, 0);
}

//Lance toutes les threads nécessaires au fonctionnement du robot
void init_threads(void){
	set_pause(TRUE);			//démarrage du robot en mode pause
	out_of_maze_indicator(ON);	//allumer les leds indicatrices

	init_tof_thread();		//lance thread du tof
	init_IR_thread();		//lance thread des capteurs IR
    init_sound_thread();	//lance thread d'analyse du son
}

//Joue une mélodie et met le robots en pause
//Jouer une fréquence de commande pour relancer le robot
void user_direction_input(void){
	playMelody(WALKING, ML_SIMPLE_PLAY, NULL);
	set_pause(TRUE);
}

//Tourne a gauche/droite de 90°
void right_angle_turn(int direction){
	move(TURN, TURN_90DEG, direction);

	//Dans les jonction en T (en venant de la gauche/droite) permet d'avancer jusqu'à ce que le robot revoit des murs, sinon cas confondu avec un end_game (plus de murs latéral ou devant)
	if(ignore_junction_value()==FALSE && pause_flag==TRUE){
		set_ignore_junction(TRUE);					//ignorer le vide sur les côtés
		move(FORWARD, MAZE_HALF_WIDTH, FORWARD);	//avancer
	}
}

//Fonction utilisée dans les culs de sac pour faire demi-tour (rotation 180°)
void dead_end(void){
	move(TURN, TURN_180DEG, TURN_RIGHT);
}

//Permet de savoir si le robot est en pause ou non en retournant la valeur du drapeau correspondant
bool pause_state(void){
	return pause_flag;
}

//Permet de modifier la vitesse
void change_speed(int new_speed){
	speed=new_speed;
}

//Permet de connaitre la vitesse choisie du robot en retournant speed
int speed_value(void){
	return speed;
}

//Animation de fin du labyrinthe
void end_game_animation(void){
	int i=0;
	bool toggle_led=FALSE;		//variable pour faire clignoter les body leds

	set_pause(TRUE);			//mise en pause du robot
	out_of_maze_indicator(ON);	//allumer les led indicatrices hors du labyrinthe

	//boucle pour faire clignoter les body leds
	for(i=0; i<NB_BLINK; i++){
		set_body_led(toggle_led);
		toggle_led=!toggle_led;

		chThdSleepMilliseconds(125);
	}
}

//Fonction appelé pour différencier les virages, les intersections et culs de sac puis effectuer l'action nécessaire
void junction_scan(void){
	unsigned int left_IR=0;	//variable utilisée pour savoir s'il y a un mur à gauche avec les capteurs IR
	unsigned int right_IR=0;	//variable utilisée pour savoir s'il y a un mur à droite avec les capteurs IR

	//enregistrer les valeurs des capteurs IR gauche et droite
	left_IR=get_calibrated_prox(5);
	right_IR=get_calibrated_prox(2);

	set_ignore_junction(TRUE);

	if(left_IR<WALL_THRESHOLD && right_IR<WALL_THRESHOLD){		//S'il n'y a pas de murs attendre une instruction de l'utilisateur
		user_direction_input();
	}
	else if(left_IR>WALL_THRESHOLD && right_IR<WALL_THRESHOLD){	//S'il y'a pas de murs a droite et un murs a gauche, tourner à droite
		right_angle_turn(TURN_RIGHT);
	}
	else if(left_IR<WALL_THRESHOLD && right_IR>WALL_THRESHOLD){	//S'il y'a pas de murs a gauche et un murs a droite, tourner à gauche
		right_angle_turn(TURN_LEFT);
	}
	else if(left_IR>WALL_THRESHOLD && right_IR>WALL_THRESHOLD){	//S'il y a un mur a gauche et a droite, faire demi-tour
		dead_end();
	}
}
