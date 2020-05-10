#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
* @brief   	Lance toutes les threads et initialise les variables de démarrage
*/
void init_threads(void);

/**
* @brief   	Met le robot en pause et joue une mélodie
*/
void user_direction_input(void);

/**
* @brief   	Fait faire une rotation de 90° au robot
*
* @param direction	permet de controler le sense de rotation 1 pour horaire, -1 pour anti-horaire
*/
void right_angle_turn(int direction);

/**
* @brief   	Faire demi tour et continuer la suite du programme
*/
void dead_end(void);

/**
* @brief   	Permet de faire avancer ou tourner le robot d'une distance/angle précis
*
* @param 	direction	-1 pour faire tourner le robot et 1 pour le faire avancer
* 			position	distance/rotation a effectuer
* 			sense		sense d'avance/rotation 1 pour avant/horraire et -1 pour reculer/anti-horraire
*/
void move(int direction, float position, int sense);

/**
* @brief   	Met le robot en pause, stoppe la prise de mesure des capteurs
*
* @param	new_status	1 pour pause, 0 pour play
*/
void set_pause(bool);

/**
* @brief   	Permet de savoir si le robot est en pause
*
* @return	1 pour pause, 0 pour play
*/
bool pause_state(void);

/**
* @brief   	Changer la vitesse du robot
*
* @param	new_speed	nouvelle vitesse du robot
*/
void change_speed(int new_speed);

/**
* @brief   	Annimation de fin du robot (clignotement des leds) et mise en pause
*/
void end_game_animation(void);

/**
* @brief   	Permet de connaitre la vitesse actuelle du robot
*
* @return	Vitesse du robot
*/
int speed_value(void);

/**
* @brief   	Identifie le type de jonction et agit en conséquence
*/
void junction_scan(void);

/**
* @brief   	Permet de controller l'intensité de 3 leds indicatrice sur le robot
*
* @param	intensity	nouvelle intensité des 3 leds
*/
void out_of_maze_indicator(int intensity);

/**
* @brief   	Permet de dire au robot d'ignorer la prochaine jonction
*
* @param	value	1 pour ignorer la prochaine jonction, 0 pour ne pas l'ignorer
*/
void set_ignore_junction(bool value);

/**
* @brief   	Permet de savoir s'il faut ignorer la prochaine jonction ou non
*
* @return	1 pour ignorer la prochaine jonction, 0 pour ne pas l'ignorer
*/
bool ignore_junction_value(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
