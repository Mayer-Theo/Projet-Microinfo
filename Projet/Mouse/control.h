#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//fonction de lancement des threads
void init_threads(void);

void user_direction_input(void);

void turn_right(void);

void turn_left(void);

void dead_end(void);

void turn(float position, int sense);

void set_pause(bool);

bool pause_state(void);

void slower(void);

void faster(void);

void end_game_animation(void);

int speed_value(void);

void junction_scan(void);

void no_maze_led_indicator(int intensity);

void advance(float position, int sense);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
