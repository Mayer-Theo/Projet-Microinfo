#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//fonction de lancement des threads
void init_threads(void);

void user_direction_input(void);

void right_angle_turn(int direction);

void dead_end(void);

void move(int direction, float position, int sense);

void set_pause(bool);

bool pause_state(void);

void change_speed(int new_speed);

void end_game_animation(void);

int speed_value(void);

void junction_scan(void);

void out_of_maze_indicator(int intensity);

void set_ignore_junction(bool value);

bool ignore_junction_value(void);

void set_speed_was_fast(bool answer);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
