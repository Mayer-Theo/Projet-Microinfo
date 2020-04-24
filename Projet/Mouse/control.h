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

void set_pause(void);

void set_play(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
