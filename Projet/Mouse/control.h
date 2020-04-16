#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void user_direction_input(void);

void turn_right(void);

void turn_left(void);

void dead_end(void);

void turn(float position, int sense);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
