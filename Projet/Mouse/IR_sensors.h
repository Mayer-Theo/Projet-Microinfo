#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void init_IR_thread(void);

void junction_scan(void);

void change_speed(void);

void faster(void);

void slower(void);

void end_game_animation(void);

bool maze_state(void);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSORS_H */
