#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void init_IR_thread(void);

bool maze_state(void);

void set_ignore_junction(bool value);

bool ignore_junction_value(void);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSORS_H */
