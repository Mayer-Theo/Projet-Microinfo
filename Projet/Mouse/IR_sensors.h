#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void init_IR_thread(void);

void junction_scan(void);

void change_speed(void);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSORS_H */
