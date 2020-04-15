#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void proximity_check(void);

void junction_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSORS_H */
