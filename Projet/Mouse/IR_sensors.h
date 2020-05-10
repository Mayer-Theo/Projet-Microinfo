#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
* @brief   Lance la thread des capteurs IR
*/
void init_IR_thread(void);

#ifdef __cplusplus
}
#endif

#endif /* IR_SENSORS_H */
