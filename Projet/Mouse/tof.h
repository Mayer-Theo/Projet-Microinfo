#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void init_tof_thread(void);

bool tof_wall_too_close(void);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
