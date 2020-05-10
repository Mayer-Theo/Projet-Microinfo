#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
* @brief   Lance la thread du tof
*/
void init_tof_thread(void);

/**
* @brief	Permet de savoir s'il y a un mur proche
*
* @return	Retourne la valeur de la variable wall_close, qui est TRUE si un mur est a une distance inférieur à WALL_STOP_DIST
*/
bool tof_wall_too_close(void);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
