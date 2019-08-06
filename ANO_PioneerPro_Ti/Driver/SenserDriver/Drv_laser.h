#ifndef _LASER_H_
#define _LASER_H_
#include "sysconfig.h"

extern u8 LASER_LINKOK;
extern u16 Laser_height_cm;

u8 		Drv_Laser_Init(void);
void 	Drv_Laser_GetOneByte(u8 data);

#endif
