#ifndef _DRV_PWMOUT_H_
#define _DRV_PWMOUT_H_

#include "sysconfig.h"

void Drv_PwmOutInit(void);
void Drv_MotorPWMSet(uint8_t Motor,uint16_t PwmValue);
void Drv_HeatSet(u16 val);
	
#endif
