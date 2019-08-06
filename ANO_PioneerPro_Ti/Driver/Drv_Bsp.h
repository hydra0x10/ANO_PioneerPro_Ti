#ifndef _DRV_BSP_H_
#define _DRV_BSP_H_

#include "sysconfig.h"

void Drv_BspInit(void);
void MyDelayMs(u32 time);
void SysTick_Init(void );
uint32_t GetSysRunTimeMs(void);
uint32_t GetSysRunTimeUs(void);
extern u8 of_init_type;
#endif
