#ifndef _DRV_RCIN_H_
#define _DRV_RCIN_H_

#include "sysconfig.h"

typedef struct
{
    int16_t Roll;
    int16_t Pitch;
    int16_t Throttle;
    int16_t Yaw;
    int16_t Aux1;
    int16_t Aux2;
    int16_t Aux3;
    int16_t Aux4;
    int16_t Aux5;
    int16_t Aux6;
    int16_t Aux7;
    int16_t Aux8;
    int16_t Aux9;
    int16_t Aux10;
    int16_t Aux11;
    int16_t Aux12;
} RCData_t;
union PPM
{
    uint16_t Captures[16];
    RCData_t Data;
} ;

//
extern union PPM  RC_PPM;
extern u16 Rc_Sbus_In[16];

void Drv_PpmInit(void);
void Drv_SbusInit(void);

#endif
