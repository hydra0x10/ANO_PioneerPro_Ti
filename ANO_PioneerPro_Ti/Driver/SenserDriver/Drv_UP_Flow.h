#ifndef __DRV_UP_Flow_H
#define __DRV_UP_Flow_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"

//==定义

//==数据声明
extern uint8_t of_buf_update_cnt;
extern uint8_t OF_DATA[];
//==函数声明

//static


//public
u8 Drv_OFInit(void);
void OFGetByte(uint8_t data);

#endif

