#ifndef __ANO_OPMV_CTRL_H
#define __ANO_OPMV_CTRL_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"

//==定义

//==数据声明
typedef struct
{
	u8 en;
	u8 height_flag;
	u8 reset_flag;
}_opmv_ct_sta_st;

//==函数声明
extern _opmv_ct_sta_st opmv_ct_sta;

//static

//public
void ANO_OPMV_Ctrl_Task(u8 dT_ms);



#endif
