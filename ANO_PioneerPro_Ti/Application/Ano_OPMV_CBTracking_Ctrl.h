#ifndef __ANO_OPMV_CBTRACKING_CTRL_H
#define __ANO_OPMV_CBTRACKING_CTRL_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"

//==定义
typedef struct
{
	//
	u8 target_loss;	
	//
	s16 opmv_pos[2];
	s16 rp2pixel_val[2];
//	s16 rp2p_fifo_lagging[3][2];

	//
	float decou_pos_pixel[2];
	float ground_pos_err_h_cm[2];
	float ground_pos_err_d_h_cmps[2];
	
	float target_gnd_velocity_cmps[2];
	//
	float exp_velocity_h_cmps[2];
}_ano_opmv_cbt_ctrl_st;

//==数据声明
extern _ano_opmv_cbt_ctrl_st ano_opmv_cbt_ctrl;

//==函数声明

//static
static void ANO_CBTracking_Decoupling(u8 *dT_ms,float rol_degs,float pit_degs);
static void ANO_CBTracking_Calcu(u8 *dT_ms,s32 relative_height_cm);
//public
void ANO_CBTracking_Task(u8 dT_ms);
void ANO_CBTracking_Ctrl(u8 *dT_ms,u8 en);




#endif
