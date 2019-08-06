#ifndef __ANO_ALTCTRL_2_H
#define __ANO_ALTCTRL_2_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"

//==定义
typedef struct
{
	//
	float target_vel_fixup_cmps;
	float target_acc_fixup_cmpss;
	//
	float expect_hei;
	float expect_vel;
	float expect_acc;
	//
	float feedback_hei;
	float feedback_vel;
	float feedback_acc;
	//
	float e_hei;
	float e_vel;
	float e_acc;
	float ei_acc;
	//
	float adj_b;
	float obs_twr;
	float out_u;

}_ano_alt_ctrl_st;

//==数据声明
_ano_alt_ctrl_st alt_ctrl;

//==函数声明

//static
void ANO_TargetVal_FixUp(u8 *dT_ms,float target_alt_vel_cmps);
void ANO_Alt_Ctrl(u8 *dT_ms);

//public
void ANO_AltCtrl_Task(u8 dT_ms);


#endif

