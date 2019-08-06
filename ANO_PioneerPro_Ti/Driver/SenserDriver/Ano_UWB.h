#ifndef _ANO_UWB_H_
#define _ANO_UWB_H_

//==引用

//
#include "Ano_Filter.h"
#include "Ano_Math.h"
#include "Ano_Imu.h"
#include "Ano_FcData.h"
//==定义
typedef struct
{
	u8 init_ok;
	u8 online;
	
	float ref_dir[2];
	float raw_data_loc[3];
	float raw_data_vel[3];	
	float w_dis_cm[3];
	float w_vel_cmps[3];

}_uwb_data_st;


//==数据声明
extern _uwb_data_st uwb_data;


//==函数声明
void Ano_UWB_Get_Byte(u8 data);
void Ano_UWB_Get_Data_Task(u8 dT_ms);
void Ano_UWB_Data_Calcu_Task(u8 dT_ms);





#endif





