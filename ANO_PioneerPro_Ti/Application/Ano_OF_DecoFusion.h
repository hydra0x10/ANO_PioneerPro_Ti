#ifndef __ANO_OF_DECOFUSION_H
#define __ANO_OF_DECOFUSION_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"

//==定义
typedef struct
{
	u8 updata;
	u8 online;
	s16 flow_x_integral;
	s16 flow_y_integral;	
	
	u16 it_ms;//integration_timespan;//dT_us
	u8 valid;//0xf5:ok  else:no
	
}_of_data_st;

	
typedef struct
{
	//sta
	u8 state; //0:init,1:normal,2:cali
	u8 quality;
	u8 valid;
	//rd
	float of_ref_height;
	float of_vel[2];
	
	
	//obs
	float gnd_vel_obs_h[2];
	float gnd_vel_obs_w[2];	
	
	//fus
	float gnd_acc_est_w[2];
	float gnd_vel_est_w[2];
	float gnd_vel_est_h[2];
	//
	float gnd_vel_fixout_w[2];	
	float gnd_vel_fixout_h[2];	
	//	
} _of_rdf_st;


//==数据声明
extern float of_rot_d_degs[];
extern _of_data_st of_data;
extern _of_rdf_st of_rdf;

//==函数声明

//static
static void ANO_OF_Data_Get(float *dT_s,u8 *of_data_buf);
static void OF_INS_Get(float *dT_s,float rad_ps_x,float rad_ps_y,float acc_wx,float acc_wy);
static void ANO_OF_Decouple(u8 *dT_ms);
static void ANO_OF_Fusion(u8 *dT_ms,s32 ref_height_cm);
static void OF_State(void);
static void OF_INS_Reset(void);
//public
void ANO_OF_Data_Prepare_Task(float dT_s);
void ANO_OFDF_Task(u8 dT_ms);




#endif
